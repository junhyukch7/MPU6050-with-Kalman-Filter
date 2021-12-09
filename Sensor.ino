#include <ros.h>
#include <MsgMpu6050.h>
#include <Wire.h>

ros::NodeHandle  nh;
mpu6050::MsgMpu6050 msg;

ros::Publisher sensor("sensor", &msg);

const int MPU_addr = 0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

double accAcX,accAcY,accAcZ;
double rateGyX,rateGyY,rateGyZ;

void initSensor(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);// I2C address of mpu6050
  Wire.write(0x6B); // comunicate start by allocateing address at 0x6B 
  Wire.write(0); // Stand by mode of mpu6050
  Wire.endTransmission(true);
}


void setup()
{
  initSensor();
  nh.initNode();
  nh.advertise(sensor);
  Serial.begin(115200); //57600
}


void getRawData(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}


// caculate
void AccXY(){
  // g = 16384;
  getRawData();
  accAcX = AcX/16384.0;
  accAcY = AcY/16384.0;
  accAcZ = AcZ/16384.0;
}
// caculate rate 
void GyroXYZ(){
  getRawData();
  rateGyX = GyX/131.0; // 131deg roatae per second
  rateGyY = GyY/131.0;
  rateGyZ = GyZ/131.0;
}


void loop()
{
  AccXY();
  GyroXYZ();
 
  msg.AcX = accAcX;
  msg.AcY = accAcY;
  msg.AcZ = accAcZ;

  msg.GyX = rateGyX;
  msg.GyY = rateGyY;
  msg.GyZ = rateGyZ;
  
  sensor.publish( &msg );
  nh.spinOnce();
  delay(333);
}
