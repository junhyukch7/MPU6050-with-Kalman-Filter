#include <ros.h>
#include <MsgMpu6050.h>
#include <Wire.h>

ros::NodeHandle  nh;
mpu6050::MsgMpu6050 msg;

ros::Publisher sensor("sensor", &msg);

const int MPU_addr = 0x68; 
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;// 가속도센서 = 가속도, 자이로센서 = 각속도

double angleAcX,angleAcY,angleAcZ;
double angleGyX,angleGyY,angleGyZ;

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


// 가속도 센서를 이용하여 가속도 구하기
void AccXY(){
  // g = 16384;
  getRawData();
  angleAcX = AcX/16384.0;
  angleAcY = AcY/16384.0;
  angleAcZ = AcZ/16384.0;
}
// 자이로 센서를 이용하여 각속도 구하기(p,q,r)
void GyroXYZ(){
  getRawData();
  angleGyX = GyX/131; // 131deg roatae per second
  angleGyY = GyY/131;
  angleGyZ = GyZ/131;
}


void loop()
{
  AccXY();
  GyroXYZ();
 
  msg.AcX = angleAcX;
  msg.AcY = angleAcY;
  msg.AcZ = angleAcZ;

  msg.GyX = angleGyX;
  msg.GyY = angleGyY;
  msg.GyZ = angleGyZ;
  
  sensor.publish( &msg );
  nh.spinOnce();
  delay(333);
}
