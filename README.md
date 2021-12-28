# MPU6050 with Kalman Filter
Kalman Filter based Sensor Fusion Algorithm (For Raspberry Pi and Arduino Serial Communication) 

The Arduino Uno is sending(publish) Acceleration and angular velocity for each X-Y-Z axis via rosserial. C++ script is subscriber which recieve the data. After then process IMU data which is consists of gyro and acclerometer data by Kalman Filter.

---
## Execution Environment
1. Ubuntu Linux 18.04 LTS
2. Raspberry Pi
3. Arduino
4. MPU-6050
---
## Prerequisite package & Library

1. Eigen Library(C++)
2. ROS
3. rosserial
---
## Deployment
1. Connect Arduino with MPU-6050
2. Connect Aurdino to Raspberry Pi
3. In Arduino IDE, uploading Sensor.ino
4. In terminal input following command in order. At the second command, port would be diffrent depend on your setting.

```
roscore
```
``` 
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=115200
```
``` 
rosrun mpu6050 topic_subscriber
```
---
## Algorithm PipeLine(Kalman Filter)

1. Setting initial value for caculating 

```c++
Kalman(){
     mpu6050 = nh.subscribe("sensor",100,&Kalman::msgCallback,this);
     mpu_pub = nh.advertise<mpu6050::MsgPose>("/pose",100);
     
     I = Matrix4d::Identity();
     H = I; // obsevation Matrix(output matrix)
     Q = 1.0*I; // system covariance, design parameter
     R = 0.001*I; // estimation covariance , design parameter
     x(0,0) = 1.0; x(1,0) = 0.0; x(2,0) = 0.0; x(3,0) = 0.0; // state variable(quaternion)
     P = I; // error covariance
   }
```
2. Init system matrix

```c++
void initSystemMatrix(){
     Matrix4d temp; //GyX == p, GyY == q, GyZ ==r;
     // this Matrix change gyro to quaternion.
     temp(0,0) = 0.0; temp(0,1) = -GyX; temp(0,2) = -GyY; temp(0,3) = -GyZ;
     temp(1,0) = GyX; temp(1,1) = 0.0; temp(1,2) = GyZ; temp(1,3) = -GyY;
     temp(2,0) = GyY; temp(2,1) = -GyZ; temp(2,2) = 0.0; temp(2,3) = GyX;
     temp(3,0) = GyZ; temp(3,1) = GyY; temp(3,2) = -GyX; temp(3,3) = 0.0;
     A = temp*dt*1/2 + I; // for dicrete system 
   }
```

3. EuelrAceel & EulerToQuaternion

```c++
void EulerAccel(){
     phi = atan(AcY/AcZ);
     theta = atan(AcX/sqrt(pow(AcY,2) + pow(AcZ,2)));
     psi = 0;
   } // to use Euler to Quternion formula, First we need to change Accel to Euler Angle.
   
   void EulerToQuaternion(){
     // this result is z(phi,theta,psi) sholud be get int to algorithm
     z(0,0) = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
     z(1,0) = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
     z(2,0) = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
     z(3,0) = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);
   } // using Euelr to Quternion formula
```

4. Kalman Filter algorithm

```c++
 void algorithm(){
     Vector4d xp;
     Matrix4d Pp;
     Matrix4d Ki;
     Matrix4d K;
     
     //kalman filter algorithm
     xp = A*x;
     Pp = A*P*(A.transpose().eval()) + Q; // aliasing problem(eval())
   
     Ki = H*Pp*(H.transpose().eval()) + R; // aliasing problem(eval())
     Ki = Ki.inverse().eval();
   
     K = Pp*(H.transpose().eval()) * Ki; // aliasing problem(eval())
   
     x = xp + K*(z - H*xp); // x = [q1 q2 q3 q4]'
     P = Pp - K*H*Pp;
     
     //changing quaternion estimation value to euelr angle
     phi = atan2((2*(x(2,0)*x(3,0) + x(0,0)*x(1,0))) , (1 - 2*(pow(x(1,0),2) + pow(x(2,0),2)))) * 180/PI;
     theta = -asin(2*(x(1,0)*x(3,0) - x(0,0)*x(2,0))) * 180/PI;
     // psi(yaw) is not correct
   }
```
---
## Result

<p align="center"><img src = "https://github.com/junhyukch7/MPU6050-with-Kalman-Filter/blob/main/roll.png" width = "80%">
<p align="center">(Figure.1) Roll with Kalman Filter
  
<p align="center"><img src = "https://github.com/junhyukch7/MPU6050-with-Kalman-Filter/blob/main/pitch.png" width = "80%">
<p align="center">(Figure.2) Pitch with Kalman Filter
  
For testing the algorithm accurancy, I was rotateing mpu6050 about -90 to 90 degree for each axis(X-Y). As you can see at the Figure, algorithm estimation values show high accurancy. In case of Roll, It showed high accuracy both in the horizontal and in the rotating situation.
But in case of Pitch, there is a little bit error when it was horizontal(3.xx degree). Because When I tested IMU data, the x-axis gyro showed an error of -9 even in the horizontal state. For this reason, It shows a little error in horizontal.
  

