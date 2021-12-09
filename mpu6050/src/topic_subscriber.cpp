// pitch value seem like correct but roll is totally weird
#include "ros/ros.h"
#include "mpu6050/MsgMpu6050.h"
#include "mpu6050/MsgPose.h"
#include "math.h"
#include <Eigen/Dense> // for inverse matrix
#define PI 3.141592
#define dt 0.333

using namespace Eigen;

class Kalman{
private:
   double AcX,AcY,AcZ,GyX,GyY,GyZ;
   double phi,theta,psi;
   
   Matrix4d A; Matrix4d H; 
   Matrix4d Q; Matrix4d R;
   Matrix4d I; Matrix4d P;
   Vector4d z; Vector4d x;
   
   ros::NodeHandle nh;
   ros::Subscriber mpu6050;
   ros::Publisher mpu_pub;
   
   mpu6050::MsgPose pose;
   
public:
   Kalman(){
     mpu6050 = nh.subscribe("sensor",100,&Kalman::msgCallback,this);
     mpu_pub = nh.advertise<mpu6050::MsgPose>("/pose",100);
     
     I = Matrix4d::Identity();
     H = I;
     Q = 1.0*I;//9.333 if Q is rising then pitch get close to 0 but roll is go to negative 
     R = 0.001*I; // 0.007
     x(0,0) = 1.0; x(1,0) = 0.0; x(2,0) = 0.0; x(3,0) = 0.0;
     P = I; 
   }
   void initSystemMatrix(){
     Matrix4d temp; //GyX == p, GyY == q, GyZ ==r;
     temp(0,0) = 0.0; temp(0,1) = -GyX; temp(0,2) = -GyY; temp(0,3) = -GyZ;
     temp(1,0) = GyX; temp(1,1) = 0.0; temp(1,2) = GyZ; temp(1,3) = -GyY;
     temp(2,0) = GyY; temp(2,1) = -GyZ; temp(2,2) = 0.0; temp(2,3) = GyX;
     temp(3,0) = GyZ; temp(3,1) = GyY; temp(3,2) = -GyX; temp(3,3) = 0.0;
     A = temp*dt*1/2 + I;
   }
   
   void EulerAccel(){
     phi = atan(AcY/AcZ);
     theta = atan(AcX/sqrt(pow(AcY,2) + pow(AcZ,2)));
     psi = 0;
   }
   
   void EulerToQuaternion(){
     // this result is z(phi,theta,psi) sholud be get int to algorithm
     z(0,0) = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
     z(1,0) = sin(phi/2)*cos(theta/2)*cos(psi/2) - cos(phi/2)*sin(theta/2)*sin(psi/2);
     z(2,0) = cos(phi/2)*sin(theta/2)*cos(psi/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
     z(3,0) = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*sin(theta/2)*cos(psi/2);
   }
   
   void algorithm(){
     // kalman filter algorithm
   
     Vector4d xp;
     Matrix4d Pp;
     Matrix4d Ki;
     Matrix4d K;
   
     xp = A*x;
     Pp = A*P*(A.transpose().eval()) + Q;
   
     Ki = H*Pp*(H.transpose().eval()) + R;
     Ki = Ki.inverse().eval();
   
     K = Pp*(H.transpose().eval()) * Ki;
   
     x = xp + K*(z - H*xp); // x = [q1 q2 q3 q4]'
     P = Pp - K*H*Pp;
   
     phi = atan2((2*(x(2,0)*x(3,0) + x(0,0)*x(1,0))) , (1 - 2*(pow(x(1,0),2) + pow(x(2,0),2)))) * 180/PI;
     theta = -asin(2*(x(1,0)*x(3,0) - x(0,0)*x(2,0))) * 180/PI;
     // psi(yaw) is not correct
   }
   
   void msgCallback(const mpu6050::MsgMpu6050::ConstPtr& msg)
   {
     //각 함수 여기 차례대로 실행
     ros::Rate loop_rate(33);
     
     AcX = msg->AcX;
     AcY = msg->AcY;
     AcZ = msg->AcZ;
    
     GyX = msg->GyX; //p
     GyY = msg->GyY; //q
     GyZ = msg->GyZ; //r
     
     initSystemMatrix();
     EulerAccel();
     EulerToQuaternion();
     algorithm();
     
     pose.roll = phi;
     pose.pitch = theta;
      
     ROS_INFO("roll = %.2lf", pose.roll);
     ROS_INFO("pitch = %.2lf", pose.pitch);
     
     mpu_pub.publish(pose);
     loop_rate.sleep();
   }
    
};
int main(int argc, char **argv)
{
  ros::init(argc,argv,"subscriber_node");
  Kalman kalman;
  
  ros::spin();
  
  return 0;
}
 
