#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <localization_data_pub/Mrpm.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include "localization_data_pub/ResetPose.h"
ros::Time previousTime;
ros::Publisher odom_pub;

double x = 0;
double y = 0;
double theta= 0;
const double pi = 3.1415;
const double wheelRadius = 0.07;
const double ticksPerMeter = (1/(2*pi*wheelRadius))*42;
const int ticks = 42;
const double wheel_x = 0.2;
const double wheel_y = 0.169;

const double gearRatio = 5.0;
std::vector<double> previousTicks(4,0.0);
std::vector<double> wheelRPM(4,0.0);


using namespace std;

class calcOdom{
public:
  calcOdom(){
  previousTime = ros::Time::now();
  ticks_sub = n.subscribe("wheel_states", 100, &calcOdom::onVelocityUpdate, this);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  velocity_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 100);
  pose_service = n.advertiseService("reset_pose", &calcOdom::resetOdometryPose, this);
  
}

void onVelocityUpdate(const sensor_msgs::JointState::ConstPtr& msg){

    	ros::Time currentTime = ros::Time::now();
      double dt = (currentTime - previousTime).toSec();
      double nX, nY, nTheta;

      std::vector<double> ticks = msg->position;

      for(int i = 0; i<ticks.size(); i++){

        double dTicks = ticks[i]-previousTicks[i];
        wheelRPM[i] = rpmFromTicks(dTicks, dt);

      }
      previousTicks  = ticks;

      double vx = calcVelX(wheelRPM);
      double vy = calcVelY(wheelRPM);
      double omega= calcVelAng(wheelRPM);
      publishVelocity(vx,vy,omega); 
	

      nX =x + (vx* cos(theta)-vy*sin(theta))*dt;
      nY =y+ (vx* sin(theta)+vy*cos(theta))*dt;
      nTheta = theta + omega*dt;

      publishOdometryMsg(nX, nY, nTheta, currentTime, vx, vy,omega);

      x = nX;
      y = nY;
      theta = nTheta;
      previousTime = currentTime;





}
double rpmFromTicks(double dTicks, double dt ){
  double rpm = (dTicks*2*pi)/(ticks*gearRatio*dt);
  return rpm;
}
double calcVelX(std::vector<double> wheelRPM){
  double vx = (wheelRadius/4)*(wheelRPM[0] + wheelRPM[1] +  wheelRPM[2] + wheelRPM[3]);
  return vx;

}
double calcVelY(std::vector<double> wheelRPM){
  double vy = (wheelRadius/4)*(-wheelRPM[0] + wheelRPM[1] +  wheelRPM[2] - wheelRPM[3]);
  return vy;

}
double calcVelAng(std::vector<double> wheelRPM){
  double omega = (wheelRadius/(4*(wheel_x+wheel_y)))*(-wheelRPM[0] + wheelRPM[1] -  wheelRPM[2] + wheelRPM[3]);
	return omega;
}




void publishVelocity(double vx, double vy, double omega){
  geometry_msgs::TwistStamped vel;
  vel.twist.linear.x = vx;
  vel.twist.linear.y = vy;
  vel.twist.angular.z= omega;
  velocity_pub.publish(vel);
}



  void publishOdometryMsg(double x, double y, double theta, ros::Time time, double vx, double vy, double omega){

    odomTransform.header.stamp = time;
    odomTransform.header.frame_id = "odom";
    odomTransform.child_frame_id = "base_link";

    //Set the position
    odomTransform.transform.translation.x = x;
    odomTransform.transform.translation.y = y;
    odomTransform.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    odomTransform.transform.rotation.x = q.x();
    odomTransform.transform.rotation.y = q.y();
    odomTransform.transform.rotation.z = q.z();
    odomTransform.transform.rotation.w = q.w();


  	odomMsg.header.stamp = time;
  	odomMsg.header.frame_id = "odom";

  	//Set the position
  	odomMsg.pose.pose.position.x = x;
  	odomMsg.pose.pose.position.y = y;
  	odomMsg.pose.pose.position.z = 0.0;



    odomMsg.pose.pose.orientation.x = q.x();
    odomMsg.pose.pose.orientation.y = q.y();
    odomMsg.pose.pose.orientation.z = q.z();
    odomMsg.pose.pose.orientation.w = q.w();

  	//Set the velocity
  	odomMsg.child_frame_id = "base_link";
  	odomMsg.twist.twist.linear.x = vx;
  	odomMsg.twist.twist.linear.y = vy;
  	odomMsg.twist.twist.angular.z = omega;

  	odom_pub.publish(odomMsg);

    //Send the transform
    br.sendTransform(odomTransform);
  

    }
  
bool resetOdometryPose(localization_data_pub::ResetPose::Request &req, localization_data_pub::ResetPose::Response &res){
  x = req.givenX;
  y = req.givenY;
  theta = req.givenTheta;
  previousTime = ros::Time::now();
  return true;
}	
	
  private:
  ros::NodeHandle n;
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped odomTransform;
  nav_msgs::Odometry odomMsg;
  ros::Subscriber ticks_sub;
  ros::Publisher odom_pub;
  ros::Publisher velocity_pub;
  ros::Publisher wheel_rpm_pub;


  ros::Time previousTime;
};

int main(int argc, char **argv){

	ros::init(argc, argv, "rb1_odometry");

	calcOdom newCalcOdom;

  ros::spin();

  return 0;
}
