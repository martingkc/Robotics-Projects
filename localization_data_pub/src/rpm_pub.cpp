#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <localization_data_pub/Mrpm.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
const double pi = 3.1415;
const double wheelRadius = 0.07;
const double ticksPerMeter = (1/(2*pi*wheelRadius))*42;
const int ticks = 42;
const double wheel_x = 0.2;
const double wheel_y = 0.169;

ros::Publisher wheel_rpm_pub;

void onVelocityUpdate(const geometry_msgs::TwistStamped::ConstPtr& msg){

  double vx = msg->twist.linear.x;
  double vy = msg->twist.linear.y;
  double omega = msg->twist.angular.z;
	
	
	//here we apply the reverse formulas to calculate the wheel rpms from Vx Vy and Omega that we retreive from cmd_vel

        //these are the variables that contain calculated RPMS
  	float nrpm_fl = (60/(2*pi*wheelRadius))*(vx - vy -(wheel_x -wheel_y)*omega);
	float nrpm_fr = (60/(2*pi*wheelRadius))*(vx + vy +(wheel_x +wheel_y)*omega);
	float nrpm_rl = (60/(2*pi*wheelRadius))*(vx + vy -(wheel_x +wheel_y)*omega);
	float nrpm_rr = (60/(2*pi*wheelRadius))*(vx - vy +(wheel_x +wheel_y)*omega);
	
	localization_data_pub::Mrpm mrpm;

        // here we publish the obtained values with our custom msg mrpm (mrpm has 4 spaces rpm_fl rpm_fr rpm_rl rpm_rr)
	mrpm.rpm_fl = nrpm_fl;
	mrpm.rpm_fr = nrpm_fr;
	mrpm.rpm_rl = nrpm_rl;
	mrpm.rpm_rr = nrpm_rr;

	wheel_rpm_pub.publish(mrpm);

}
int main(int argc, char **argv)
{
  ros::init(argc, argv,"rpm_pub");
  ros::NodeHandle n;


  ros::Subscriber vel_sub = n.subscribe("cmd_vel", 1000, onVelocityUpdate);
  wheel_rpm_pub = n.advertise<localization_data_pub::Mrpm>("wheels_rpm",1000);
  ros::Rate loop_rate(100);
  while(ros::ok()){

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;

}

