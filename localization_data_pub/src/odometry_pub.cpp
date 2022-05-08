#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <localization_data_pub/Mrpm.h>
#include <sensor_msgs/JointState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "localization_data_pub/ResetPose.h"

#include <dynamic_reconfigure/server.h>
#include <localization_data_pub/parametersConfig.h>

ros::Time previousTime;
ros::Publisher odom_pub;

double x = 0;
double y = 0;
double theta = 0;
const double pi = 3.1415;
//Calibrated Values For Bag1
const double wheelRadius = 0.07;

const int ticks = 42;
const double wheel_x = 0.2;
const double wheel_y = 0.169;

//Calibrated Values For Bag2
/*const double wheelRadius = 0.075;

const int ticks = 42;
const double wheel_x = 0.186;
const double wheel_y = 0.169;*/

//Calibrated Values For Bag3
/*const double wheelRadius = 0.07;

const int ticks = 42;
const double wheel_x = 0.19;
const double wheel_y = 0.169;*/

bool hasPrev;

const double gearRatio = 5.0;
std::vector<double> previousTicks(4,0.0); //the vector that contains the previous ticks that are necessary to calculate delta ticks
std::vector<double> wheelRPM(4,0.0); //the vector that contains the speeds of each wheel in radiants in seconds 

enum IntegrationMethod { Euler, RungeKutta};

using namespace std;

class calcOdom{
public:
  calcOdom(){
  previousTime = ros::Time::now();
  ticks_sub = n.subscribe("wheel_states", 100, &calcOdom::onVelocityUpdate, this); 
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 100);
  velocity_pub = n.advertise<geometry_msgs::TwistStamped>("cmd_vel", 100);

  integMethod = Euler;

  hasPrev = false;

  method_callback = boost::bind(&calcOdom::onIntegrationMethodChange, this, _1, _2);
  method_server.setCallback(method_callback);

  pose_service = n.advertiseService("reset_pose", &calcOdom::resetOdometryPose, this);

}

//Choice between Euler and RK

void onIntegrationMethodChange(localization_data_pub::parametersConfig &config, uint32_t level){
  switch (config.integMethod)
  {
    case 0:
      integMethod = Euler;
    break;
    case 1:
      integMethod = RungeKutta;
    break;
  }
}



//Computation of the several formulas and odometry publication

void onVelocityUpdate(const sensor_msgs::JointState::ConstPtr& msg){

      if(!hasPrev){ //if its the first data received from the encoders the first dticks has to be 0 or else the robot starts in random places 
        previousTicks = msg->position; 
        hasPrev = true;
      }

      ros::Time currentTime = ros::Time::now();;
      double dt = (currentTime - previousTime).toSec(); //we take the time readings in nSecs and we transform them to secs 
      double nX, nY, nTheta; // we initialize the new X, Y, Theta values

      std::vector<double> ticks = msg->position; //this is the vector that is going to hold the current encoder readings

      for(int i = 0; i<ticks.size(); i++){ 

        double dTicks = ticks[i]-previousTicks[i];
        wheelRPM[i] = rpmFromTicks(dTicks, dt);

      }
      previousTicks  = ticks;

      double vx = calcVelX(wheelRPM); //these variables contain the velocities that are calculated with the calcVel function
      double vy = calcVelY(wheelRPM);
      double omega= calcVelAng(wheelRPM);
      publishVelocity(vx,vy,omega); 

      switch(integMethod){

        case Euler:
         nX =x + (vx* cos(theta)-vy*sin(theta))*dt;
         nY =y+ (vx* sin(theta)+vy*cos(theta))*dt;
         nTheta = theta + omega*dt;
        break;

        case RungeKutta:
          float phi = theta + omega*dt/2;
          nX =x + (vx* cos(phi)-vy*sin(phi))*dt;
          nY =y+ (vx* sin(phi)+vy*cos(phi))*dt;
          nTheta = theta + omega*dt;
        break;

      }

      publishOdometryMsg(nX, nY, nTheta, currentTime, vx, vy,omega);

      x = nX;
      y = nY;
      theta = nTheta;
      previousTime = currentTime;

}

double rpmFromTicks(double dTicks, double dt ){ //this is in radiants not in rpms 
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


//Publish the velocity as geometry_msgs/TwistStamped

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

    //Set the position with theta=0
    odomTransform.transform.translation.x = x;
    odomTransform.transform.translation.y = y;
    odomTransform.transform.translation.z = 0.0;

    //Inizialize the quaternion, normalize and then convert to a geometry_msg
    tf2::Quaternion myQuaternion;

    myQuaternion.setRPY(0, 0, 0);

    ROS_INFO_STREAM("x: " << myQuaternion.getX() << " y : " << myQuaternion.getY() <<
          " z: " << myQuaternion.getZ() << " w: " << myQuaternion.getW());

    myQuaternion.normalize();

    geometry_msgs::Quaternion quat_msgs;
    quat_msgs.x = x;
    quat_msgs.y = y;
    quat_msgs.w = theta;
    tf2::convert(myQuaternion, quat_msgs);

    //Set the rotation
    odomTransform.transform.rotation.x = myQuaternion.x();
    odomTransform.transform.rotation.y = myQuaternion.y();
    odomTransform.transform.rotation.z = myQuaternion.z();
    odomTransform.transform.rotation.w = myQuaternion.w();


    odomMsg.header.stamp = time;
    odomMsg.header.frame_id = "odom";

    //Set the position
    odomMsg.pose.pose.position.x = x;
    odomMsg.pose.pose.position.y = y;
    odomMsg.pose.pose.position.z = 0.0;


    //Set the orientation
    odomMsg.pose.pose.orientation.x = myQuaternion.x();
    odomMsg.pose.pose.orientation.y = myQuaternion.y();
    odomMsg.pose.pose.orientation.z = myQuaternion.z();
    odomMsg.pose.pose.orientation.w = myQuaternion.w();

    //Set the velocity
    odomMsg.child_frame_id = "base_link";
    odomMsg.twist.twist.linear.x = vx;
    odomMsg.twist.twist.linear.y = vy;
    odomMsg.twist.twist.angular.z = omega;

    //Publish to odom topic
    odom_pub.publish(odomMsg);

    //Send the transform in broadcast
    br.sendTransform(odomTransform);


    }

//Reset the pose to any given one

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
  ros::ServiceServer pose_service;

  dynamic_reconfigure::Server<localization_data_pub::parametersConfig> method_server;
  dynamic_reconfigure::Server<localization_data_pub::parametersConfig>::CallbackType method_callback;

  IntegrationMethod integMethod;

  ros::Time previousTime = ros::Time::now();
};

int main(int argc, char **argv){

  ros::init(argc, argv, "rb1_odometry");

  calcOdom newCalcOdom;

  ros::spin();

  return 0;
}
