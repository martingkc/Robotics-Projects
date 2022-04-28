#include "ros/ros.h"
#include "std_msgs/String.h"
#include "project1/Num.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <dynamic_reconfigure/server.h>

void pubCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("Ho sentito: [%s]", msg->data.c_str());
}

class compute_odometry{
	public:
		compute_odometry(){

	}
};

int main(int argc, char **argv){

	ros::init(argc, argv, "sub");
	ros::NodeHandle n;

	ros::Time::now();

	ros::Subscriber sub = n.subscribe("/publisher", 1000, pubCallback);
	ros::spin();

	return 0;
}

