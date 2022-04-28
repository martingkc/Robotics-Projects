#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>

int main(int argc, char **argv){
	ros::init(argc, argv, "pub");

	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("publisher", 1000);
	ros::Rate loop_rate(10);

	/*tf2::Quaternion myQuaternion;

	myQuaternion.setRPY(0, 0, 0);

	ROS_INFO_STREAM("x: " << myQuaternion.getX() << " y : " << myQuaternion.getY() <<
					" z: " << myQuaternion.getZ() << " w: " << myQuaternion.getW());

	myQuaternion.normalize();*/

	while(ros::ok()){
		std_msgs::String msg;
		std::stringstream ss;
		ss << "Hello";
		msg.data = ss.str();

		chatter_pub.publish(msg);

		loop_rate.sleep();

	}
}