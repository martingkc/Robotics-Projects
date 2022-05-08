# Robotics-Projects 1 

This project was prepared by: 

The initial position of the robot and the parameter calibrations were calculated by taking an average of the starting points of each bag. 
The structure of the TF tree is present in the tfstructure.jpeg file 

WARNINGS: 
Sometimes when the bag files recording ends and loops back due to the TF buffer being full rviz can give errors and shift the odometry. We suggest not playing the bag files with the loop functionality of rosbag turned on. 

SUGGESTIONS: 
- our launch file is configured to run bags we suggest you to check the paths to the bags before running the project (for ease you can directly copy the bag files to the bags folder present in the project folder) 
- run the project by typing catkin_make && roslaunch localization_data_pub project.launch
- we highly suggest you to reset rviz before running the project 

rpm_pub.cpp - node that subscribes to the /cmd_vel topic and calculates the rpms of the four wheels of the robot and outputs the data with a custom message file called Mrpm.msg through the /wheels_rpm topic

Mrpm.msg - the custom message consisting of one header area and four float64 areas (one for each wheel rpm_fl, rpm_fr, rpm_rl and rpm_rr)

odometry_pub.cpp - node that subscribes to the /wheel_states topic and outputs odometry data through the topic /odom and the robot velocity through the topic /cmd_vel 

ResetPose.srv - service that resets the coordinates of the odometry to a given (x,y,z) coordinate. You can use this service by typing rosservice call /reset_pose [x val] [y val] [z val] 
