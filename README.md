# Robotics-Projects 1 

This project was prepared by: 
- Ahmet Martin Gökçü 10716652
- Jody Roberto Battistini 10691667
- Matteo Barin 10618370


All the calibrations were done manually you can switch to the calibrated vlues for each bag by uncommenting the variables that are indicated for the corrisponding bags number. Also the quaternions for the world frame for each bag is present in the project.launch file all the quaternions are indicated with their corrisponding bags number. 
The structure of the TF tree is present in the frames.pdf file the structure of the tree was obtained by using rosrun tf view_frames

WARNINGS: 
Sometimes when the bag files recording ends and loops back due to the TF buffer being full rviz can give errors and shift the odometry. We suggest not playing the bag files with the loop functionality of rosbag turned on. 

!!!!!Dont forget to change the world frames quaternion for each bag from the launch file and dont forget to change to the calibrated values by uncommenting the values in the odonmetry_pub.cpp file for each bag!!!!!

SUGGESTIONS: 
- our launch file is configured to run bags we suggest you to check the paths to the bags before running the project (for ease you can directly copy the bag files to the bags folder present in the project folder) 
- run the project by typing catkin_make && roslaunch localization_data_pub project.launch
- we highly suggest you to reset rviz before running the project 

rpm_pub.cpp - node that subscribes to the /cmd_vel topic and calculates the rpms of the four wheels of the robot and outputs the data with a custom message file called Mrpm.msg through the /wheels_rpm topic

Mrpm.msg - the custom message consisting of one header area and four float64 areas (one for each wheel rpm_fl, rpm_fr, rpm_rl and rpm_rr)

odometry_pub.cpp - node that subscribes to the /wheel_states topic and outputs odometry data through the topic /odom and the robot velocity through the topic /cmd_vel 

ResetPose.srv - service that resets the coordinates of the odometry to a given (x,y,z) coordinate. You can use this service by typing rosservice call /reset_pose [x val] [y val] [z val] 
