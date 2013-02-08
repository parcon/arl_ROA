/*
Parker Conroy
ARLab @ University of Utah


Using RV02_3D
*/

#include <ros/ros.h>
#include <Eigen/Dense>
#include "Agent.cpp"



int main(int argc, char** argv)
{
	//ROS stuff
	ROS_INFO("Starting Kalman");
	ros::init(argc, argv,"RVO2_3D");
    ros::NodeHandle node;
    ros::Rate loop_rate(45);
	//ros::Subscriber nav_sub;
	//ros::Subscriber imu_sub;
	//ros::Publisher state_pub;

	ROS_INFO("Starting RV02_3D loop \n");
	
	while (ros::ok()){
			
		//get funky
		
		
		ros::spinOnce();
		loop_rate.sleep();
		}//while ros ok
		
	
}//main
