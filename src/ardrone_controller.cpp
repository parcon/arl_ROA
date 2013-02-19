/*
Parker Conroy
Algorithmic Robotics Lab @ University of Utah

This code takes in joy messages and allows contol of the drone and exports a velocity.
*/
#include <ros/ros.h>
#include <std_msgs/Empty.h>
//#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>
#include <ardrone_autonomy/Navdata.h>
	
double max_speed = 1.0; //[m/s]
double des_altd= 1.0;

double joy_x_,joy_y_,joy_z_;
int joy_a_,joy_b_,joy_xbox_;
double joy_x,joy_y,joy_z;
int joy_a,joy_b,joy_xbox;
double cmd_x,cmd_y,cmd_z;
//int new_msg=0;
int drone_state =0; 
float drone_batt =100.0;

// state: {0 is failure, 2 is landed, 3 is flying, 4 is hovering, 6 taking off, 8 landing}
float forget =0.99;
//double joy_x_old,joy_y_old,joy_z_old;

//geometry_msgs::Twist twist_msg;
std_msgs::Empty emp_msg;
geometry_msgs::Vector3 v3_msg; //[x, y,z]
sensor_msgs::Joy joy_msg_in;
	
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	//Take in xbox controller
	joy_x_=joy_msg_in.axes[1]; //left stick up-down
	joy_y_=joy_msg_in.axes[0]; //left stick left-right
	joy_z_=joy_msg_in.axes[4]; //left stick left-right
	joy_a_=joy_msg_in.buttons[0]; //a button
	joy_b_=joy_msg_in.buttons[1]; //b button
	joy_xbox_=joy_msg_in.buttons[8]; //xbox button
}	

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	drone_state=msg_in.state;	
	drone_batt=msg_in.batteryPercent;
}

double map(double value, double in_min, double in_max, double out_min, double out_max) {
  return (double)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}	

void merge_new_mgs(void){
		joy_x=joy_x_;
		joy_y=joy_y_;
		joy_z=joy_z_;
		joy_a=joy_a_;
		joy_b=joy_b_;
		joy_xbox=joy_xbox_;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv,"ARDrone_Control");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);

	ros::Publisher pub_empty_reset;
	ros::Publisher pub_empty_land;
	ros::Publisher pub_empty_takeoff;
	ros::Publisher pub_v3;
	ros::Subscriber joy_sub;
	ros::Subscriber nav_sub;

	joy_sub = node.subscribe("joy", 1, joy_callback);
	nav_sub = node.subscribe("ardrone/navdata", 1, nav_callback);

    pub_v3 = node.advertise<geometry_msgs::Vector3>("joy_vel", 1); 
	pub_empty_reset = node.advertise<std_msgs::Empty>("ardrone/reset", 1);
	pub_empty_takeoff = node.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
	pub_empty_land = node.advertise<std_msgs::Empty>("ardrone/land", 1);
	
    ROS_INFO("Starting AR-Drone Controller");
 	while (ros::ok()) {
	merge_new_mgs();
	
	if (drone_batt < 30.0)
	{
	ROS_ERROR("BATTERY IS CRITICAL LOW");
	}
	
		//	system(chmod a+rw /dev/input/js0);
		//commands to change state of drone
		if (joy_a){
			while (drone_state ==2){
				ROS_INFO("Controller: Launching drone");
				pub_empty_takeoff.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone take off
		}	
		if (joy_b){
			while (drone_state ==3 || drone_state ==4){
				ROS_INFO("Controller: Landing drone");
				pub_empty_land.publish(emp_msg); //launches the drone
				ros::spinOnce();
				loop_rate.sleep();
			}//drone land
		}
		if (joy_xbox){
			double time_start=(double)ros::Time::now().toSec();
			while (drone_state == 0 ){
				ROS_INFO("Controller: Resetting drone");
				pub_empty_reset.publish(emp_msg); //resets the drone
				ros::spinOnce();
				loop_rate.sleep();
				if((double)ros::Time::now().toSec()> time_start+3.0){ 					
					ROS_ERROR("Controller: Time limit reached, unable reset ardrone");
					break; //exit loop
				}
			}//drone take off	
		}
		if (fabs(joy_x)<0.10) {joy_x =0;}
		//else {joy_x=joy_x*forget+joy_x_old*(1-forget);} //smoothing via forget

		if (fabs(joy_y)<0.1) {joy_y =0;}
		//else {joy_y=joy_y*forget+joy_y_old*(1-forget);}

		if (fabs(joy_z)<0.1) {joy_z =0;}
		//else {joy_z=joy_z*forget+joy_z_old*(1-forget);} 

		cmd_x= joy_x*max_speed;
		cmd_y= joy_y*max_speed;
		cmd_z= joy_z*max_speed;
	
		v3_msg.x=cmd_x;
		v3_msg.y=cmd_y;
		v3_msg.z=cmd_z;
		pub_v3.publish(v3_msg);

		ros::spinOnce();
		loop_rate.sleep();
		}//ros::ok
ROS_ERROR("AR_Drone_Controller has exited");
}//main
