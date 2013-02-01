/*
Parker Conroy
ARLab @ University of Utah


This is sample Kalman filter code.

x(t)= A*x(t-1) +B*u(t-1) + w(t-1)
z(t)=Hx(t)+v(t)

x vector is n x 1
u vector is l x 1
z vector is m x 1

A matrix is n x n
B matrix is n x l
H matrix is m x n
Kalman Gain K matrix is m x n  
w is process white noise ~ N(0,Q)
v is measurement white noise ~ N(0,R)
*/

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <ardrone_autonomy/Navdata.h>
#include "kalman.h"
#include <math.h>

//Matrix part one taken from here PARCON

std_msgs::Float32MultiArray x_msg;
float deg2rad= 0.0174532925;
float rotation_roll =0.0;
float rotation_pitch =0.0;
float rotation_yaw =0.0;
float vel_x=0.0;
float vel_y=0.0;
float vel_z=0.0;
float w_roll=0.0;
float w_pitch=0.0;
float w_yaw=0.0;

float x12_hat;
float y12_hat;
float z12_hat;
float vx1_hat;
float vy1_hat;
float vz1_hat;
float rp1_hat;
float rr1_hat;
float wp1_hat;
float wr1_hat;

int tag_id =0;//CHANGES WHICH TAG TO DISPLAY
float vision_angle[2];
float tag_position[3];
int cam_height=360;
int cam_width=640;
float cam_width_rads=92*deg2rad;
float cam_height_rads=51*deg2rad; //cam_width_degree*(cam_height/cam_width) //NOT MEASURED
const float focal_length = .004;//[pixel/m] [???????????????]
const float center2camera_length = .004;// [m] [???????????????]

uint32_t tags_count;
uint32_t tags_type[10];
uint32_t tags_xc[10];
uint32_t tags_yc[10];
uint32_t tags_width[10];
uint32_t tags_height[10];
float tags_orientation[10];
float tags_distance[10];
double time_stamp;
int had_message_1 =0;
int had_message_2 =0;

void state_callback(const ardrone_autonomy::Navdata& msg_in)
{
	had_message_1=1;
	//Take in states of robot that we are interested in from the navdata topic
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
//	rotation_yaw=msg_in.rotZ*deg2rad;
	
	vel_x=msg_in.vx*0.001; //  mm/s to m/s
	vel_y=msg_in.vy*0.001; //  mm/s to m/s
	vel_z=msg_in.vz*0.001; //  mm/s to m/s
	
	//tags
	tags_count=msg_in.tags_count;
	time_stamp=msg_in.tm;	

	for (uint32_t i=0; i <tags_count; i++){
		tags_distance[i]=msg_in.tags_distance[i]*0.01; // cm to m
		tags_xc[i]=msg_in.tags_xc[i];
		tags_yc[i]=msg_in.tags_yc[i];
		tags_width[i]=msg_in.tags_width[i];
		tags_height[i]=msg_in.tags_height[i];
		tags_orientation[i]=msg_in.tags_orientation[i];
	   }
}

void Imu_callback(const sensor_msgs::Imu& imu_in)
{
	had_message_2=1;
	//Take in state of remaining parts of state from the imu topic	
	w_roll=imu_in.angular_velocity.x; //in rads/sec
	w_pitch=imu_in.angular_velocity.y; //in rads/sec
	w_yaw=imu_in.angular_velocity.z; //in rads/sec
}

void get_new_residual_and_H(void){

	//UPDATE Z
	//z = [uy uz d v1x v1y v1z]T
	z(0)=(float)tags_xc[tag_id]; //global y vector
	z(1)=(float)tags_yc[tag_id]; // global z vector
	z(2)=(float)tags_distance[tag_id];
	z(3)=vel_x;
	z(4)=vel_y;
	z(5)=vel_z;
		
	x12_hat=x_minus(0); //estamate of position between rotors
	y12_hat=x_minus(1);
	z12_hat=x_minus(2);
	vx1_hat=x_minus(3); //estimate of velocity quad1
	vy1_hat=x_minus(4);
	vz1_hat=x_minus(5);
	//x_minus(6);
	//x_minus(7);
	//x_minus(8);

//Make the residual
//in the form y= new observation minus nonlinear model
	
	float sqr_xyz= pow(pow((float)x12_hat,2)+pow((float)y12_hat,2)+pow((float)z12_hat,2),0.5);
	
	
	y(0)=z(0)-focal_length*( 
	(y12_hat*cos(rotation_roll)+sin(rotation_roll)*
	(z12_hat*cos(rotation_pitch)+x12_hat*sin(rotation_pitch))
	/
	(-center2camera_length+x12_hat*cos(rotation_pitch)+z12_hat*sin(rotation_pitch) ) )
	); //error in uy
	
	y(1)=z(1)-focal_length*( (y12_hat*sin(rotation_roll)-cos(rotation_roll)*(z12_hat*cos(rotation_pitch)+x12_hat*sin(rotation_pitch))/
	(center2camera_length-x12_hat*cos(rotation_pitch)+z12_hat*sin(rotation_pitch) ) )); //error in uz
	
	y(2)=z(2)-sqr_xyz; //error in distance away

	//linear parts
	y(3)=z(3)- vx1_hat; //error in vx of quad1
	y(4)=z(4)- vy1_hat; //error in vy of quad1
	y(5)=z(5)- vz1_hat; //error in vz of quad1


//Build the H matrix 

	
	//z = [uy uz d vx1 vy1 vz1]T
	
	//H 1st row (uy)
	
	H(0,0)=-focal_length*
	( 
	(y12_hat*cos(rotation_roll)*cos(rotation_pitch)+sin(rotation_roll)*(z12_hat+center2camera_length*sin(rotation_pitch)) )
	/
	( pow(center2camera_length-x12_hat*cos(rotation_pitch)+z12_hat*sin(rotation_pitch) ,2) 	)
	); //duy dx12
	
	H(0,1)=focal_length*(cos(rotation_roll)/(center2camera_length-x12_hat*cos(rotation_pitch)+z12_hat*sin(rotation_pitch))); //duy dy12
	H(0,2)=focal_length*
	( 
	((x12_hat-center2camera_length*cos(rotation_pitch))*sin(rotation_roll)+y12_hat*cos(rotation_roll)*sin(rotation_pitch))
	/
	( pow(center2camera_length-(x12_hat*cos(rotation_pitch))+(z12_hat*sin(rotation_pitch)) ,2) 	)
	);//duy dz12

	//H 2nd row (uz)
	
	H(1,0)= -focal_length*
	( 
	(-y12_hat*cos(rotation_pitch)*sin(rotation_roll)+cos(rotation_roll)* ( z12_hat+center2camera_length*sin(rotation_pitch) ) )/
	( pow(center2camera_length-(x12_hat*cos(rotation_pitch))+(z12_hat*sin(rotation_pitch)) ,2) 	)
	); //duz dx12
	H(1,1)=(focal_length*sin(rotation_roll))/ ( center2camera_length-( x12_hat*cos(rotation_pitch) )  +(z12_hat*sin(rotation_pitch)) ); 
	H(1,2)=(focal_length* ( cos(rotation_roll)*(x12_hat-center2camera_length*cos(rotation_pitch))-y12_hat*sin(rotation_pitch)*sin(rotation_roll)    )  )/ 
	( pow(center2camera_length-(x12_hat*cos(rotation_pitch))+(z12_hat*sin(rotation_pitch)) ,2) );

	
	//H 3rd row (d)
	
	float dd_dx=x12_hat/sqr_xyz;
	float dd_dy=y12_hat/sqr_xyz;
	float dd_dz=z12_hat/sqr_xyz;

	std::cout <<"sqr_xyz"<<std::endl;
	std::cout << sqr_xyz << std::endl;
	
	std::cout <<"z12_hat"<<std::endl;
	std::cout << z12_hat << std::endl;

	std::cout <<"dd_dz"<<std::endl;
	std::cout << dd_dz << std::endl;

	if (sqr_xyz==0) //check for divide by zero
	{
		H(2,0)=0; 
		H(2,1)=0; 
		H(2,2)=0;
	}
	else
	{
		H(2,0)=dd_dx; 
		H(2,1)=dd_dy; 
		H(2,2)=dd_dz;
	}
	//Rest of the H matrix is just composed of ones and zeros.
	//ones on matching terms aka: dvx1/dvx1, dvy1/dvy1
	//zeros elsewhere
}


int main(int argc, char** argv)
{
	//ROS stuff
	ROS_INFO("Starting Kalman");
	ros::init(argc, argv,"Kalman");
    ros::NodeHandle node;
    ros::Rate loop_rate(45);
	ros::Subscriber nav_sub;
	ros::Subscriber imu_sub;
	ros::Publisher state_pub;
	
	state_pub = node.advertise<std_msgs::Float32MultiArray> ("state_post_KF", 1);
	nav_sub = node.subscribe("/ardrone/navdata", 1, state_callback);
	imu_sub = node.subscribe("/ardrone/imu", 1, Imu_callback);
	

	init_matrix(); //Setup Matrices 

	ROS_INFO("Starting Kalman loop \n");
	
	while ( (had_message_1 ==0) || (had_message_2 ==0) )
	{
	ros::spinOnce();
	loop_rate.sleep();
}

	while (ros::ok() && had_message_1 && had_message_2){
			
		//Prediction Step
		x_minus=A*x_old+B1*u1_old;
		P_minus=A*P_old*A.transpose() + Q;
		get_new_residual_and_H();
		//Correction Step
		O=H*P_minus*H.transpose()+R;
		K=P_minus*H.transpose()*O.inverse();
		x=x_minus+K*y;
		P=(I-K*H)*P_minus;

		std::cout <<"x"<<std::endl;
		std::cout << x << std::endl;
		
		std::cout << "y" << std::endl;
		std::cout << y << std::endl;
	/*	
		std::cout <<"x_minus"<<std::endl;
		std::cout << x_minus << std::endl;
		
		std::cout << "z" << std::endl;
		std::cout << z << std::endl;
		

		
		std::cout << "H" << std::endl;
		std::cout << H << std::endl;
		
		std::cout <<"R"<<std::endl;
		std::cout << R << std::endl;
		  
		std::cout <<"O"<<std::endl;
		std::cout << O << std::endl;
		
		std::cout <<"K"<<std::endl;
		std::cout << K << std::endl;
	*/  
	 	x_msg.data.clear(); //clear data
		float move =0.0;
	    for (long int i=0; i<dimention_n; i++)
			{
			move=x[i];
			x_msg.data.push_back(move);//fill msg
			}
		
//		std::cout << x_msg << std::endl;
		state_pub.publish(x_msg); //publish message
		
		x_old=x;
		P_old=P;
		u1_old=u1;
		u2_old=u2;
		
		ros::spinOnce();
		loop_rate.sleep();
		}//while ros ok
		
	
}//main
