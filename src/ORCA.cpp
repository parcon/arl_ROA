#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

#include "Vector3.h" //     arl_RVO2_3D/Vector3.h
#include "Agent.cpp"
#include <cstddef>
#include <vector>
#include <iostream>
#include <fstream>

using namespace RVO;

void runORCA(const Vector3& pos_a, const Vector3& pos_b, const Vector3& vel_a, const Vector3& vel_b, const Vector3& cmd_vel, const float& max_vel, Vector3& new_vel)
{
    const float timeStep = 0.02;    // Simulation time step
    const float timeHorizon = .001;    // Time horizon for collision checker
    const float invTimeHorizon = 1.0f / timeHorizon;       // Inverse of time horizon
    
    // Relative position
    Vector3 pos_rel = pos_b - pos_a;

    // Relative velocity of two quads
    Vector3 vel_rel = vel_a-vel_b;
    
    // Distance between the two quads
    const float distSq = absSq(pos_rel);                

    // Combined radius of the two quads (m)
    //const float combinedRadius = 1.2;                   
    const float combinedRadius = 2.0;

    // Square of combined radius of quads.
    const float combinedRadiusSq = sqr(combinedRadius); 
        
    // Variable to save half-plane from ORCA
    Plane plane;                                        

    // Vector of half-planes for finding safe velocity for more than 2 agents
    std::vector<Plane> orcaPlanes;                      

    // Unit vector of half-plane
    Vector3 u;                                          
    
    // if the robots will not collide:
    if (distSq > combinedRadiusSq)                      
    {
	    const Vector3 w = vel_rel - invTimeHorizon * pos_rel;
        const float wLengthSq = absSq(w);
		const float dotProduct1 = w * pos_rel;
		if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) 
        {
			const float wLength = std::sqrt(wLengthSq);
			const Vector3 unitW = w / wLength;
			plane.normal = unitW;
			u = (combinedRadius * invTimeHorizon - wLength) * unitW;
		}
		else 
        {
			const float A = distSq;
			const float B = pos_rel * vel_rel;
			const float C = absSq(vel_rel) - absSq(cross(pos_rel, vel_rel)) / (distSq - combinedRadiusSq);
			const float t = (B + std::sqrt(sqr(B) - A * C)) / A;
			const Vector3 w = vel_rel - t * pos_rel;
			const float wLength = abs(w);
			const Vector3 unitW = w / wLength;
			plane.normal = unitW;
			u = (combinedRadius * t - wLength) * unitW;
		}
	}
    // if the robots will collide:
	else 
    {
		const float invTimeStep = 1.0f / timeStep;
		const Vector3 w = vel_rel - invTimeStep * pos_rel;
		const float wLength = abs(w);
		const Vector3 unitW = w / wLength;
		plane.normal = unitW;
		u = (combinedRadius * invTimeStep - wLength) * unitW;
	}
    plane.point = vel_a + 0.5f * u;
	orcaPlanes.push_back(plane);
    const size_t planeFail = linearProgram3(orcaPlanes, max_vel, cmd_vel, false, new_vel);
	if (planeFail < orcaPlanes.size()) {
		linearProgram4(orcaPlanes, planeFail, max_vel, new_vel);
	}
}
/* 
void altd_controller(double vx_des,double vy_des,double altd_des,double Kp, double Kd,double max_speed)
{
		geometry_msgs::Twist twist_msg_gen;
	
		cmd_x=Kp*(vx_des-drone_vx); //-Kd *drone_vx_	; //{-1 to 1}=K*( m/s - m/s)
		cmd_y=Kp*(vy_des-drone_vy); 
		//cmd_z=Kp*(vz_des-drone_vz);
		cmd_z=Kp*(altd_des-drone_altd);
		twist_msg_gen.angular.x=1.0; 
		twist_msg_gen.angular.y=1.0;
		twist_msg_gen.angular.z=0.0;
		
		if (cmd_z > max_speed)
			{cmd_z=1.0;}
		if (cmd_z < 0.0)
			{cmd_z=0.0;}
		std::vector result(cmd_z
		return 
}	
*/

//Globals for callback
Vector3 posB_in;
Vector3 velA_in;
Vector3 velB_in;
Vector3 AgoalVel_in;
int had_message =0;
int had_message2 =0;
float maxVel = 1.0;      // max possible velocity of agent
double Kp= 1.0;
double Kd= 0.5;
double des_altd= 1.0;

double drone_vx_, drone_vy_ , drone_vz_;
double drone_ax_, drone_ay_ , drone_az_, drone_altd_;
double drone_vx, drone_vy , drone_vz;
double drone_ax, drone_ay , drone_az, drone_altd;

double map(double value, double in_min, double in_max, double out_min, double out_max) {
  return (double)((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

geometry_msgs::Twist twist_controller(geometry_msgs::Vector3 v_des,double K)
{
		geometry_msgs::Twist twist_msg_gen;
				
		twist_msg_gen.linear.x=K*(v_des.x-drone_vx_); 
		twist_msg_gen.linear.y=K*(v_des.y-drone_vy_); 
		twist_msg_gen.linear.z=K*(v_des.z-drone_vz_);
		twist_msg_gen.angular.x=1.0; 
		twist_msg_gen.angular.y=1.0;
		twist_msg_gen.angular.z=0.0;
	
		twist_msg_gen.linear.x= map(twist_msg_gen.linear.x, -maxVel, maxVel, -1.0, 1.0);  //{-1 to 1}=K*( m/s - m/s)
		twist_msg_gen.linear.y= map(twist_msg_gen.linear.y, -maxVel, maxVel, -1.0, 1.0);
		twist_msg_gen.linear.z= map(twist_msg_gen.linear.z, -maxVel, maxVel, -1.0, 1.0);
		return twist_msg_gen;
}

void kalman_callback(const std_msgs::Float32MultiArray& state_in)
{
	//Take in state post KF and put into vels and relative state for orca

	posB_in[0]=state_in.data[0];
	posB_in[1]=state_in.data[1];
	posB_in[2]=state_in.data[2];

	velA_in[0]=state_in.data[3];
	velA_in[1]=state_in.data[4];
	velA_in[2]=state_in.data[5];

	velB_in[0]=state_in.data[6];
	velB_in[1]=state_in.data[7];
	velB_in[2]=state_in.data[8];

	had_message=1;
}

void nav_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in state of ardrone	
	drone_vx_=msg_in.vx*0.001; //[mm/s] to [m/s]
	drone_vy_=msg_in.vy*0.001;	
	drone_vz_=msg_in.vz*0.001;
	drone_altd_=msg_in.altd*0.001;
	
	drone_ax_=msg_in.ax*9.8; //[g] to [m/s2]
	drone_ay_=msg_in.ay*9.8;	
	drone_az_=msg_in.az*9.8;
		
	//drone_state=msg_in.state;	
	//ROS_INFO("getting sensor reading");	
}

void joy_callback(const geometry_msgs::Vector3& joy_in)
{
	//Take in state post KF and put into vels and relative state for orca
	AgoalVel_in[0]=joy_in.x;
	AgoalVel_in[1]=joy_in.y;
	AgoalVel_in[2]=joy_in.z;
	had_message2=1;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ORCA_3D");  // Initialize ROS
    ros::NodeHandle n;                  // declare node handle
    ros::Rate loop_rate(50);    // Set frequency of rosnode
    ros::Publisher pub_posA, pub_posB;  // Setup position publishers
    ros::Publisher pub_velA, pub_velB;  // Setup velocity publishers
    ros::Subscriber sub_state; 
    ros::Subscriber sub_joy;
    ros::Subscriber sub_nav;
   	ros::Publisher pub_twist; 

	sub_nav = n.subscribe("ardrone/navdata", 1, nav_callback);
    pub_velA = n.advertise<geometry_msgs::Vector3>("cmd_vel_u",1); // Set advertiser for velA
    sub_state= n.subscribe("state_post_KF", 1, kalman_callback);
    sub_joy= n.subscribe("joy_vel", 1, joy_callback);
    pub_twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

   // geometry_msgs::Vector3 posA_msg, posB_msg; // Set position value for in sim
    geometry_msgs::Vector3 cmd_vel_u_msg;
    //, velB_msg; // Set velocity value for in sim
 	geometry_msgs::Twist cmd_vel_twist;
    // State parameters
    Vector3 posB(0.0,0.0,0.0);   // Position of B rel to A, but since A is at (0,0,0) relAB = posB
    Vector3 velA(0.0,0.0,0.0);
    Vector3 velB(0.0,0.0,0.0);  // Velocity of A, Velocity of B
    Vector3 newVel(0.0,0.0,0.0);                    // Velocity that will come out of ORCA
	Vector3 AgoalVel(0.0,0.0,0.0);
	Vector3 empty_vec(0.0,0.0,0.0);

    ROS_INFO("Starting the ORCA node");    

	while ( (had_message ==0) || (had_message2 ==0) )
	{
	ros::spinOnce();
	loop_rate.sleep();
	}

    while(ros::ok() && had_message && had_message2)
    {
        // For quad A:
        //      Pass desired velocity into ORCA
        //      Get new goal velocity
        // For quad B: 
        //      Pass desired velocity into ORCA 
        //      Get new goal velocity
        //  For quad A:
        //      Update next position as PA_k+1 = PA_k + VA_k*delta_T
        //  For quad B:
        //      Update next posiiton as PB_k+1 = PB_k + VB_k*delta_T   
            
        velA=velA_in;
        velB=velB_in;
		posB=posB_in;
		AgoalVel=AgoalVel_in;
 
        // run ORCA for A->B
        runORCA(empty_vec,posB, velA, velB, AgoalVel, maxVel, newVel);

		geometry_msgs::Vector3 cmd_vel_temp;
		cmd_vel_temp.x=newVel[0];
		cmd_vel_temp.y=newVel[1];
		cmd_vel_temp.z=newVel[2];

		cmd_vel_twist=twist_controller(cmd_vel_temp,Kp);
 
        cmd_vel_u_msg.x = newVel[0];
        cmd_vel_u_msg.y = newVel[1];
        cmd_vel_u_msg.z = newVel[2];
        pub_velA.publish(cmd_vel_u_msg);
        pub_twist.publish(cmd_vel_twist);

        ros::spinOnce();
        loop_rate.sleep();
     }
     return 0;
     ROS_ERROR("ORCA_3D has exited");
}
