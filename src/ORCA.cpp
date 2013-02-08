#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32MultiArray.h>

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
    const float timeHorizon = 2.0;    // Time horizon for collision checker
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
 

//Globals for callback
Vector3 posB_in;
Vector3 velA_in;
Vector3 velB_in;
Vector3 AgoalVel_in;
int had_message =0;
int had_message2 =0;

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
    ros::Publisher pub_posA, pub_posB;  // Setup position publishers
    ros::Publisher pub_velA, pub_velB;  // Setup velocity publishers
    ros::Subscriber sub_state; 
    ros::Subscriber sub_joy; 

    pub_posA = n.advertise<geometry_msgs::Vector3>("posA",1); // Set advertiser for posA
    pub_posB = n.advertise<geometry_msgs::Vector3>("posB",1); // Set advertiser for posB
    pub_velA = n.advertise<geometry_msgs::Vector3>("velA",1); // Set advertiser for velA
    pub_velB = n.advertise<geometry_msgs::Vector3>("velB",1); // Set advertiser for velB
    sub_state= n.subscribe("state_post_KF", 1, kalman_callback);
    sub_joy= n.subscribe("joy_vel", 1, joy_callback);

    ros::Rate loop_rate(50);    // Set frequency of rosnode

    geometry_msgs::Vector3 posA_msg, posB_msg; // Set position value for in sim
    geometry_msgs::Vector3 velA_msg, velB_msg; // Set velocity value for in sim
 


    // State parameters
    float maxVel = 0.5;      // max possible velocity of agent
    Vector3 posB;   // Position of B rel to A, but since A is at (0,0,0) relAB = posB
    Vector3 velA, velB;  // Velocity of A, Velocity of B
    Vector3 newVel;                    // Velocity that will come out of ORCA
	Vector3 AgoalVel(0.0,0.0,0.0);
	Vector3 empty_vec(0.0,0.0,0.0);

    ROS_INFO("Starting the ORCA node.");    


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
            velA=newVel;


        velA_msg.x = velA[0];
        velA_msg.y = velA[1];
        velA_msg.z = velA[2];
        pub_velA.publish(velA_msg);
  
        ros::spinOnce();
        loop_rate.sleep();
     
    }


    return 0;
}
