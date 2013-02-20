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
#include <Eigen/Dense>
#include <cstddef>
#include <vector>
#include <iostream>
#include <fstream>

using namespace RVO;

void runORCA(const Vector3& pos_a, const Vector3& pos_b, const Vector3& vel_a, const Vector3& vel_b, const Vector3& cmd_vel, const float& max_vel, Vector3& new_vel)
{
    const int num_agents = 5;    
    const float timeStep = 0.02;    // Simulation time step
    const float timeHorizon = 3.0f;    // Time horizon for collision checker
    const float invTimeHorizon = 1.0f / timeHorizon;       // Inverse of time horizon
    
    // Combined radius of the two quads (m)
    const float combinedRadius = 1.2;                   

    // Square of combined radius of quads.
    const float combinedRadiusSq = sqr(combinedRadius); 

    Vector3 offset(0.0,0.0,1.0);

    // Relative positions
    std::vector<Vector3> pos_rel;
    pos_rel.push_back(pos_b - pos_a + offset*0.5*combinedRadius);  // Dummy quad 1
    pos_rel.push_back(pos_b - pos_a + offset*combinedRadius);      // Dummy quad 2
    pos_rel.push_back(pos_b - pos_a);                              // Real quad
    pos_rel.push_back(pos_b - pos_a - offset*0.5*combinedRadius);  // Dummy quad 3
    pos_rel.push_back(pos_b - pos_a - offset*combinedRadius);      // Dummy quad 4

    // Relative velocity of two quads
    std::vector<Vector3> vel_rel;
    vel_rel.push_back(vel_a-vel_b);
    vel_rel.push_back(vel_a-vel_b);
    vel_rel.push_back(vel_a-vel_b);
    vel_rel.push_back(vel_a-vel_b);
    vel_rel.push_back(vel_a-vel_b);
    
    // Variable to save half-plane from ORCA
    Plane plane; 

    // Vector of half-planes for finding safe velocity for more than 2 agents
    std::vector<Plane> orcaPlanes;

    // Unit vector of half-plane
    Vector3 u;  

    // Distance between the two quads
    std::vector<float> distSq;
    distSq.push_back(absSq(pos_rel[0]));
    distSq.push_back(absSq(pos_rel[1]));
    distSq.push_back(absSq(pos_rel[2]));
    distSq.push_back(absSq(pos_rel[3]));
    distSq.push_back(absSq(pos_rel[4]));

    for(int i = 0; i < num_agents; i++)
    {
        // if the robots will not collide:
        if (distSq[i] > combinedRadiusSq)                      
        {
	        const Vector3 w = vel_rel[i] - invTimeHorizon * pos_rel[i];
            const float wLengthSq = absSq(w);
	        const float dotProduct1 = w * pos_rel[i];
	        if (dotProduct1 < 0.0f && sqr(dotProduct1) > combinedRadiusSq * wLengthSq) 
            {
		        const float wLength = std::sqrt(wLengthSq);
		        const Vector3 unitW = w / wLength;
		        plane.normal = unitW;
		        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
	        }
	        else 
            {
		        const float A = distSq[i];
		        const float B = pos_rel[i] * vel_rel[i];
		        const float C = absSq(vel_rel[i]) - absSq(cross(pos_rel[i], vel_rel[i])) / (distSq[i] - combinedRadiusSq);
		        const float t = (B + std::sqrt(sqr(B) - A * C)) / A;
		        const Vector3 w = vel_rel[i] - t * pos_rel[i];
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
		    const Vector3 w = vel_rel[i] - invTimeStep * pos_rel[i];
		    const float wLength = abs(w);
		    const Vector3 unitW = w / wLength;
		    plane.normal = unitW;
		    u = (combinedRadius * invTimeStep - wLength) * unitW;
	    }
        plane.point = vel_a + 0.5f * u;
	orcaPlanes.push_back(plane);
    }

    const size_t planeFail = linearProgram3(orcaPlanes, max_vel, cmd_vel, false, new_vel);
	if (planeFail < orcaPlanes.size())
    {
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
float maxVel = 1.0;      // max possible velocity of agent
double Kp= 0.75;			// Kp = 2.0 at max angle of 5deg, 
double des_altd= 1.0;

double drone_vx_, drone_vy_ , drone_vz_;
double drone_ax_, drone_ay_ , drone_az_, drone_altd_;
double drone_vx, drone_vy , drone_vz;
double drone_ax, drone_ay , drone_az, drone_altd;
double drone_rx_, drone_ry_;  // Tilt angle of drone, DAMAN.

// Initialize LQR Flag so LQR matrices are only computed once, global LQR matrices
int LQR_flag = 0;
Eigen::MatrixXf L(2,4);
Eigen::MatrixXf E(2,2);

double map(double value, double in_min, double in_max, double out_min, double out_max) 
{
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

//void LQR_matrices(const Eigen::Matrix<float,2,2>& Q, const Eigen::Matrix<float,2,2>& R, Eigen::Matrix<float,2,4>& L, Eigen::Matrix<float,2,2>& E)
void LQR_matrices(const Eigen::MatrixXf& Q, const Eigen::MatrixXf& R, Eigen::MatrixXf& L, Eigen::MatrixXf& E)
{
    float Kt = 10;
    float Cd = 0.25;
    float mg = 0.38*9.81;

    Eigen::MatrixXf A(4,4);
    Eigen::MatrixXf B(4,2);

    A << -Kt,   0,   0,   0,
           0, -Kt,   0,   0,
           0,  mg, -Cd,   0,
          mg,   0,   0, -Cd;

    B << Kt,  0,
          0, Kt,
          0,  0, 
          0,  0;

    Eigen::MatrixXf S(4,4);
    Eigen::MatrixXf T(4,2);
    Eigen::MatrixXf V(2,4);
    
    V << 0, 0, 1, 0,
         0, 0, 0, 1;

    Eigen::MatrixXf Vt(4,2);
    Vt = V.transpose();
    Eigen::MatrixXf At(4,4);
    At=A.transpose();
    Eigen::MatrixXf Bt(2,4);
    Bt= B.transpose();

    // Initialize S and T
    S = Vt*Q*V;
    //T = Vt*Q;
    T = (-1.0)*(Vt*Q); // Made negative like code in LQR obstacles Daman

    // Find convergence values for S and T
    for(int i = 0; i<200; i++)
    {
        S = Vt*Q*V + At*S*A - At*S*B*(R+Bt*S*B).inverse()*Bt*S*A;
        //T = Vt*Q + At*T - At*S*B*(R+Bt*S*B).inverse()*Bt*T;
        T = (-1.0)*(Vt*Q) + At*T - At*S*B*(R+Bt*S*B).inverse()*Bt*T; // Made Vt*Q negative like LQR Daman
    }

    //L = (R+Bt*S*B).inverse()*Bt*S*A;
    L = (-1.0)*((R+Bt*S*B).inverse()*Bt*S*A);
    //E = (R+Bt*S*B).inverse()*Bt*T;
    E = (-1.0)*((R+Bt*S*B).inverse()*Bt*T);

    // Reset flag so function won't run again.
    LQR_flag = 1;           
    
}

geometry_msgs::Twist LQR_controller(geometry_msgs::Vector3 vel_des)
{
    // Setup cost matrices
    Eigen::MatrixXf Q(2,2);
    Eigen::MatrixXf R(2,2);
    
    Q << 10, 0,
          0, 10;

    R << 0.0001, 0,
         0,   0.0001;

    // Computer matrices for r_d = -Lx+Ev_d
    if (LQR_flag == 0)
    {
        LQR_matrices(Q, R, L, E); // Sets L, E, and flag (they are sent in as pointers)
    }
	
	std::ostringstream OUTL;
	OUTL << L;
	ROS_INFO("LQR L: %s",OUTL.str().c_str());
	
	std::ostringstream OUTE;
	OUTE << E;
	ROS_INFO("LQR E: %s",OUTE.str().c_str());
	
    // Set state x
    Eigen::Vector4f x;
    Eigen::Vector2f rd;
    Eigen::Vector2f vd;

    vd[0] = vel_des.x;
    vd[1] = vel_des.y;

    x[0] = drone_rx_; // Rotation about x-axis
    x[1] = drone_ry_; // Rotation about y-axis
    x[2] = drone_vx_; // X velocity
    x[3] = drone_vy_; // Y velocity

	std::ostringstream OUTX;
	OUTX << x;
	ROS_INFO("LQR x: %s",OUTX.str().c_str());
	
	std::ostringstream OUTvd;
	OUTvd << vd;
	ROS_INFO("LQR vd: %s",OUTvd.str().c_str());
	
	rd = -L*x + E*vd; 
	//rd = L*x + E*vd; // Made L*x positive like LQR obstacles

	std::ostringstream OUTR;
	OUTR << rd;
	ROS_INFO("LQR rd: %s",OUTR.str().c_str());
	
	
    // Set r_des for output
    geometry_msgs::Twist twist_msg_gen;

    twist_msg_gen.linear.x = rd[1];   // Velocity in x related to angle ABOUT y, 2nd element of state
    twist_msg_gen.linear.y = rd[0];   // Velocity in y related to angle ABOUT x, 1st element of state
    twist_msg_gen.linear.z = AgoalVel_in[2];
    twist_msg_gen.angular.x = 1.0;
    twist_msg_gen.angular.y = 1.0;
    twist_msg_gen.angular.z = 0.0;

	double maxAngle = 0.25;
    twist_msg_gen.linear.x= map(twist_msg_gen.linear.x, -maxAngle, maxAngle, -1.0, 1.0);  //{-1 to 1}=K*( m/s - m/s)
	twist_msg_gen.linear.y= map(twist_msg_gen.linear.y, -maxAngle, maxAngle, -1.0, 1.0);
	twist_msg_gen.linear.z= map(twist_msg_gen.linear.z, -maxAngle, maxAngle, -1.0, 1.0);
	
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
	drone_altd_=msg_in.altd*0.01;
	
	drone_ax_=msg_in.ax*9.8; //[g] to [m/s2]
	drone_ay_=msg_in.ay*9.8;	
	drone_az_=msg_in.az*9.8;
		
    drone_rx_ = msg_in.rotX*3.14159/180; // [degree] to [radians]
    drone_ry_ = msg_in.rotY*3.14159/180; // [degree] to [radians]

	//drone_state=msg_in.state;	
	//ROS_INFO("getting sensor reading");	
}
/*
void joy_callback(const geometry_msgs::Vector3& joy_in)
{
	// Take in joystick values
	AgoalVel_in[0]=joy_in.x;
	AgoalVel_in[1]=joy_in.y;
	AgoalVel_in[2]=joy_in.z;
	had_message2=1;
}
*/
void joy_twist_callback(const geometry_msgs::Twist& joy_twist_in)
{
	//Take in state post KF and put into vels and relative state for orca
	AgoalVel_in[0]=joy_twist_in.linear.x;
	AgoalVel_in[1]=joy_twist_in.linear.y;
	AgoalVel_in[2]=joy_twist_in.linear.z;
	cmd_yaw=joy_twist_in.angular.z;
	had_message2=1;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"ORCA_3D");     // Initialize ROS
    ros::NodeHandle n;                  // declare node handle
    ros::Rate loop_rate(50);            // Set frequency of rosnode
    ros::Publisher pub_posA, pub_posB;  // Setup position publishers
    ros::Publisher pub_velA, pub_velB;  // Setup velocity publishers
    ros::Subscriber sub_state; 
    ros::Subscriber sub_joy;
    ros::Subscriber sub_nav;
    ros::Subscriber sub_twist;
   	ros::Publisher pub_twist; 

	sub_nav = n.subscribe("ardrone/navdata", 1, nav_callback);
    pub_velA = n.advertise<geometry_msgs::Vector3>("cmd_vel_u",1); 
    sub_state= n.subscribe("state_post_KF", 1, kalman_callback);
   // sub_joy= n.subscribe("joy_vel", 1, joy_callback);
    sub_twist=n.subscribe("joy_vel_twist", 1, joy_twist_callback);
    pub_twist = n.advertise<geometry_msgs::Twist>("cmd_vel", 1); 

    geometry_msgs::Vector3 cmd_vel_u_msg;
 	geometry_msgs::Twist cmd_vel_twist;

    // State parameters
    Vector3 posB(0.0,0.0,0.0);   // Position of B rel to A, but since A is at (0,0,0) relAB = posB
    Vector3 velA(0.0,0.0,0.0);
    Vector3 velB(0.0,0.0,0.0);  // Velocity of A, Velocity of B
    Vector3 newVel(0.0,0.0,0.0); // Velocity that will come out of ORCA
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
           
        velA=velA_in;
        velB=-velB_in; // made negative to put in world frame not second quad PARCON
        
	    posB=posB_in;
	    AgoalVel=AgoalVel_in;
 
 	    //ROS_INFO("vBx: %f, vBy: %f, vBz: %f", velB[0], velB[1], velB[2]);
 	    //ROS_INFO("vBAx: %f, vBAy: %f, vBAz: %f", velB[0]+velA[0], velB[1]+velA[1], velB[2]+velA[2]);
        runORCA(empty_vec,posB, velA, velB+velA, AgoalVel, maxVel, newVel); // made velB act as a relative, added velA to get a global PARCON
	
    	geometry_msgs::Vector3 cmd_vel_temp;
    	cmd_vel_temp.x=newVel[0];
    	cmd_vel_temp.y=newVel[1];
    	cmd_vel_temp.z=newVel[2];

	std::ostringstream OUTCMD;
	OUTCMD << cmd_vel_temp;
	ROS_INFO("cmd_vel before LQR: %s",OUTCMD.str().c_str());
	    
        
        // No Controller
	    cmd_vel_twist.linear.x=cmd_vel_temp.x; 
	    cmd_vel_twist.linear.y=cmd_vel_temp.y; 
	    cmd_vel_twist.linear.z=cmd_vel_temp.z;
	    cmd_vel_twist.angular.x=1.0; 
	    cmd_vel_twist.angular.y=1.0;
	    cmd_vel_twist.angular.z=cmd_yaw;
	  
	
        // P controller	    
        //cmd_vel_twist=twist_controller(cmd_vel_temp,Kp);

        // LQR Controller
        //cmd_vel_twist = LQR_controller(cmd_vel_temp);

        cmd_vel_u_msg.x = cmd_vel_twist.linear.x;
        cmd_vel_u_msg.y = cmd_vel_twist.linear.y;
        cmd_vel_u_msg.z = cmd_vel_twist.linear.z;
        
        pub_velA.publish(cmd_vel_u_msg);
        pub_twist.publish(cmd_vel_twist);

        ros::spinOnce();
        loop_rate.sleep();
     }
     return 0;
     ROS_ERROR("ORCA_3D has exited");
}
