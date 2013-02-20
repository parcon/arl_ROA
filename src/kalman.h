//matrix work for kalman


const int ROS_HZ = 40;
const float ROS_RATE = 1.0/ROS_HZ;
//part one
const int dimention_n = 9; //length of state vector
const int dimention_m= 6; //length of observation vector
const int dimention_m_small= 3; //length of observation vector
const int dimention_l= 3; //size of control vector


typedef Eigen::Matrix<float,dimention_n,1> State_vector;
typedef Eigen::Matrix<float,dimention_l,1> control_vector;
typedef Eigen::Matrix<float,dimention_m,1> obs_vector;

typedef Eigen::Matrix<float, dimention_n, dimention_n> dynamics_model;
typedef Eigen::Matrix<float, dimention_n, dimention_l> control_model;
typedef Eigen::Matrix<float, dimention_m, dimention_n> state_to_obs;
typedef Eigen::Matrix<float, dimention_n, dimention_m> obs_to_state;
typedef Eigen::Matrix<float, dimention_n, dimention_n> error_cov;
typedef Eigen::Matrix<float, dimention_m, dimention_m> measurement_error_cov;

typedef Eigen::Matrix<float,dimention_m_small,1> obs_vector_small;
typedef Eigen::Matrix<float, dimention_m_small, dimention_n> state_to_obs_small;
typedef Eigen::Matrix<float,dimention_m_small, dimention_m_small> measurement_error_cov_small;
typedef Eigen::Matrix<float, dimention_n, dimention_m_small> obs_to_state_small;

error_cov I;
obs_to_state K;
state_to_obs H;

state_to_obs H_trans;
control_model B;
dynamics_model A;
dynamics_model A_trans;
error_cov Q;
error_cov P;
error_cov P_old;
error_cov P_minus;
measurement_error_cov R;
measurement_error_cov O;

measurement_error_cov_small R_wo_tag;
measurement_error_cov_small O_wo_tag;
state_to_obs_small H_wo_tag;
obs_to_state_small K_wo_tag;
obs_vector_small y_wo_tag;
obs_vector_small z_wo_tag;

obs_vector n;
obs_vector z;
obs_vector y;
control_vector u1;
control_vector u1_old;
control_vector u2;
control_vector u2_old;

State_vector x_minus;
State_vector x;
State_vector x_old;
////////////////////////////////

void init_matrix(void){

//part two
	//int g=  9.8;
	float k1= 0.25;
	float k2= 0.5;

//I
I<<Eigen::Matrix<float, dimention_n, dimention_n>::Identity();

//Q: process noise
Q=I;
Q(0,0)=.001;
Q(1,1)=.001;
Q(2,2)=.001;

Q(3,3)=.25;
Q(4,4)=.25;
Q(5,5)=.25;

Q(6,6)=1.0;
Q(7,7)=1.0;
Q(8,8)=1.0;

//q for vel2 should be large
//qfor vel1 some noise
//q for pos1 is small

//R: observation noise
R=Eigen::Matrix<float, dimention_m, dimention_m>::Identity();
//R=R*.1;
R(0,0)=.5;
R(1,1)=.5;
R(2,2)=.5;

R(3,3)=.25;
R(4,4)=.25;
R(5,5)=.25;

//R(6,6)=2.0;
//R(7,7)=2.0;
//R(8,8)=2.0;

R_wo_tag=Eigen::Matrix<float, dimention_m_small, dimention_m_small>::Identity();
R_wo_tag=.25*R_wo_tag;

//P_old
P_old=I;

//u1
//u1<< 0,0,0;

//u1_old
u1_old<< 0,0,0;

//x_old
//x_old <<1,1,1, 0,1,0,  1,0,1;
x_old <<100000,100000,100000, 0,0,0,  0,0,0;

//A
A<< 0,0,0, 1,0,0, -1,0,0,  //first line
	0,0,0, 0,1,0, 0,-1,0,  //2nd line
	0,0,0, 0,0,1, 0,0,-1,  //3nd line

	0,0,0, -k1,0,0,   0,0,0, //4 line
	0,0,0, 0,-k1,0,   0,0,0, //5 line
	0,0,0, 0,0,-k2,   0,0,0, //6 line

	0,0,0, 0,0,0, 0,0,0,   //7 line
	0,0,0, 0,0,0, 0,0,0,   //8 line
	0,0,0, 0,0,0, 0,0,0; //9 line

A=(ROS_RATE*A+I); //discritize 




//B
B<< 0,0,0, //1
	0,0,0, //2
	0,0,0, //3

	k1,0,0, //4
	0,k1,0, //5
	0,0,k2, //6

	0,0,0, //7
	0,0,0, //8
	0,0,0; //9

B=B*ROS_RATE;  //discritize
std::cout << std::endl;
std::cout << "B" << std::endl;
std::cout << B << std::endl;


//H

H<< 1,0,0, 0,0,0,   0,0,0,  
	0,1,0, 0,0,0,   0,0,0,  
	0,0,1, 0,0,0,   0,0,0,  

	0,0,0, 1,0,0,   0,0,0,  
	0,0,0, 0,1,0,   0,0,0,   
	0,0,0, 0,0,1,   0,0,0;


//old H from kalman_old

H_wo_tag<<  0,0,0, 1,0,0,  0,0,0,
			0,0,0, 0,1,0,  0,0,0,
			0,0,0, 0,0,1,  0,0,0;

std::cout << std::endl;
std::cout << "H" << std::endl;
std::cout << H << std::endl;



}//end matrix setup
