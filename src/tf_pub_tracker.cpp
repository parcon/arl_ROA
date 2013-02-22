

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32MultiArray.h>
#include <ardrone_autonomy/Navdata.h>
#include <Eigen/Dense>
#include <geometry_msgs/Vector3.h>
//#include <tf/StampedTransform.h>

const int dimention_n =3;
typedef Eigen::Matrix<float, dimention_n, dimention_n> rotationMatrix;
typedef Eigen::Matrix<float,dimention_n,1> State_vector;


const int tag_id =0;
const float focal_length_uy = 729.0;// [pixel/m * m]
const float focal_length_uz = 1350.0;// [pixel/m * m] 
int had_message_1=0;
float deg2rad= 0.0174532925;
float x=0;
float y=0;
float z=0;
double rotation_roll =0.0;
double rotation_pitch =0.0;
double rotation_yaw =0.0;
//void poseCallback(const std_msgs::Float32MultiArray& msg){
//}

void state_callback(const ardrone_autonomy::Navdata& msg_in)
{

	uint32_t tags_count;
	uint32_t tags_yc[10];
	uint32_t tags_zc[10];
	float tags_distance[10];
	had_message_1=1;
	//Take in states of robot that we are interested in from the navdata topic
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
	rotation_yaw=msg_in.rotZ*deg2rad;

	tags_count=msg_in.tags_count;
//	time_stamp=msg_in.tm;	

	for (uint32_t i=0; i <tags_count; i++){
		tags_distance[i]=msg_in.tags_distance[i]*0.01; // cm to m
		tags_yc[i]=msg_in.tags_xc[i];
		tags_zc[i]=msg_in.tags_yc[i];
		//tags_width[i]=msg_in.tags_width[i];
		//tags_height[i]=msg_in.tags_height[i];
		//tags_orientation[i]=msg_in.tags_orientation[i];
	   }
	 if(tags_count==0)
	 {
	tags_distance[0]=0; // cm to m
	tags_zc[0]=0;
	tags_yc[0]=0;
	 }
	float tag_yc= (float)((int)tags_yc[tag_id]-500)*-1.0;
	float tag_zc= (float)((int)tags_zc[tag_id]-500)*-1.0; // global z vector
	
	ROS_INFO("TF; Tag: %f %f %f",tags_distance[tag_id],tag_yc,tag_zc);
	
	int gam=pow((tag_zc/focal_length_uz),2);
	int alp=pow((tag_yc/focal_length_uy),2);
	x=pow( ( pow(tags_distance[0],2) ) / (gam+1+alp) ,0.5);
	y=((x*tag_yc)/focal_length_uy);
	z=((x*tag_zc)/focal_length_uz);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tag_tf_track");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);
  tf::StampedTransform transRot;
  tf::Transform transform;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  rotationMatrix R;
  ros::Publisher tag_pub;
 geometry_msgs::Vector3 rotated_tag;

  ros::Subscriber nav_sub = node.subscribe("ardrone/navdata", 1, state_callback);
  tag_pub = node.advertise<geometry_msgs::Vector3> ("rot_tag", 1);
while(1){
		transform.setOrigin( tf::Vector3(0, 0, 0) );
		transform.setRotation( tf::Quaternion(rotation_roll,rotation_pitch,rotation_yaw) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "ardrone_base_link"));
		
		
		//R= trans;
		transform.setOrigin( tf::Vector3(x, y, z) );
		ROS_INFO("TF; Displaying point: %f %f %f",x,y,z);
		transform.setRotation( tf::Quaternion(0, 0, 0) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone_base_link", "tracked_tag"));
		
		#if ROS_VERSION_MINIMUM(1,8,0)
  tf::Matrix3x3 trans;      
#else
  btMatrix3x3 trans;
#endif
		//btMatrix3x3 trans;
		trans.setRPY(rotation_roll,rotation_pitch,0.0);
		tf::Vector3 rotatedTrans=trans*tf::Vector3(x, y, z);
		transform.setOrigin( rotatedTrans );
		ROS_INFO("TF; Rotated point: %f %f %f",rotatedTrans[0],rotatedTrans[1],rotatedTrans[2]);
		transform.setRotation( tf::Quaternion(0, 0, 0) );
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone_base_link", "rot_tracked_tag"));
		
		rotated_tag.x =rotatedTrans[0];
		rotated_tag.y=rotatedTrans[1];
		rotated_tag.z=rotatedTrans[2];
		tag_pub.publish(rotated_tag);
		
		ros::spinOnce();
		loop_rate.sleep();
}
return 0;
}
