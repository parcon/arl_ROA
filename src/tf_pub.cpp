#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Float32MultiArray.h>



void poseCallback(const std_msgs::Float32MultiArray& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.data[0], msg.data[1], msg.data[2]) );
  //ROS_INFO("TF_Display; Kalman_tag: %f %f %f",msg.data[0],msg.data[1],msg.data[2]);
  transform.setRotation( tf::Quaternion(0, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ardrone_base_link", "Kalman_tag"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;
  ros::Rate loop_rate(50);
  //state_pub = node.advertise<std_msgs::Float32MultiArray> ("state_post_KF", 1);
  ros::Subscriber sub = node.subscribe("state_post_KF", 10, poseCallback);

while(1){
		ROS_INFO("TF_Display; loopin");
		ros::spinOnce();
		loop_rate.sleep();
}
return 0;
}
