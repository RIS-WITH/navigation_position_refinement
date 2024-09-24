
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf_test");
  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
  sleep(1);
  geometry_msgs::PoseStamped p;
  p.header.frame_id = "nikon";
  p.pose.position.x = 1;
  p.pose.position.y = 1;
  p.pose.orientation.x=0;
  p.pose.orientation.y=0;
  p.pose.orientation.z=0.707;
  p.pose.orientation.w=0.707;

  auto val = tfBuffer_.transform(p, "base_footprint");
  std::cout << val.header.frame_id << std::endl;
  std::cout << "x : " << val.pose.position.x << " y : " << val.pose.position.y << " z : " << val.pose.position.z << std::endl;

  return 0;
};