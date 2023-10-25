
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
    std::cout << "x : " << val.pose.position.x << " y : " << val.pose.position.y << " z : " << val.pose.position.z
              << std::endl;
    // tf2::Transform transform;
    // tf2::fromMsg(val,transform);

//    tf2::Vector3 vec(val.pose.position.x, val.pose.position.y, val.pose.position.z);
    // std::cout << "length : " << transform.getOrigin().length() << "  length2 : " << transform.getOrigin().length2() << std::endl;
    // double roll, pitch, yaw;
    // transform.getBasis().getRPY(roll, pitch, yaw);
    // std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;






//  tf2::Quaternion q1(0, 0, 1, 0);
//  tf2::Quaternion q2(0, 0, 0.707, 0.707);
//  tf2::Quaternion q1_inv = q1.inverse();
//  std::cout << q1_inv << std::endl;
//  tf2::Quaternion qd = q1_inv*q2;// good calcul with matrix et yaw
//  qd.normalize();
//
//  std::cout << "q1 : " << q1 << " angle : " << q1.getAngle()<< std::endl;
//  tf2::Matrix3x3 m1(q1);
//  double roll, pitch, yaw;
//  m1.getRPY(roll, pitch, yaw);
//  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
//
//  std::cout << "q2 : " << q2 << " angle : " << q2.getAngle()<< std::endl;
//  tf2::Matrix3x3 m2(q2);
////   double roll, pitch, yaw;
//  m2.getRPY(roll, pitch, yaw);
//  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
//
//  std::cout << "q1_inv : " << q1_inv << " angle : " << q1_inv.getAngle()<< std::endl;
//  tf2::Matrix3x3 m1_inv(q1_inv);
////   double roll, pitch, yaw;
//  m1_inv.getRPY(roll, pitch, yaw);
//  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
//  std::cout << "qd : " << q1_inv << " angle : " << qd.getAngle()<< std::endl;
//  tf2::Matrix3x3 md(qd);
////   double roll, pitch, yaw;
//  md.getRPY(roll, pitch, yaw);
//  std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
//  std::cout << "fct angle q1.angle(q2)" << q1.angle(q2) << std::endl;


    return 0;
};