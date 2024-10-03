#include <ros/ros.h>
#include "navigation_position_refinement/Parameters.h"
#include <navigation_position_refinement/BlindMovementAction.h>

#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>

struct BlindParams_t
{
  ros::Duration controler_period;
  std::string reference_frame;
  std::string command_topic;
};

class BlindMovement
{
public:
  BlindMovement(ros::NodeHandle &nh, BlindParams_t params) : nh_(nh), params_(params),
                                                             blind_movement_action_server_(nh,
                                                                                           "blind_movement",
                                                                                           boost::bind(
                                                                                               &BlindMovement::onNewGoal,
                                                                                               this, _1),
                                                                                           false)
  {
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
    cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(params_.command_topic, 10, true);
    blind_movement_action_server_.start();
    // tfBroadcaster_ = std::make_shared<tf::TransformBroadcaster>
    ROS_INFO_NAMED("move blind server", "Ready");
  }

  void onNewGoal(const navigation_position_refinement::BlindMovementGoalConstPtr goal)
  {
    std::cout << "##### NEW GOAL #####" << std::endl;
    ros::Time timeZero(0.0);

    navigation_position_refinement::BlindMovementResult asResult;

    // we will record transforms here
    tf2::Stamped<tf2::Transform> current_transform;

    double roll_from_start, pitch_from_start, yaw_from_start;
    bool goal_x = false, goal_y = false, goal_theta = false;
    double integral_x = 0, integral_y = 0, integral_yaw = 0;

    // record the starting transform from the odometry to the base frame
    auto start_transform_msg = tfBuffer_.lookupTransform("odom_combined", "base_footprint",
                                                         ros::Time(0), ros::Duration(1.0));
    tf2::fromMsg(start_transform_msg, start_transform_);
    // we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.y = 0;
    base_cmd.linear.x = 0;
    base_cmd.angular.z = 0;

    ros::Time action_start_time = ros::Time::now();
    ros::Time lastControl = ros::Time::now();
    ros::Time now = lastControl;

    bool done = false;
    while (!done && nh_.ok())
    {
      // send the drive command
      cmdVelPub_.publish(base_cmd);
      params_.controler_period.sleep();

      now = ros::Time::now();

      // get the current transform
      try
      {
        auto current_transform_msg = tfBuffer_.lookupTransform("odom_combined", "base_footprint",
                                                               ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(current_transform_msg, current_transform);
      }
      catch (tf2::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        break;
      }
      // see how far we've traveled

      tf2::Transform relative_transform = current_transform.inverse() * start_transform_;
      auto pose_from_start = relative_transform.getOrigin();
      std::cout << "### " << -pose_from_start[0] << " : " << -pose_from_start[1] << " : " << -pose_from_start[2] << std::endl;
      relative_transform.getBasis().getRPY(roll_from_start, pitch_from_start, yaw_from_start);

      double error_x = goal->x_movement + pose_from_start[0];
      double error_y = goal->y_movement + pose_from_start[1];
      double error_yaw = goal->theta_rotation + yaw_from_start;

      double max_integral_lin = 1.0;
      double max_integral_ang = 3.0;
      integral_x = std::max(-max_integral_lin, std::min(max_integral_lin, integral_x + error_x * (now - lastControl).toSec()));
      integral_y = std::max(-max_integral_lin, std::min(max_integral_lin, integral_y + error_y * (now - lastControl).toSec()));
      integral_yaw = std::max(-max_integral_ang, std::min(max_integral_ang, integral_yaw + error_yaw * (now - lastControl).toSec()));

      std::cout << "error x: " << error_x << " | error y: " << error_y << " | error yaw " << error_yaw << std::endl;
      std::cout << "integ x: " << integral_x << " | integ y: " << integral_y << " | integ yaw " << integral_yaw << std::endl;

      double P_lin = 1.45;
      double P_ang = 0.9;
      double I_lin = 0.0; // 2;
      double I_ang = 0.05;
      base_cmd.linear.x = std::max(-(double)goal->linear_velocity, std::min((double)goal->linear_velocity, P_lin * error_x + I_lin * integral_x));
      base_cmd.linear.y = std::max(-(double)goal->linear_velocity, std::min((double)goal->linear_velocity, P_lin * error_y + I_lin * integral_y));
      base_cmd.angular.z = std::max(-(double)goal->angular_velocity, std::min((double)goal->angular_velocity, P_ang * error_yaw + I_ang * integral_yaw));
      std::cout << "cmd = x: " << base_cmd.linear.x << " y: " << base_cmd.linear.y << " yaw: " << base_cmd.angular.z << std::endl;

      goal_x = (std::abs(error_x) < 0.007) || (goal->linear_velocity == 0);
      goal_y = (std::abs(error_y) < 0.007) || (goal->linear_velocity == 0);
      goal_theta = (std::abs(error_yaw) < 0.5 * M_PI / 180.) || (goal->angular_velocity == 0);

      lastControl = now;

      navigation_position_refinement::BlindMovementFeedback feedback;
      feedback.action_start = action_start_time;
      feedback.distance_x_to_goal = -pose_from_start[0];
      feedback.distance_y_to_goal = -pose_from_start[1];
      feedback.angular_to_goal = -yaw_from_start;
      blind_movement_action_server_.publishFeedback(feedback);
      if (goal_theta && goal_x && goal_y)
        done = true;
    }

    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    base_cmd.angular.z = 0.0;
    asResult.action_end = ros::Time::now();
    cmdVelPub_.publish(base_cmd);
    asResult.error_code = asResult.SUCCESS;
    blind_movement_action_server_.setSucceeded(asResult);
  }

private:
  ros::NodeHandle nh_;
  BlindParams_t params_;
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  // std::shared_ptr<tf_ros::TransformBroadcaster> tfBroadcaster_;
  actionlib::SimpleActionServer<navigation_position_refinement::BlindMovementAction> blind_movement_action_server_;
  ros::Publisher cmdVelPub_;
  tf2::Stamped<tf2::Transform> start_transform_;
};

int main(int argc, char **argv)
{
  std::cout << "<<<<<<<<<<<<<<<<<< Launching navigation move blind  <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
  ros::init(argc, argv, "navigation_move_blind");
  ros::NodeHandle n;

  Parameters params;

  params.insert(Parameter("controller period", {"-cp", "--controller_period"}, {"0.1"}));

  params.insert(Parameter("reference frame", {"-f", "--reference_frame"}, {"base_footprint"}));
  params.insert(Parameter("command topic", {"-t", "--command_topic"}, {"/cmd_vel"}));

  params.set(argc, argv);
  params.display();

  BlindParams_t parsed_params;

  parsed_params.controler_period.fromSec(std::stod(params.at("controller period").getFirst()));

  parsed_params.reference_frame = params.at("reference frame").getFirst();
  parsed_params.command_topic = params.at("command topic").getFirst();

  BlindMovement blind(n, parsed_params);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
