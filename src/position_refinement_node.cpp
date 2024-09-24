#include <ros/ros.h>
#include "navigation_position_refinement/Parameters.h"
#include <navigation_position_refinement/PositionRefinementAction.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>

struct Params_t
{
  double angular_gain_P;
  double angular_gain_I;
  double gain_P;
  double gain_I;
  double distance_tolerance;
  double angular_tolerance;
  double max_integral;
  double max_angular_integral;
  double max_speed;
  double max_angular_speed;
  ros::Duration controler_period;
  std::string reference_frame;
  std::string command_topic;
};

class PositionRefinement
{
public:
  PositionRefinement(ros::NodeHandle &nh, Params_t params) : nh_(nh), params_(params),
                                                             position_refinement_ActionServer_(nh,
                                                                                               "position_refinement",
                                                                                               boost::bind(
                                                                                                   &PositionRefinement::onNewGoal,
                                                                                                   this, _1),
                                                                                               false)
  {
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(tfBuffer_);
    cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>(params_.command_topic, 10, true);
    position_refinement_ActionServer_.start();
    ROS_INFO_NAMED("position refinement server", "Ready");
  }

  void onNewGoal(const navigation_position_refinement::PositionRefinementGoalConstPtr &goal)
  {
    ros::Time timeZero(0.0);

    navigation_position_refinement::PositionRefinementResult asResult;
    geometry_msgs::PoseStamped poseWanted = goal->goal;
    if (!tfBuffer_.canTransform(params_.reference_frame, poseWanted.header.frame_id, timeZero))
    {
      ROS_WARN_STREAM_NAMED("navigation position refinement",
                            "Can't transform frame '" << poseWanted.header.frame_id << "' in '" << params_.reference_frame
                                                      << "'. Aborting action.");
      asResult.error_code = asResult.NO_MAP_TO_OBJ_TRANSFORM;
      position_refinement_ActionServer_.setAborted(asResult);
      return;
    }
    std::vector<double> errors = getErrors(poseWanted);

    double cmd = 0.0, integral, cmd_angle = 0.0, integral_angle;
    ros::Time lastControl = ros::Time::now();
    ros::Time now = lastControl;
    ros::Time actionStart = ros::Time::now();
    geometry_msgs::Twist cmdVel;
    integral = 0.0;
    navigation_position_refinement::PositionRefinementFeedback feedback;
    feedback.action_start = actionStart;
    feedback.distance_to_goal = errors[0];
    feedback.angular_to_goal = errors[1];
    position_refinement_ActionServer_.publishFeedback(feedback);
    bool stop_condition = false;

    while (!stop_condition && ros::ok())
    {
      errors = getErrors(poseWanted);
      feedback.distance_to_goal = errors[0];
      feedback.angular_to_goal = errors[1];

      position_refinement_ActionServer_.publishFeedback(feedback);
      now = ros::Time::now();
      if (position_refinement_ActionServer_.isPreemptRequested())
      {
        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        cmdVelPub_.publish(cmdVel);
        asResult.error_code = asResult.PREEMPTED;
        asResult.action_end = ros::Time::now();
        position_refinement_ActionServer_.setPreempted(asResult);
        return;
      }
      integral = std::max(-params_.max_integral,
                          std::min(params_.max_integral, integral + errors[0] * (now - lastControl).toSec()));
      cmd = std::max(-params_.max_speed,
                     std::min(params_.max_speed, params_.gain_P * errors[0] + params_.gain_I * integral));
      auto val = tfBuffer_.transform(poseWanted, params_.reference_frame);
      double angle = std::atan2(val.pose.position.y, val.pose.position.x);

      integral_angle = std::max(-params_.max_angular_integral,
                                std::min(params_.max_angular_integral, integral_angle + errors[1] * (now - lastControl).toSec()));
      double angular_factor = 1;
      cmd_angle = std::max(-params_.max_angular_speed,
                           std::min(params_.max_angular_speed, params_.angular_gain_P * errors[1] + params_.angular_gain_I * integral_angle));

      if (errors[0] > params_.distance_tolerance)
      {
        cmdVel.linear.x = cmd * std::cos(angle);
        cmdVel.linear.y = cmd * std::sin(angle);
      }
      else
      {
        cmdVel.linear.x = 0.;
        cmdVel.linear.y = 0.;
      }
      if (std::abs(errors[1]) > params_.angular_tolerance)
      {
        cmdVel.angular.z = cmd_angle * angular_factor;
      }

      cmdVelPub_.publish(cmdVel);
      lastControl = now;
      params_.controler_period.sleep();

      stop_condition = ((errors[0] < params_.distance_tolerance) && (std::abs(errors[1]) < params_.angular_tolerance));
      std::cout << "stop_condition : " << stop_condition << std::endl;
    }
    cmdVel.linear.x = 0.0;
    cmdVel.linear.y = 0.0;
    cmdVel.angular.z = 0.0;
    asResult.action_end = ros::Time::now();
    for (unsigned int j = 0; j < 2; j++)
    { // TODO (guilhembn): does not work with morse otherwise, check on real robot
      cmdVelPub_.publish(cmdVel);
      params_.controler_period.sleep();
    }
    cmdVelPub_.publish(cmdVel);
    asResult.error_code = asResult.SUCCESS;
    position_refinement_ActionServer_.setSucceeded(asResult);
  }

  std::vector<double> getErrors(const geometry_msgs::PoseStamped &poseWanted)
  {
    auto val = tfBuffer_.transform(poseWanted, params_.reference_frame);
    tf2::Quaternion q(val.pose.orientation.x, val.pose.orientation.y, val.pose.orientation.z,
                      val.pose.orientation.w);
    tf2::Vector3 v(val.pose.position.x, val.pose.position.y, val.pose.position.z);
    tf2::Transform transform(q, v);
    double error = transform.getOrigin().length();
    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);
    std::cout << "current distance error : " << error << " angular error : " << yaw << std::endl;
    std::vector<double> vec{error, yaw};
    return vec;
  }

private:
  ros::NodeHandle nh_;
  Params_t params_;
  tf2_ros::Buffer tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  actionlib::SimpleActionServer<navigation_position_refinement::PositionRefinementAction> position_refinement_ActionServer_;
  ros::Publisher cmdVelPub_;
};

int main(int argc, char **argv)
{
  std::cout << "<<<<<<<<<<<<<<<<<< Launching navigation position refinement <<<<<<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
  ros::init(argc, argv, "navigation_position_refinement");
  ros::NodeHandle n;

  Parameters params;
  params.insert(Parameter("gain P", {"-p", "--gain_p"}, {"0.1"}));
  params.insert(Parameter("gain I", {"-i", "--gain_i"}, {"0.05"}));

  params.insert(Parameter("angular gain P", {"-ap", "--angular_gain_p"}, {"100"}));
  params.insert(Parameter("angular gain I", {"-ai", "--angular_gain_i"}, {"1"}));

  params.insert(Parameter("angular tolerance", {"-a", "--angular_tolerance"}, {"0.034"}));
  params.insert(Parameter("distance tolerance", {"-d", "--distance_tolerance"}, {"0.2"}));

  params.insert(Parameter("max speed", {"-ms", "--max_speed"}, {"0.2"}));
  params.insert(Parameter("max angular speed", {"-as", "--angular_max_speed"}, {"0.1"}));

  params.insert(Parameter("max angular integral", {"-ami", "--angular_max_integral"}, {"5.0"}));
  params.insert(Parameter("max integral", {"-mi", "--max_integral"}, {"5.0"}));

  params.insert(Parameter("controller period", {"-cp", "--controller_period"}, {"0.1"}));

  params.insert(Parameter("reference frame", {"-f", "--reference_frame"}, {"base_footprint"}));
  params.insert(Parameter("command topic", {"-t", "--command_topic"}, {"/cmd_vel"}));

  params.set(argc, argv);
  params.display();

  Params_t parsed_params;
  parsed_params.gain_P = std::stod(params.at("gain P").getFirst());
  parsed_params.gain_I = std::stod(params.at("gain I").getFirst());

  parsed_params.angular_gain_P = std::stod(params.at("angular gain P").getFirst());
  parsed_params.angular_gain_I = std::stod(params.at("angular gain I").getFirst());

  parsed_params.angular_tolerance = std::stod(params.at("angular tolerance").getFirst());
  parsed_params.distance_tolerance = std::stod(params.at("distance tolerance").getFirst());

  parsed_params.max_speed = std::stod(params.at("max speed").getFirst());
  parsed_params.max_integral = std::stod(params.at("max integral").getFirst());

  parsed_params.max_angular_speed = std::stod(params.at("max angular speed").getFirst());
  parsed_params.max_angular_integral = std::stod(params.at("max angular integral").getFirst());

  parsed_params.controler_period.fromSec(std::stod(params.at("controller period").getFirst()));

  parsed_params.reference_frame = params.at("reference frame").getFirst();
  parsed_params.command_topic = params.at("command topic").getFirst();

  PositionRefinement position(n, parsed_params);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::waitForShutdown();

  //   cedural::RosInterface ros_interface(&n, onto_manipulators, time_manipulators, params.at("name").getFirst());
  //    if(ros_interface.init(params.at("yaml_path").getFirst(), stod(params.at("ttl").getFirst()), stoi(params.at("max_size").getFirst())))
  //        ros_interface.run();
  return 0;
}
