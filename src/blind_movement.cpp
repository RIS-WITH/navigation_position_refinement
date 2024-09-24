#include <ros/ros.h>
#include "navigation_position_refinement/Parameters.h"
#include <navigation_position_refinement/BlindMovementAction.h>

#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
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
    BlindMovement(ros::NodeHandle& nh, BlindParams_t params) : nh_(nh), params_(params),
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
        ROS_INFO_NAMED("move blind server", "Ready");

    }

    void onNewGoal(const navigation_position_refinement::BlindMovementGoalConstPtr goal)
    {
        ros::Time timeZero(0.0);

        navigation_position_refinement::BlindMovementResult asResult;

        //we will record transforms here
        tf2::Stamped<tf2::Transform> current_transform;

        double roll, pitch, yaw;
        bool goal_x, goal_y, goal_theta = false;

        //record the starting transform from the odometry to the base frame
        auto start_transform_msg = tfBuffer_.lookupTransform("base_footprint", "odom_combined",
                                                             ros::Time(0), ros::Duration(1.0));
        tf2::fromMsg(start_transform_msg, start_transform_);
        //we will be sending commands of type "twist"
        geometry_msgs::Twist base_cmd;
        //assign velocity sign
        if (goal->y_movement != 0.) base_cmd.linear.y = ((goal->y_movement > 0) ? 1. : -1.) * goal->linear_velocity;
        if (goal->x_movement != 0.) base_cmd.linear.x = ((goal->x_movement > 0) ? 1. : -1.) * goal->linear_velocity;
        if (goal->theta_rotation != 0.)
            base_cmd.angular.z = ((goal->theta_rotation > 0) ? 1. : -1.) * goal->angular_velocity;

        // distance = std::abs(distance);

        bool done = false;
        while (!done && nh_.ok())
        {
            //send the drive command
            cmdVelPub_.publish(base_cmd);
            params_.controler_period.sleep();
            //get the current transform
            try
            {
                auto current_transform_msg = tfBuffer_.lookupTransform("base_footprint", "odom_combined",
                                                                       ros::Time(0), ros::Duration(1.0));
                tf2::fromMsg(current_transform_msg, current_transform);
            }
            catch (tf2::TransformException ex)
            {
                ROS_ERROR("%s", ex.what());
                break;
            }
            //see how far we've traveled

            tf2::Transform relative_transform = current_transform.inverse() * start_transform_;
            auto vec_error = relative_transform.getOrigin();
            std::cout << "val : x:" << vec_error[0] << " y:" << vec_error[1] << std::endl;
            relative_transform.getBasis().getRPY(roll, pitch, yaw);

            if (abs(vec_error[0]) > abs(goal->x_movement))
            {
                if (not goal_x)
                    std::cout << "distance x ok " << std::endl;
                base_cmd.linear.x = 0;
                goal_x = true;

            }
            if (abs(vec_error[1]) > abs(goal->y_movement))
            {
                if (not goal_y)
                    std::cout << "distance y ok " << std::endl;
                base_cmd.linear.y = 0;
                goal_y = true;


            }
            if (abs(yaw) > abs(goal->theta_rotation))
            {
                if (not goal_theta)
                    std::cout << "distance theta ok " << std::endl;
                base_cmd.angular.z = 0;
                goal_theta = true;


            }
            ros::Time lastControl = ros::Time::now();
            ros::Time now = lastControl;
            ros::Time actionStart = ros::Time::now();

            navigation_position_refinement::BlindMovementFeedback feedback;
            feedback.action_start = actionStart;
            feedback.distance_x_to_goal = vec_error[0];
            feedback.distance_y_to_goal = vec_error[1];
            feedback.angular_to_goal = vec_error[2];
            blind_movement_action_server_.publishFeedback(feedback);
            if (goal_theta and goal_x and goal_y) done = true;
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
    actionlib::SimpleActionServer<navigation_position_refinement::BlindMovementAction> blind_movement_action_server_;
    ros::Publisher cmdVelPub_;
    tf2::Stamped<tf2::Transform> start_transform_;


};


int main(int argc, char** argv)
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
