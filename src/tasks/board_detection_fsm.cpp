#include <ros/ros.h>

#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include "hrii_robothon_msgs/BoardDetection.h"
#include "hrii_robothon_msgs/BoardLocalization.h"
#include "hrii_task_board_fsm/utils/ControllerUtils.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

class BoardDetectionFSM
{
    public:
        BoardDetectionFSM(ros::NodeHandle& nh) : 
            nh_(nh), tf_listener_{tf_buffer_}
        {
            board_localization_service_name_ = "/robothon/board_localization";
            board_localization_client_ = nh_.serviceClient<hrii_robothon_msgs::BoardDetection>(board_localization_service_name_);

            activation_server_ = nh_.advertiseService("activate", &BoardDetectionFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }

    private:
        ros::NodeHandle nh_;

        HRII::TrajectoryHelper::Ptr traj_helper_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;

        ros::ServiceServer activation_server_;
        double desired_contact_force_;

        std::string board_localization_service_name_;
        ros::ServiceClient board_localization_client_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        bool activationCallback(hrii_robothon_msgs::BoardDetection::Request& req,
                                hrii_robothon_msgs::BoardDetection::Response& res)
        {
            ROS_INFO_STREAM("Activate board detection for robot: " << req.robot_id);

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 4.0;

            ROS_INFO("BOARD DETECTION FSM STARTED!");

            // Call board localization srv to retrieve world to task_board_base_link tf
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(board_localization_service_name_) << " ROS service...");
            board_localization_client_.waitForExistence();

            hrii_robothon_msgs::BoardLocalization board_localization_srv;

            if (!board_localization_client_.call(board_localization_srv))
            {
                ROS_ERROR("Error calling board localization service.");
                return false;
            }
            else if (!board_localization_srv.response.success)
            {
                ROS_ERROR("Failure localizing board. Exiting.");
                return false;
            }

            // Move above the red button
            geometry_msgs::TransformStamped redButtonTransform;
            try{
                redButtonTransform = tf_buffer_.lookupTransform(req.robot_id+"_link0", "task_board_red_button_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << req.robot_id << "_link0 and task_board_red_button_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << req.robot_id <<"_link0 and task_board_red_button_link NOT found!");
            }
            geometry_msgs::Pose red_button_pose;

            // Use homing orientation to let camera face downwards
            red_button_pose.orientation = req.homing_pose.pose.orientation;

            red_button_pose.position.x = redButtonTransform.transform.translation.x;
            red_button_pose.position.y = redButtonTransform.transform.translation.y;
            // 24 cm below homing on the z-axis
            red_button_pose.position.z = req.homing_pose.pose.position.z - 0.24;
            // red_button_pose.position.z += -0.24; - 0.24;

            waypoints.push_back(red_button_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to go above the red button pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            
            // Call again board localization srv to retrieve world to task_board_base_link tf (more accurately)
            if (!board_localization_client_.call(board_localization_srv))
            {
                ROS_ERROR("Error calling board localization service.");
                return false;
            }
            else if (!board_localization_srv.response.success)
            {
                ROS_ERROR("Failure localizing board. Exiting.");
                return false;
            }

            // We move the robot back to the homing pose
            waypoints.push_back(req.homing_pose.pose);
            execution_time = 4.0;

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to return to the homing pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            res.success = true;
            res.message = "";
            return true;
        }
}; // BoardDetectionFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "board_detection_fsm");
    ros::NodeHandle nh("~");

    BoardDetectionFSM fsm(nh);

    ros::spin();

    return 0;
}
