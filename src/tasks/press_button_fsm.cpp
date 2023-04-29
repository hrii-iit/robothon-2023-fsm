#include <ros/ros.h>

#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include "hrii_robothon_msgs/PressButton.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_task_board_fsm/utils/ControllerUtils.h"

class PressButtonFSM
{
    public:
        PressButtonFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10),
            desired_contact_force_(10),
            controller_name_("cart_hybrid_motion_force_controller"),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
        {
            activation_server_ = nh_.advertiseService("activate", &PressButtonFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }

    private:
        ros::NodeHandle nh_;

        HRII::TrajectoryHelper::Ptr traj_helper_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_;
        std::string controller_set_EE_T_task_frame_service_name_;

        ros::ServiceServer activation_server_;
        //ros::Publisher desired_wrench_pub_;
        double desired_contact_force_;

        std::string controller_name_;

        bool activationCallback(hrii_robothon_msgs::PressButton::Request& req,
                                hrii_robothon_msgs::PressButton::Response& res)
        {
            ROS_INFO_STREAM("Activate press button interface for robot: " << req.robot_id);

            controller_set_EE_T_task_frame_client_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_.waitForExistence();

            // Set task frame to the end-effector frame
            hrii_robot_msgs::SetPose set_EE_T_task_frame_srv;
            set_EE_T_task_frame_srv.request.pose_stamped.pose.position.z = 0; 
            set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 1.0; 

            if (!controller_set_EE_T_task_frame_client_.call(set_EE_T_task_frame_srv))
            {
                ROS_ERROR("Error calling set_EE_T_task_frame ROS service.");
                return false;
            }
            else if (!set_EE_T_task_frame_srv.response.success)
            {
                ROS_ERROR("Failure setting EE_T_task_frame. Exiting.");
                return false;
            }

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize ROS publishers and subscribers
            
            // Initialize gripper and close it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            if (!gripper_->close(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized.");

            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 5.0;

            ROS_INFO("PRESS BUTTON FSM STARTED!");

            // Move to an approach pose w.r.t. the world frame
            geometry_msgs::Pose approach_pose = req.button_pose.pose;
            // approach_pose.position.z += 0.02;
            approach_pose.position.z = 0.110; // absolute

            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the first approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move to an approach pose w.r.t. the world frame
            // approach_pose.position.z -= 0.015;
            approach_pose.position.z = 0.098; // absolute
            execution_time = 2.0;

            waypoints.erase(waypoints.begin());
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the second approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Switch to task force in Z-axis
            geometry_msgs::WrenchStamped desired_wrench;
            desired_wrench.header.stamp = ros::Time::now();
            desired_wrench.wrench.force.z = desired_contact_force_;

            if (!applyContactForce(nh_, req.robot_id,
                            "cart_hybrid_motion_force_controller",
                            hrii_robot_msgs::TaskSelection::Request::Z_LIN,
                            desired_wrench))
            {
                ROS_ERROR_STREAM("Contact force application failed.");
                return false;
            }
            
            waypoints.erase(waypoints.begin());

            // We move the robot back to the approach pose upon the button
            approach_pose.position.z += 0.055; // button_pose + 0.05m
            waypoints.push_back(approach_pose);
            execution_time = 1.0;

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to return to the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            res.success = true;
            res.message = "";
            return true;
        }
}; // PressButtonFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "press_button_fsm");
    ros::NodeHandle nh("~");

    PressButtonFSM fsm(nh);

    ros::spin();

    return 0;
}
