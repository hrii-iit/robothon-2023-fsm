#include <ros/ros.h>
#include "hrii_robothon_msgs/Homing.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

class HomingFSM
{
    public:
        HomingFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
        {
            activation_server_ = nh_.advertiseService("activate", &HomingFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_;
        std::string controller_set_EE_T_task_frame_service_name_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_robothon_msgs::Homing::Request& req,
                                hrii_robothon_msgs::Homing::Response& res)
        {
            ROS_INFO_STREAM("Activate homing for robot: " << req.robot_id);

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

            // Initialize gripper and close it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            if (!gripper_->close(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 5.0;

            ROS_INFO("HOMING FSM STARTED!");

            // Move to an approach pose w.r.t. the world frame
            geometry_msgs::Pose approach_pose = req.home_pose.pose;
            //approach_pose.position.z += 0.02;
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            res.success = true;
            res.message = "";
            return true;
        }
}; // HomingFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "homing_fsm");
    ros::NodeHandle nh("~");

    HomingFSM fsm(nh);

    ros::spin();

    return 0;
}
