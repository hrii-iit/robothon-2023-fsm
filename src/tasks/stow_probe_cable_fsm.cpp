#include <ros/ros.h>
#include "hrii_robothon_msgs/StowProbeCable.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include <eigen_conversions/eigen_msg.h>

class StowProbeCableFSM
{
    public:
        StowProbeCableFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10),
            default_grasping_gripper_force_(5.0),
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose")
        {
            activation_server_ = nh_.advertiseService("activate", &StowProbeCableFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr probe_holder_robot_gripper_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;

        HRII::TrajectoryHelper::Ptr probe_holder_robot_traj_helper_;

        ros::ServiceServer activation_server_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_;
        std::string controller_set_EE_T_task_frame_service_name_;

        ros::Publisher desired_pose_pub_;
        std::string controller_desired_pose_topic_name_;

        bool activationCallback(hrii_robothon_msgs::StowProbeCable::Request& req,
                                hrii_robothon_msgs::StowProbeCable::Response& res)
        {

            ROS_INFO_STREAM("Activate probe cable stowing. Probe holder: " << req.probe_holder_robot_id << " - Cable stower: " << req.cable_stower_robot_id);

            // Change task frame of Probe holder robot
            controller_set_EE_T_task_frame_client_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.probe_holder_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.probe_holder_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_.waitForExistence();

            // Set the new task frame to probe hole in gripper fingertips
            // The task frame is rotated by 45° wrt EE orientation
            hrii_robot_msgs::SetPose set_EE_T_task_frame_srv;
            set_EE_T_task_frame_srv.request.pose_stamped.pose.position.z = -0.01;
            set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.y = -0.3826834; 
            set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = -0.9238795; 

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
            probe_holder_robot_traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.probe_holder_robot_id+"/trajectory_handler");
            if (!probe_holder_robot_traj_helper_->init()) return false;
            probe_holder_robot_traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized (Probe holder robot).");

            // Initialize gripper
            probe_holder_robot_gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.probe_holder_robot_id+"/gripper");
            if (!probe_holder_robot_gripper_->init()) return false;
            // if (!gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized (Probe holder robot).");
            
            double execution_time = 20.0;

            // // Store probe handle pose
            // geometry_msgs::Pose approach_probe_handle_pose, probe_handle_pose;
            // probe_handle_pose = req.probe_handle_pose.pose;
            // approach_probe_handle_pose = probe_handle_pose;

            // // Move above the probe handle pose
            // approach_probe_handle_pose.position.z += 0.15;
            // if(!traj_helper_->moveToTargetPoseAndWait(approach_probe_handle_pose, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to reach the approach the probe.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Move to probe handle pose
            // execution_time = 4.0;
            // if(!traj_helper_->moveToTargetPoseAndWait(probe_handle_pose, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to reach the approach the probe.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Grasp the probe
            // // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed...");

            // // Extract the probe
            // Eigen::Affine3d robot_link0_T_probe_handle;
            // tf::poseMsgToEigen(probe_handle_pose, robot_link0_T_probe_handle);

            // Eigen::Affine3d probe_handle_T_probe_extraction_pose = Eigen::Affine3d::Identity();
            // probe_handle_T_probe_extraction_pose.translation() << 0, 0.0, -0.03;
            // geometry_msgs::Pose desired_pose_msg;
            // tf::poseEigenToMsg(robot_link0_T_probe_handle * probe_handle_T_probe_extraction_pose, desired_pose_msg);
            // execution_time = 5.0;

            // if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to extract the handle.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Approach circuit pose
            // desired_pose_msg = req.circuit_pose.pose;
            // desired_pose_msg.position.z += 0.1;
            // execution_time = 8.0;
            // if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to approach the circuit pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Approach circuit pose
            // desired_pose_msg = req.circuit_pose.pose;
            // desired_pose_msg.position.z += 0.04;
            // if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to extract the handle.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Switch to task force in Z-axis
            // geometry_msgs::WrenchStamped desired_wrench;
            // desired_wrench.header.stamp = ros::Time::now();
            // desired_wrench.wrench.force.z = desired_contact_force_;
            // // desired_wrench.wrench.force.z = 0.0;

            // if (!applyContactForce(nh_, req.robot_id,
            //                 "cart_hybrid_motion_force_controller",
            //                 hrii_robot_msgs::TaskSelection::Request::Z_LIN,
            //                 desired_wrench))
            // {
            //     ROS_ERROR_STREAM("Contact force application failed.");
            //     return false;
            // }

            // // Move to approach pose
            // desired_pose_msg = req.circuit_pose.pose;
            // desired_pose_msg.position.z += 0.15;
            // if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to extract the handle.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }




            // // Task frame back to original one
            // hrii_robot_msgs::SetPose original_EE_T_task_frame_srv;
            // original_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 1.0; 

            // if (!controller_set_EE_T_task_frame_client_.call(original_EE_T_task_frame_srv))
            // {
            //     ROS_ERROR("Error calling set_EE_T_task_frame ROS service.");
            //     return false;
            // }
            // else if (!original_EE_T_task_frame_srv.response.success)
            // {
            //     ROS_ERROR("Failure setting EE_T_task_frame. Exiting.");
            //     return false;
            // }

            ROS_INFO("Probe cable stowed.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // StowProbeCableFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stow_probe_cable_fsm");
    ros::NodeHandle nh("~");

    StowProbeCableFSM fsm(nh);

    ros::spin();

    return 0;
}
