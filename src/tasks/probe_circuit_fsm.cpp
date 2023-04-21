#include <ros/ros.h>
#include "hrii_robothon_msgs/ProbeCircuit.h"
#include "hrii_robot_msgs/SetPose.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include <eigen_conversions/eigen_msg.h>
#include "hrii_task_board_fsm/utils/ControllerUtils.h"

class ProbeCircuitFSM
{
    public:
        ProbeCircuitFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10.0),
            default_grasping_gripper_force_(5.0),
            desired_contact_force_(10.0),
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose"),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
        {
            activation_server_ = nh_.advertiseService("activate", &ProbeCircuitFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;
        double desired_contact_force_;

        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_;
        std::string controller_set_EE_T_task_frame_service_name_;

        ros::Publisher desired_pose_pub_;
        std::string controller_desired_pose_topic_name_;

        bool activationCallback(hrii_robothon_msgs::ProbeCircuit::Request& req,
                                hrii_robothon_msgs::ProbeCircuit::Response& res)
        {
            ROS_INFO_STREAM("Activate circuit probing for robot: " << req.robot_id);

            controller_set_EE_T_task_frame_client_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_.waitForExistence();

            // Set the new task frame to probe hole in gripper fingertips
            // The task frame is rotated by 45° wrt EE orientation
            hrii_robot_msgs::SetPose set_EE_T_task_frame_srv;
            set_EE_T_task_frame_srv.request.pose_stamped.pose.position.z = -0.01;
            set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.y = -0.3826834; 
            set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 0.9238795; 

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

            // Initialize gripper and open it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            if (!gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized.");
            
            double execution_time = 5.0;

            // Move to probe handle pose w.r.t. the world frame
            if(!traj_helper_->moveToTargetPoseAndWait(req.probe_handle_pose.pose, execution_time))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach the probe.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Grasp the probe
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed...");

            // Extract the probe
            Eigen::Affine3d robot_link0_T_probe_handle;
            tf::poseMsgToEigen(req.probe_handle_pose.pose, robot_link0_T_probe_handle);

            Eigen::Affine3d probe_handle_T_probe_extraction_pose = Eigen::Affine3d::Identity();
            probe_handle_T_probe_extraction_pose.translation() << -0.04, 0.0, 0.0;
            geometry_msgs::Pose desired_pose_msg;
            tf::poseEigenToMsg(robot_link0_T_probe_handle * probe_handle_T_probe_extraction_pose, desired_pose_msg);

            if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.robot_id+" failed to extract the handle.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Approach circuit pose
            desired_pose_msg = req.circuit_pose.pose;
            desired_pose_msg.position.z += 0.1;
            if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.robot_id+" failed to approach the circuit pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            desired_pose_msg = req.circuit_pose.pose;
            desired_pose_msg.position.z += 0.02;
            if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.robot_id+" failed to extract the handle.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Tmp stopping here
            res.success = true;
            res.message = "";
            return true;
            
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

            // Move to approach pose
            desired_pose_msg = req.circuit_pose.pose;
            desired_pose_msg.position.z -= 0.1;
            if(!traj_helper_->moveToTargetPoseAndWait(desired_pose_msg, execution_time))
            {
                res.success = false;
                res.message = req.robot_id+" failed to extract the handle.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            // // Plan trajectory
            // Eigen::Affine3d w_T_door_handle, w_T_center_of_rotation;
            // tf::poseMsgToEigen(req.door_handle_pose.pose, w_T_door_handle);
            // tf::poseMsgToEigen(req.center_of_rotation_pose.pose, w_T_center_of_rotation);

            // ROS_INFO_STREAM("w_T_door_handle:\t" << w_T_door_handle.translation().transpose());
            // ROS_INFO_STREAM("w_T_center_of_rotation\t" << w_T_center_of_rotation.translation().transpose());
            
            // Eigen::Affine3d init_center_of_rotation_T_door_handle = w_T_center_of_rotation.inverse() * w_T_door_handle;
            // ROS_INFO_STREAM("init_center_of_rotation_T_door_handle:\t" << init_center_of_rotation_T_door_handle.translation().transpose());
            
            // double time = 0;
            // double T = req.execution_time;

            // ros::Rate loop_rate(std::round(1.0/req.sampling_time));

            // while (ros::ok() && (time <= req.execution_time))
            // {
            //     time += req.sampling_time;
            //     double t_interp = 10 * pow(time/req.execution_time,3) - 15 * pow(time/req.execution_time,4) + 6 * pow(time/req.execution_time,5);

            //     // Compute the new desired angle
            //     double desired_angle = t_interp * req.final_desired_angle;
            //     ROS_WARN_STREAM(desired_angle);

            //     // Compute desired pose
            //     Eigen::Quaterniond center_of_rotation_T_desired_angle_quat = 
            //         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
            //         Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*
            //         Eigen::AngleAxisd(desired_angle, Eigen::Vector3d::UnitZ());
            //     Eigen::Affine3d center_of_rotation_T_desired_angle_pose;
            //     center_of_rotation_T_desired_angle_pose = Eigen::Affine3d::Identity();
            //     center_of_rotation_T_desired_angle_pose.linear() = center_of_rotation_T_desired_angle_quat.matrix();

            //     Eigen::Affine3d w_T_desired_pose = w_T_center_of_rotation * center_of_rotation_T_desired_angle_pose * init_center_of_rotation_T_door_handle;

            //     geometry_msgs::PoseStamped w_T_desired_pose_msg;
            //     tf::poseEigenToMsg(w_T_desired_pose, w_T_desired_pose_msg.pose);
            //     w_T_desired_pose_msg.header.stamp = ros::Time::now();
            //     w_T_desired_pose_msg.header.frame_id=req.robot_id+"_link0";

            //     // ROS_INFO_STREAM("center_of_rotation_T_desired_angle_pose:\t" << center_of_rotation_T_desired_angle_pose.matrix());
            //     // ROS_INFO_STREAM("w_T_center_of_rotation:\t" << w_T_center_of_rotation.matrix());
            //     // ROS_INFO_STREAM("center_of_rotation_T_desired_angle_pose:\t" << center_of_rotation_T_desired_angle_pose.matrix());

            //     desired_pose_pub_.publish(w_T_desired_pose_msg);

            //     loop_rate.sleep();
            // }

            // if (!gripper_->open(default_closing_gripper_speed_)) return false;

            

            // Set the new task frame to probe hole in gripper fingertips
            // The task frame is rotated by 45° wrt EE orientation
            hrii_robot_msgs::SetPose original_EE_T_task_frame_srv;
            original_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 1.0; 

            if (!controller_set_EE_T_task_frame_client_.call(original_EE_T_task_frame_srv))
            {
                ROS_ERROR("Error calling set_EE_T_task_frame ROS service.");
                return false;
            }
            else if (!original_EE_T_task_frame_srv.response.success)
            {
                ROS_ERROR("Failure setting EE_T_task_frame. Exiting.");
                return false;
            }

            ROS_INFO("Circuit probed.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // ProbeCircuitFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_fsm");
    ros::NodeHandle nh("~");

    ProbeCircuitFSM fsm(nh);

    ros::spin();

    return 0;
}
