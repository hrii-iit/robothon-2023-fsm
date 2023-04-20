#include <ros/ros.h>
#include "hrii_robothon_msgs/OpenDoor.h"
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
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;

        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;

        ros::Publisher desired_pose_pub_;
        std::string controller_desired_pose_topic_name_;

        bool activationCallback(hrii_robothon_msgs::OpenDoor::Request& req,
                                hrii_robothon_msgs::OpenDoor::Response& res)
        {
            // ROS_INFO_STREAM("Activate door opening for robot: " << req.robot_id);

            // desired_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/"+req.robot_id+"/"+controller_desired_pose_topic_name_, 1);
            // // while (!(desired_pose_pub_.getNumSubscribers > 0));

            // // Trajectory helper declaration and initialization
            // traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            // if (!traj_helper_->init()) return false;
            // traj_helper_->setTrackingPositionTolerance(0.2);
            // ROS_INFO("Trajectory handler client initialized.");

            // // Initialize gripper and close it
            // gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            // if (!gripper_->init()) return false;
            // if (!gripper_->open(default_closing_gripper_speed_)) return false;
            // ROS_INFO("Gripper client initialized.");
            
            // std::vector<geometry_msgs::Pose> waypoints;
            // double execution_time = 5.0;

            // // Move to an approach pose w.r.t. the world frame
            // waypoints.push_back(req.door_handle_pose.pose);

            // if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to reach the approach pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed...");

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

            // ROS_INFO("Door open.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // StowProbeCableFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_fsm");
    ros::NodeHandle nh("~");

    StowProbeCableFSM fsm(nh);

    ros::spin();

    return 0;
}
