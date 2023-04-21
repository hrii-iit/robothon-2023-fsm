#include <ros/ros.h>
#include "hrii_robothon_msgs/OpenDoor.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include <eigen_conversions/eigen_msg.h>

class OpenDoorFSM
{
    public:
        OpenDoorFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10),
            default_grasping_gripper_force_(5.0),
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose")
        {
            activation_server_ = nh_.advertiseService("activate", &OpenDoorFSM::activationCallback, this);
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
            ROS_INFO_STREAM("Activate door opening for robot: " << req.robot_id);

            desired_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/"+req.robot_id+"/"+controller_desired_pose_topic_name_, 1);

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize gripper and close it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            if (!gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 5.0;

            // Store door handle pose and center of rotation pose
            geometry_msgs::Pose door_handle_pose, center_of_rotation_pose;
            door_handle_pose = req.door_handle_pose.pose;
            center_of_rotation_pose = req.center_of_rotation_pose.pose;

            // Move to an approach pose w.r.t. the world frame
            waypoints.push_back(door_handle_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed...");

            // Plan trajectory
            Eigen::Affine3d w_T_door_handle, w_T_center_of_rotation;
            tf::poseMsgToEigen(door_handle_pose, w_T_door_handle);
            tf::poseMsgToEigen(center_of_rotation_pose, w_T_center_of_rotation);

            ROS_INFO_STREAM("w_T_door_handle:\t" << w_T_door_handle.translation().transpose());
            ROS_INFO_STREAM("w_T_center_of_rotation\t" << w_T_center_of_rotation.translation().transpose());
            
            Eigen::Affine3d init_center_of_rotation_T_door_handle = w_T_center_of_rotation.inverse() * w_T_door_handle;
            ROS_INFO_STREAM("init_center_of_rotation_T_door_handle:\t" << init_center_of_rotation_T_door_handle.translation().transpose());
            
            double time = 0;
            double T = req.execution_time;

            ros::Rate loop_rate(std::round(1.0/req.sampling_time));


            geometry_msgs::PoseStamped w_T_desired_pose_msg;

            while (ros::ok() && (time <= T ))
            {
                time += req.sampling_time;
                double t_interp = 10 * pow(time/T ,3) - 15 * pow(time/T ,4) + 6 * pow(time/T,5);

                // Compute the new desired angle
                double desired_angle = t_interp * req.final_desired_angle;
                ROS_INFO_STREAM_THROTTLE(1, "Desired angle: " << desired_angle << "/" << req.final_desired_angle);

                // Compute desired pose
                Eigen::Quaterniond center_of_rotation_T_desired_angle_quat = 
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(desired_angle, Eigen::Vector3d::UnitZ());
                Eigen::Affine3d center_of_rotation_T_desired_angle_pose;
                center_of_rotation_T_desired_angle_pose = Eigen::Affine3d::Identity();
                center_of_rotation_T_desired_angle_pose.linear() = center_of_rotation_T_desired_angle_quat.matrix();

                Eigen::Affine3d w_T_desired_pose = w_T_center_of_rotation * center_of_rotation_T_desired_angle_pose * init_center_of_rotation_T_door_handle;


                tf::poseEigenToMsg(w_T_desired_pose, w_T_desired_pose_msg.pose);

                // Trying to keep orientation fixed to avoid joint limits
                w_T_desired_pose_msg.pose.orientation = door_handle_pose.orientation;

                w_T_desired_pose_msg.header.stamp = ros::Time::now();
                w_T_desired_pose_msg.header.frame_id=req.robot_id+"_link0";

                desired_pose_pub_.publish(w_T_desired_pose_msg);

                loop_rate.sleep();
            }

            if (!gripper_->open(default_closing_gripper_speed_)) return false;


            // Move a bit above the last commanded pose
            w_T_desired_pose_msg.pose.position.z += 0.1;
            waypoints.push_back(w_T_desired_pose_msg.pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            ROS_INFO("Door open.");

            res.success = true;
            res.message = "";
            return true;
        }
}; // OpenDoorFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "open_door_fsm");
    ros::NodeHandle nh("~");

    OpenDoorFSM fsm(nh);

    ros::spin();

    return 0;
}
