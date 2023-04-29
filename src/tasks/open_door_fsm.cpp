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
            default_grasping_gripper_force_(10.0),
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

            // Initialize gripper and open it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            if (!gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 3.0;

            // Store door handle pose and center of rotation pose
            geometry_msgs::Pose door_handle_pose, approach_door_handle_pose, center_of_rotation_pose;
            door_handle_pose = req.door_handle_pose.pose;
            center_of_rotation_pose = req.center_of_rotation_pose.pose;

            // Move to an approach pose
            approach_door_handle_pose = door_handle_pose;
            approach_door_handle_pose.position.z += 0.1;
            waypoints.push_back(approach_door_handle_pose);

            ROS_INFO("Moving to approach pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move to door handle pose
            geometry_msgs::Pose temp_pose;
            temp_pose = door_handle_pose;
            temp_pose.position.z -= 0.002;
            waypoints.push_back(temp_pose);
            execution_time = 1.0;

            ROS_INFO("Moving to door handle pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the door handle pose.";
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
            ROS_INFO_STREAM("w_T_center_of_rotation orient\t" << center_of_rotation_pose.orientation);
            
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

                // Compute desired position
                Eigen::Quaterniond center_of_rotation_T_desired_angle_quat = 
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(desired_angle, Eigen::Vector3d::UnitZ());
                Eigen::Affine3d center_of_rotation_T_desired_angle_pose;
                center_of_rotation_T_desired_angle_pose = Eigen::Affine3d::Identity();
                center_of_rotation_T_desired_angle_pose.linear() = center_of_rotation_T_desired_angle_quat.matrix();

                Eigen::Affine3d w_T_desired_pose = w_T_center_of_rotation * center_of_rotation_T_desired_angle_pose * init_center_of_rotation_T_door_handle;

                tf::poseEigenToMsg(w_T_desired_pose, w_T_desired_pose_msg.pose);

                // Compute desired orientation
                Eigen::Quaterniond center_of_rotation_T_desired_angle_quat_orient = 
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())*
                    Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())*
                    Eigen::AngleAxisd(desired_angle*4/9, Eigen::Vector3d::UnitZ());
                Eigen::Affine3d center_of_rotation_T_desired_angle_pose_orient;
                center_of_rotation_T_desired_angle_pose_orient = Eigen::Affine3d::Identity();
                center_of_rotation_T_desired_angle_pose_orient.linear() = center_of_rotation_T_desired_angle_quat_orient.matrix();

                Eigen::Affine3d w_T_desired_orientation = w_T_center_of_rotation * center_of_rotation_T_desired_angle_pose_orient * init_center_of_rotation_T_door_handle;
                geometry_msgs::PoseStamped w_T_desired_orientation_pose;
                tf::poseEigenToMsg(w_T_desired_orientation, w_T_desired_orientation_pose.pose);
                w_T_desired_pose_msg.pose.orientation = w_T_desired_orientation_pose.pose.orientation;

                // Keeping orientation fixed to avoid joint limits
                // w_T_desired_pose_msg.pose.orientation = door_handle_pose.orientation;

                w_T_desired_pose_msg.header.stamp = ros::Time::now();
                w_T_desired_pose_msg.header.frame_id=req.robot_id+"_link0";

                desired_pose_pub_.publish(w_T_desired_pose_msg);

                loop_rate.sleep();
            }

            // Delete previous point and add last commanded pose
            waypoints.erase(waypoints.begin());
            

            // Continue straight motion
            // Eigen::Quaterniond quat(slider_pose.orientation.w, slider_pose.orientation.x, slider_pose.orientation.y, slider_pose.orientation.z);
            // Eigen::Matrix3d displacement_transformation_rot_matrix = quat.toRotationMatrix();
            ROS_INFO_STREAM("Rotation matrix between franka_left_link0 and board slider: " << w_T_center_of_rotation.linear());

            Eigen::Vector3d displacement_vector(0.013, 0, 0);
            //ROS_INFO_STREAM("Displacement along slider y-axis in Vector3:" << displacement_vector);

            // Displacement vector in robot_base RF
            displacement_vector =  w_T_center_of_rotation.linear() * displacement_vector;
            ROS_INFO_STREAM("First displacement respect robot frame: " << displacement_vector);

            // Pose of the reference point where we have to move the slider
            geometry_msgs::Pose reference_pose;
            reference_pose.position = w_T_desired_pose_msg.pose.position; //we assign to the first reference point the same initial pose of the slider
            reference_pose.position.x += displacement_vector(0);      //then we add the computed displacement along each axes wrt robot RF
            reference_pose.position.y += displacement_vector(1);
            reference_pose.position.z += displacement_vector(2);          
            reference_pose.orientation = w_T_desired_pose_msg.pose.orientation;     //we assing the same rotation of the slider
            
            waypoints.push_back(w_T_desired_pose_msg.pose);
            waypoints.push_back(reference_pose);

            ROS_INFO("Continue motion straight.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the continue motion pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Opening gripper
            ROS_WARN_STREAM("This command (gripper opening) fails many times..");
            if (!gripper_->open(default_closing_gripper_speed_)) ROS_WARN("Gripper: open failed...");

            // Move a bit above the last commanded pose
            w_T_desired_pose_msg.pose.position.z += 0.04;
            waypoints.push_back(w_T_desired_pose_msg.pose);

            ROS_INFO("Moving above last commanded pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Compute and move to opened door pushing start
            Eigen::Affine3d door_handle_T_door_pushing_start;
            door_handle_T_door_pushing_start = Eigen::Affine3d::Identity();
            door_handle_T_door_pushing_start.translation()[0] = 0.02; // 0.05
            door_handle_T_door_pushing_start.translation()[2] = -0.095;

            Eigen::Affine3d w_T_door_pushing_start = w_T_door_handle * door_handle_T_door_pushing_start;

            geometry_msgs::PoseStamped w_T_door_pushing_start_msg;
            tf::poseEigenToMsg(w_T_door_pushing_start, w_T_door_pushing_start_msg.pose);
            
            // Fixing orientation as door handle pose one
            w_T_desired_pose_msg.pose.orientation = door_handle_pose.orientation;

            ROS_INFO_STREAM("w_T_door_pushing_start transl:\t" << w_T_door_pushing_start.translation().transpose());
            // ROS_INFO_STREAM("w_T_door_pushing_start orient (x di 180):\t" << w_T_door_pushing_start_msg.pose.orientation);//.coeffs().transpose());

            waypoints.push_back(w_T_door_pushing_start_msg.pose);

            ROS_INFO("Moving to door pushing start (above) pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach door pushing start (above) pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Closing the gripper
            if (!gripper_->close(default_closing_gripper_speed_)) return false;

            // Move down a bit w.r.t the world frame
            w_T_door_pushing_start.translation()[2] += -0.03;

            tf::poseEigenToMsg(w_T_door_pushing_start, w_T_door_pushing_start_msg.pose);

            // Fixing orientation as door handle pose one
            w_T_desired_pose_msg.pose.orientation = door_handle_pose.orientation;

            ROS_INFO_STREAM("w_T_door_pushing_start (moved down):\t" << w_T_door_pushing_start.translation().transpose());

            waypoints.push_back(w_T_door_pushing_start_msg.pose);

            ROS_INFO("Moving to door pushing start pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach door pushing start pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Compute and move to opened door pushing end
            Eigen::Affine3d door_pushing_start_T_door_pushing_end;
            door_pushing_start_T_door_pushing_end = Eigen::Affine3d::Identity();
            door_pushing_start_T_door_pushing_end.translation()[0] = 0.09;

            Eigen::Affine3d w_T_door_pushing_end = w_T_door_pushing_start * door_pushing_start_T_door_pushing_end;

            geometry_msgs::PoseStamped w_T_door_pushing_end_msg;
            tf::poseEigenToMsg(w_T_door_pushing_end, w_T_door_pushing_end_msg.pose);

            ROS_INFO_STREAM("w_T_door_pushing_end:\t" << w_T_door_pushing_end.translation().transpose());

            waypoints.push_back(w_T_door_pushing_end_msg.pose);

            ROS_INFO("Moving to opened door pushing pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach opened door pushing pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

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
