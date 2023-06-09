#include <ros/ros.h>
#include "hrii_robothon_msgs/MovePlug.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"
#include "hrii_task_board_fsm/utils/ControllerUtils.h"

class MovePlugFSM
{
    public:
        MovePlugFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10),
            default_closing_gripper_force_(10),
            desired_contact_force_(20),
            desired_contact_force_timeout_(2.0)
        {
            activation_server_ = nh_.advertiseService("activate", &MovePlugFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_closing_gripper_force_;
        double desired_contact_force_;
        double desired_contact_force_timeout_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_robothon_msgs::MovePlug::Request& req,
                                hrii_robothon_msgs::MovePlug::Response& res)
        {
            ROS_INFO_STREAM("Activate move plug interface for robot: " << req.robot_id);

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize gripper
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            ROS_INFO("Gripper client initialized.");

            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 3.0;

            ROS_INFO("MOVE PLUG FSM STARTED!");

            // Store starting and ending (approach) poses
            geometry_msgs::Pose starting_pose, starting_approach_pose;
            starting_pose = req.starting_plug_pose.pose;
            starting_approach_pose = starting_pose;
            starting_approach_pose.position.z += 0.02;

            geometry_msgs::Pose ending_pose, ending_approach_pose;
            ending_pose = req.ending_plug_pose.pose;


            // Trying to keep same orientation
            ending_pose.orientation = starting_pose.orientation;


            ending_approach_pose = ending_pose;
            ending_approach_pose.position.z += 0.02;



            // Gripper opening
            if (!gripper_->setWidth(default_closing_gripper_speed_,0.03)) return false;


            // Move the robot to the starting hole connector approaching pose
            waypoints.push_back(starting_approach_pose);
            
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the starting approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }


            // Move to starting pose
            waypoints.push_back(starting_pose);
            execution_time = 1.0;
            ROS_INFO("Moving to starting pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the starting move plug pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Closing the gripper
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) return false;

            // Move the robot back to the starting hole connector approaching pose
            starting_approach_pose.position.z+=0.008;
            waypoints.push_back(starting_approach_pose);
            execution_time = 1.0;

            ROS_INFO("Moving to starting approach pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the starting approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Move the robot above the ending hole connector approaching pose
            ending_approach_pose.position.z+=0.008;
            waypoints.push_back(ending_approach_pose);
            execution_time = 1.0;

            ROS_INFO("Moving to ending approach pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the ending approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Move the robot to the ending hole connector approaching pose  (step 2)
            ending_approach_pose.position.z-=0.007;
            waypoints.push_back(ending_approach_pose);
            execution_time = 1.0;

            ROS_INFO("Moving to ending approach pose (step 2).");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the ending approach pose (step 2).";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Switch to task force in Z-axis
            geometry_msgs::WrenchStamped desired_wrench;
            desired_wrench.header.stamp = ros::Time::now();
            desired_wrench.wrench.force.z = desired_contact_force_;

            if (!applyContactForce(nh_, req.robot_id,
                            "cart_hybrid_motion_force_controller",
                            hrii_robot_msgs::TaskSelection::Request::Z_LIN,
                            desired_wrench,
                            -1.0,
                            desired_contact_force_timeout_))
            {
                ROS_ERROR_STREAM("Contact force application failed.");
                return false;
            }
            waypoints.erase(waypoints.begin());


            ROS_WARN("HERE PERFORM ROTATION: NEEDED FOR BETTER STOWING CABLE!");
            // Move to real ending pose (with rotation)
            // ending_pose = req.ending_plug_pose.pose;
            // waypoints.push_back(ending_pose);

            // ROS_INFO("Moving to real ending pose (with rotation). Needed for better stowing cable later...");
            // if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true)) // true, only one point
            // {
            //     res.success = false;
            //     res.message = req.robot_id+" failed to reach real ending pose (with rotation).";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // Gripper opening
            if (!gripper_->setWidth(default_closing_gripper_speed_,0.03)) return false;

            // Move the robot up
            ending_pose.position.z+=0.050;
            waypoints.push_back(ending_pose);
            execution_time = 1.0;

            ROS_INFO("Moving to up pose.");
            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true)) // Put to false if rotation in prev step enabled
            // if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, false))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the up pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            res.success = true;
            res.message = "";
            return true;
        }
}; // MovePlugFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_plug_fsm");
    ros::NodeHandle nh("~");

    MovePlugFSM fsm(nh);

    ros::spin();

    return 0;
}
