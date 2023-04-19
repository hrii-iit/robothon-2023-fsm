#include <ros/ros.h>
#include "hrii_robothon_msgs/MovePlug.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

class MovePlugFSM
{
    public:
        MovePlugFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10)
        {
            activation_server_ = nh_.advertiseService("activate", &MovePlugFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
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

            // USE LATER
            // Close gripper
            if (!gripper_->close(default_closing_gripper_speed_)) return false;

            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 5.0;

            ROS_INFO("MOVE PLUG FSM STARTED!");

            // Move to an approach pose w.r.t. the world frame
            geometry_msgs::Pose approach_pose = req.plug_pose.pose;
            approach_pose.position.z += 0.02;
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            waypoints.push_back(req.plug_pose.pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the move plug pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            waypoints.erase(waypoints.begin());
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to return to the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Gripper opening
            if (!gripper_->open(default_closing_gripper_speed_)) return false;

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
