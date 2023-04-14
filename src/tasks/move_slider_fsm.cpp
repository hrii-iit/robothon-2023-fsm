#include <ros/ros.h>
#include "hrii_task_board_fsm/MoveSlider.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

class MoveSliderFSM
{
    public:
        MoveSliderFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_speed_(10)
        {
            activation_server_ = nh_.advertiseService("activate", &MoveSliderFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_task_board_fsm::MoveSlider::Request& req,
                                hrii_task_board_fsm::MoveSlider::Response& res)
        {
            ROS_INFO_STREAM("Activate move slider interface for robot: " << req.robot_id);

            //Wait until the controller_started param is found
            // ROS_WARN("Waiting for controller to start...");
            // do{
            //     ros::Duration(0.5).sleep();
            // }while(!HRII_Utils::getParamSuccess(req.robot_id+"controller_started"));
            // ROS_INFO("Controller started.");

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize gripper and close it
            // gripper_ = std::make_shared<GripperInterfaceClientHelper>(req.robot_id+"trajectory_handler/execute_trajectory");
            // if (!gripper_->init()) return false;
            // if (!gripper_->close(default_closing_gripper_speed_)) return false;
            // ROS_INFO("Gripper client initialized.");
            

            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 5.0;

            ROS_INFO("MOVE SLIDER FSM STARTED!");

            // Move to an approach pose w.r.t. the world frame
            geometry_msgs::Pose approach_pose = req.button_pose.pose;
            approach_pose.position.z += 0.02;
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to reach the approach pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            waypoints.push_back(req.button_pose.pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to move the slider.";
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

            res.success = true;
            res.message = "";
            return true;
        }
}; // MoveSliderFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dummy_fsm");
    ros::NodeHandle nh("~");

    MoveSliderFSM fsm(nh);

    ros::spin();

    return 0;
}
