#include <ros/ros.h>
#include "hrii_robothon_msgs/MoveSlider.h"
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
        

        bool activationCallback(hrii_robothon_msgs::MoveSlider::Request& req,
                                hrii_robothon_msgs::MoveSlider::Response& res)
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

            if(req.times == 1)
            {
                ROS_INFO("MOVE SLIDER FSM STARTED! Approaching for the first time the slider");
                // Move to an approach pose upon the slider (w.r.t. the world frame)
                geometry_msgs::Pose approach_pose = req.slider_pose.pose;
                approach_pose.position.z += 0.02;
                waypoints.push_back(approach_pose);

                if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
                {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the first approaching slider pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
                }

                // Move to the slider pose (w.r.t. the world frame)
                waypoints.push_back(req.slider_pose.pose);

                if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
                {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the slider.";
                ROS_ERROR_STREAM(res.message);
                return true;
                }

                res.message = " Slider grasped";
                ROS_INFO(" Slider grasped");
                waypoints.erase(waypoints.begin());
            }
            
            // Move the slider to the reference point of the screen
            
            waypoints.push_back(req.reference_pose.pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to move the slider to the desired pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            ROS_INFO_STREAM(" Succeded in move the robot in the slider pose: " << req.times);

            //Move back to the approach pose upon the slider only if it is the second movement

            if (req.times ==2)
            {
                // //waypoints.erase(waypoints.begin());
                // waypoints.push_back(approach_pose);

                // if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
                // {
                //     res.success = false;
                //     res.message = req.robot_id+" failed to return to the approach pose.";
                //     ROS_ERROR_STREAM(res.message);
                //     return true;
                // }

                ROS_INFO(" Succeded in move the robot back to approach initial pose");
            }
            
            ros::Duration(5).sleep();
            res.success = true;
            res.message = "";
            return true;
        }
}; // MoveSliderFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_slider_fsm");
    ros::NodeHandle nh("~");

    MoveSliderFSM fsm(nh);

    ros::spin();

    return 0;
}
