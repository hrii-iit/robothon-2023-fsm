#include <ros/ros.h>
#include "hrii_robothon_msgs/MoveSlider.h"
#include "hrii_robothon_msgs/DesiredSliderDisplacement.h"
#include "hrii_trajectory_planner/trajectory_helper/TrajectoryHelper.h"
#include "hrii_gri_interface/client_helper/GripperInterfaceClientHelper.h"

class MoveSliderFSM
{
    public:
        MoveSliderFSM(ros::NodeHandle& nh) : 
            nh_(nh),
            default_closing_gripper_force_(0.2),
            default_closing_gripper_speed_(0.02)
        {
            activation_server_ = nh_.advertiseService("activate", &MoveSliderFSM::activationCallback, this);
            ROS_INFO_STREAM(nh_.resolveName("activate") << " ROS service available.");

            slider_displacement_service_name_ = "/robothon/slider_desired_pose";
            slider_displacement_client_ = nh_.serviceClient<hrii_robothon_msgs::DesiredSliderDisplacement>(slider_displacement_service_name_);
        }
    
    private:
        ros::NodeHandle nh_;
        GripperInterfaceClientHelper::Ptr gripper_;
        double default_closing_gripper_speed_;
        double default_closing_gripper_force_;
        HRII::TrajectoryHelper::Ptr traj_helper_;

        std::string slider_displacement_service_name_;
        ros::ServiceClient slider_displacement_client_;

        ros::ServiceServer activation_server_;
        

        bool activationCallback(hrii_robothon_msgs::MoveSlider::Request& req,
                                hrii_robothon_msgs::MoveSlider::Response& res)
        {
            ROS_INFO_STREAM("Activate move slider interface for robot: " << req.robot_id);

            // Trajectory helper declaration and initialization
            traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.robot_id+"/trajectory_handler");
            if (!traj_helper_->init()) return false;
            traj_helper_->setTrackingPositionTolerance(0.2);
            ROS_INFO("Trajectory handler client initialized.");

            // Initialize gripper and open it
            gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.robot_id+"/gripper");
            if (!gripper_->init()) return false;
            double opening_gri_speed = 1.0;
            if (!gripper_->open(opening_gri_speed)) return false;
            ROS_INFO("Gripper client initialized.");
            
            std::vector<geometry_msgs::Pose> waypoints;
            double execution_time = 3.0;

            ROS_INFO("MOVE SLIDER FSM STARTED!");
            geometry_msgs::Pose slider_homing_pose, slider_pose, approach_pose;

            // Move to slider homing pose to avoid joint limits
            slider_homing_pose.position.x = 0.370;
            slider_homing_pose.position.y = -0.162;
            slider_homing_pose.position.z = 0.247;
            slider_homing_pose.orientation.x = 1.000;
            slider_homing_pose.orientation.y = 0.000;
            slider_homing_pose.orientation.z = 0.000;
            slider_homing_pose.orientation.w = 0.000;
            waypoints.push_back(slider_homing_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the first approaching slider pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move to an approach pose upon the slider 
            slider_pose = req.slider_pose.pose;
            approach_pose = slider_pose;
            approach_pose.position.z += 0.02;
            waypoints.push_back(approach_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the first approaching slider pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Move to the slider pose
            slider_pose.position.z -= 0.002;
            waypoints.push_back(slider_pose);
            execution_time = 1.5;

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {
                res.success = false;
                res.message = req.robot_id+" failed to grasp the slider.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

            // Closing the gripper
            // if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) return false;
            if (!gripper_->graspFromOutside(default_closing_gripper_speed_, default_closing_gripper_force_)) ROS_WARN("Gripper: grasp from outside failed...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(slider_displacement_service_name_) << " ROS service...");
            slider_displacement_client_.waitForExistence();

            // Calling desired slider displacement service
            hrii_robothon_msgs::DesiredSliderDisplacement desired_slider_displacement_srv;

            if (!slider_displacement_client_.call(desired_slider_displacement_srv))
            {
                ROS_ERROR("Error calling desired slider displacement service.");
                return false;
            }
            
            // Transform from tf to rotation matrix
            Eigen::Quaternion<double> Q(slider_pose.orientation.w, slider_pose.orientation.x, slider_pose.orientation.y, slider_pose.orientation.z);
            Eigen::Matrix3d displacement_transformation_rot_matrix = Q.toRotationMatrix();
            ROS_INFO_STREAM("Rotation matrix between franka_left_link0 and board slider: " << displacement_transformation_rot_matrix);

            float abs_displacement = 0.0;

            // Loop until the task is not accomplished
            while(!desired_slider_displacement_srv.response.task_accomplished)
            {
                // Move the slider to the desired displacement
                float relative_displacement = desired_slider_displacement_srv.response.displacement;

                abs_displacement+=relative_displacement;

                if(abs_displacement < 0.0){
                    ROS_INFO_STREAM("Request displacement out of scale (" << abs_displacement << ")! Manually set to 0.0.");
                    abs_displacement = 0.0;
                }else if(abs_displacement > 0.025){
                    ROS_INFO_STREAM("Request displacement out of scale (" << abs_displacement << ")! Manually set to 0.025.");
                    abs_displacement = 0.025;
                }
                ROS_INFO_STREAM("Displacement along slider y-axis: " << abs_displacement);

                Eigen::Vector3d displacement_vector(0, -abs_displacement, 0);
                //ROS_INFO_STREAM("Displacement along slider y-axis in Vector3:" << displacement_vector);

                // Displacement vector in robot_base RF
                displacement_vector =  displacement_transformation_rot_matrix * displacement_vector;
                ROS_INFO_STREAM("First displacement respect robot frame: " << displacement_vector);

                // Pose of the reference point where we have to move the slider
                geometry_msgs::Pose reference_pose;
                reference_pose.position = slider_pose.position; //we assign to the first reference point the same initial pose of the slider
                reference_pose.position.x += displacement_vector(0);      //then we add the computed displacement along each axes wrt robot RF
                reference_pose.position.y += displacement_vector(1);
                reference_pose.position.z += displacement_vector(2);          
                reference_pose.orientation = slider_pose.orientation;     //we assing the same rotation of the slider

                // Move the robot to reference pose
                waypoints.push_back(reference_pose);
                execution_time = 4.0;

                if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
                {
                    res.success = false;
                    res.message = req.robot_id+" failed to grasp the slider.";
                    ROS_ERROR_STREAM(res.message);
                    return true;
                }
                waypoints.erase(waypoints.begin());

                // Temporary sleep to wait for vision unit, maybe remove in the future
                ros::Duration(0.5).sleep();

                // Check if task is accomplished or get new displacement
                if (!slider_displacement_client_.call(desired_slider_displacement_srv))
                {
                    ROS_ERROR("Error calling desired slider displacement service.");
                    return false;
                }
            }

            ROS_INFO("Move slide task accomplished, opening gripper.");
            double opening_gripper_speed = 1.0;
            if (!gripper_->open(opening_gripper_speed)) return false;

            // Move to slider homing pose to avoid joint limits
            waypoints.push_back(slider_homing_pose);

            if(!traj_helper_->moveToTargetPoseAndWait(waypoints, execution_time, true))
            {   
                res.success = false;
                res.message = req.robot_id+" failed to reach the first approaching slider pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            waypoints.erase(waypoints.begin());

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
