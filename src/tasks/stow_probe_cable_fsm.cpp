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
            controller_desired_pose_topic_name_("cart_hybrid_motion_force_controller/desired_tool_pose"),
            controller_set_EE_T_task_frame_service_name_("cart_hybrid_motion_force_controller/set_EE_T_task_frame")
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

            /* Move probe holder robot all the way up to keep to cable in tension */
            // Store ending connector hole pose
            geometry_msgs::Pose w_T_ending_connector_hole_msg, w_T_probe_tension_msg;
            w_T_ending_connector_hole_msg = req.probe_holder_robot_to_ending_connector_hole.pose;

            // Design tf from ending connector hole to probe tension
            Eigen::Affine3d w_T_ending_connector_hole, ending_connector_hole_T_probe_tension;
            ending_connector_hole_T_probe_tension = Eigen::Affine3d::Identity();
            ending_connector_hole_T_probe_tension.translate(Eigen::Vector3d(0.0, -0.28,-0.5));

            // Perform a rotation of 90 deg around y axis
            ending_connector_hole_T_probe_tension.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,1,0)));


            ROS_INFO_STREAM("ending_connector_hole_T_probe_tension:\n" << ending_connector_hole_T_probe_tension.matrix());


            // ending_connector_hole_T_probe_tension. ()[2] = -0.50;

            tf::poseMsgToEigen(w_T_ending_connector_hole_msg, w_T_ending_connector_hole);

            Eigen::Affine3d w_T_probe_tension = w_T_ending_connector_hole * ending_connector_hole_T_probe_tension;

            geometry_msgs::PoseStamped w_T_door_pushing_start_msg;
            tf::poseEigenToMsg(w_T_probe_tension, w_T_probe_tension_msg);

            // Move above the probe handle pose
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(w_T_probe_tension_msg, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to move to probe tension pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

           




            // Open the gripper to release the probe
            // if (!probe_holder_robot_gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            // if (!probe_holder_robot_gripper_->open(default_closing_gripper_speed_)) ROS_WARN("Gripper: open failed...");


            // Task frame back to original one
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
