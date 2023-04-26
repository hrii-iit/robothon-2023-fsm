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
        GripperInterfaceClientHelper::Ptr probe_holder_robot_gripper_, cable_stower_robot_gripper_;
        double default_closing_gripper_speed_;
        double default_grasping_gripper_force_;

        HRII::TrajectoryHelper::Ptr probe_holder_robot_traj_helper_, cable_stower_robot_traj_helper_;

        ros::ServiceServer activation_server_;

        ros::ServiceClient controller_set_EE_T_task_frame_client_, cs_controller_set_EE_T_task_frame_client_;
        std::string controller_set_EE_T_task_frame_service_name_, cs_controller_set_EE_T_task_frame_service_name_;

        ros::Publisher desired_pose_pub_;
        std::string controller_desired_pose_topic_name_;

        bool activationCallback(hrii_robothon_msgs::StowProbeCable::Request& req,
                                hrii_robothon_msgs::StowProbeCable::Response& res)
        {

            ROS_INFO_STREAM("Activate probe cable stowing. Probe holder: " << req.probe_holder_robot_id << " - Cable stower: " << req.cable_stower_robot_id);

            ROS_WARN_STREAM("Tmp sleep to wait for controllers activation (2s). To be removed!");
            ros::Duration(2.0).sleep();

            // Trajectory helpers declaration and initialization
            probe_holder_robot_traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.probe_holder_robot_id+"/trajectory_handler");
            if (!probe_holder_robot_traj_helper_->init()) return false;
            probe_holder_robot_traj_helper_->setTrackingPositionTolerance(0.25);
            ROS_INFO("Trajectory handler client initialized (Probe holder robot).");

            cable_stower_robot_traj_helper_ = std::make_shared<HRII::TrajectoryHelper>("/"+req.cable_stower_robot_id+"/trajectory_handler");
            if (!cable_stower_robot_traj_helper_->init()) return false;
            cable_stower_robot_traj_helper_->setTrackingPositionTolerance(0.25);
            ROS_INFO("Trajectory handler client initialized (Cable stower robot).");

            // Initialize grippers
            probe_holder_robot_gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.probe_holder_robot_id+"/gripper");
            if (!probe_holder_robot_gripper_->init()) return false;
            // if (!probe_holder_robot_gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized (Probe holder robot).");

            cable_stower_robot_gripper_ = std::make_shared<GripperInterfaceClientHelper>("/"+req.cable_stower_robot_id+"/gripper");
            if (!cable_stower_robot_gripper_->init()) return false;
            // if (!cable_stower_robot_gripper_->open(default_closing_gripper_speed_)) return false;
            ROS_INFO("Gripper client initialized (Cable stower robot).");
            
            double execution_time = 10.0;

            // Robots rendez-vous in world frame using always fixed poses
            geometry_msgs::Pose ph_T_rendesvouz, cs_T_rendesvouz, cs_T_pre_rendesvouz, cs_T_post_rendesvouz;
            ph_T_rendesvouz.position.x = 0.527;
            ph_T_rendesvouz.position.y = 0.021;
            ph_T_rendesvouz.position.z = 0.725;
            ph_T_rendesvouz.orientation.x = 0.707;
            ph_T_rendesvouz.orientation.y = -0.000;
            ph_T_rendesvouz.orientation.z = 0.707;
            ph_T_rendesvouz.orientation.w = 0.000;

            cs_T_rendesvouz.position.x = 0.421;
            cs_T_rendesvouz.position.y = -0.605;
            cs_T_rendesvouz.position.z = 0.644;
            cs_T_rendesvouz.orientation.x = 0.831;
            cs_T_rendesvouz.orientation.y = -0.012;
            cs_T_rendesvouz.orientation.z = 0.556;
            cs_T_rendesvouz.orientation.w = -0.000;

            cs_T_pre_rendesvouz = cs_T_rendesvouz;
            cs_T_pre_rendesvouz.position.x -= 0.15;

            cs_T_post_rendesvouz = cs_T_rendesvouz;
            cs_T_post_rendesvouz.position.z -= 0.40;
            cs_T_post_rendesvouz.orientation.x = 0.924;
            cs_T_post_rendesvouz.orientation.y = 0.000;
            cs_T_post_rendesvouz.orientation.z = 0.383;
            cs_T_post_rendesvouz.orientation.w = 0.000;

            // Probe Holder: move to rendes-vouz pose
            ROS_INFO("[Probe Holder] Moving to rendes-vouz pose.");
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(ph_T_rendesvouz, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach rendes-vouz pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to pre rendes-vouz pose
            ROS_INFO("[Cable Stower] Moving to rendes-vouz pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_pre_rendesvouz, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach rendes-vouz pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to rendes-vouz pose
            ROS_INFO("[Cable Stower] Moving to rendes-vouz pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_rendesvouz, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach rendes-vouz pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
           
            // Cable Stower: grasp the cable
            // if (!probe_holder_robot_gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            if (!cable_stower_robot_gripper_->close(default_closing_gripper_speed_)) ROS_WARN("Gripper: close failed...");

            // Cable Stower: move to post rendes-vouz pose
            ROS_INFO("[Cable Stower] Moving to rendes-vouz pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_post_rendesvouz, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach rendes-vouz pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            res.success = true;
            res.message = "";
            return true;










            // Change task frame of Probe Holder (ph) robot
            controller_set_EE_T_task_frame_client_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.probe_holder_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.probe_holder_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            controller_set_EE_T_task_frame_client_.waitForExistence();

            // Set the new task frame to probe hole in gripper fingertips
            // The task frame is rotated by 45° on y-axis wrt EE orientation
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

            // Change task frame of Cable Stower (cs) robot
            cs_controller_set_EE_T_task_frame_client_ = nh_.serviceClient<hrii_robot_msgs::SetPose>("/"+req.cable_stower_robot_id+"/"+controller_set_EE_T_task_frame_service_name_);
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName("/"+req.cable_stower_robot_id+"/"+controller_set_EE_T_task_frame_service_name_) << " ROS service...");
            cs_controller_set_EE_T_task_frame_client_.waitForExistence();

            // Set the new task frame to stow probe in gripper fingertips
            // The task frame is rotated by -45° on y-axis wrt EE orientation
            hrii_robot_msgs::SetPose cs_set_EE_T_task_frame_srv;
            cs_set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.y = -0.3826834; 
            cs_set_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 0.9238795; 

            if (!cs_controller_set_EE_T_task_frame_client_.call(cs_set_EE_T_task_frame_srv))
            {
                ROS_ERROR("Error calling cs_set_EE_T_task_frame ROS service.");
                return false;
            }
            else if (!cs_set_EE_T_task_frame_srv.response.success)
            {
                ROS_ERROR("Failure setting cs_EE_T_task_frame. Exiting.");
                return false;
            }


            // /* Move probe holder robot all the way up to keep to cable in tension */
            // // Store ending connector hole pose
            // geometry_msgs::Pose ph_T_ending_connector_hole_msg, ph_T_probe_tension_msg;
            // ph_T_ending_connector_hole_msg = req.probe_holder_robot_to_ending_connector_hole.pose;

            // // Probe holder: design tf from ending connector hole to probe tension
            // Eigen::Affine3d ph_T_ending_connector_hole, ending_connector_hole_T_probe_tension;
            // ending_connector_hole_T_probe_tension = Eigen::Affine3d::Identity();
            // // ending_connector_hole_T_probe_tension.translate(Eigen::Vector3d(-0.10, -0.28,-0.5));
            // // ending_connector_hole_T_probe_tension.translate(Eigen::Vector3d(-0.10, -0.28,-0.78)); this was good but the other robot cannot reach, consider to pull up afterwards
            // ending_connector_hole_T_probe_tension.translate(Eigen::Vector3d(0.10, -0.28,-0.63));

            // // Perform a rotation of 90 + 45 deg around y-axis
            // ending_connector_hole_T_probe_tension.rotate(Eigen::AngleAxisd(M_PI/2 + M_PI/4, Eigen::Vector3d(0,1,0)));
            // ROS_INFO_STREAM("ending_connector_hole_T_probe_tension:\n" << ending_connector_hole_T_probe_tension.matrix());

            // tf::poseMsgToEigen(ph_T_ending_connector_hole_msg, ph_T_ending_connector_hole);
            // Eigen::Affine3d ph_T_probe_tension = ph_T_ending_connector_hole * ending_connector_hole_T_probe_tension;
            // tf::poseEigenToMsg(ph_T_probe_tension, ph_T_probe_tension_msg);

            // // Probe holder: move above the probe handle pose
            // ROS_INFO("[Probe Holder] Moving to probe tension pose");
            // if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(ph_T_probe_tension_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.probe_holder_robot_id+" failed to move to probe tension pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Cable stower: compute grasp and pregrasp cable pose
            // geometry_msgs::Pose cs_T_ending_connector_hole_msg, cs_T_grasp_cable_msg, cs_T_pregrasp_cable_msg;
            // cs_T_ending_connector_hole_msg = req.cable_stower_robot_to_ending_connector_hole.pose;

            // Eigen::Affine3d cs_T_ending_connector_hole, ending_connector_hole_T_grasp_cable, ending_connector_hole_T_pregrasp_cable;
            // ending_connector_hole_T_grasp_cable = Eigen::Affine3d::Identity();
            // ending_connector_hole_T_pregrasp_cable = Eigen::Affine3d::Identity();
            // ending_connector_hole_T_grasp_cable.translate(Eigen::Vector3d(0.017,-0.27,-0.56));
            // ending_connector_hole_T_pregrasp_cable.translate(Eigen::Vector3d(0.20,-0.27,-0.56));

            // // Perform a rotation of 180 deg around z-axis
            // ending_connector_hole_T_grasp_cable.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)));
            // ending_connector_hole_T_pregrasp_cable.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)));
            // // ROS_INFO_STREAM("ending_connector_hole_T_grasp_cable:\n" << ending_connector_hole_T_grasp_cable.matrix());


            // // Perform a rotation of 45 deg around y-axis
            // ending_connector_hole_T_grasp_cable.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,1,0)));
            // ending_connector_hole_T_pregrasp_cable.rotate(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0,1,0)));


            // tf::poseMsgToEigen(cs_T_ending_connector_hole_msg, cs_T_ending_connector_hole);
            // Eigen::Affine3d cs_T_grasp_cable = cs_T_ending_connector_hole * ending_connector_hole_T_grasp_cable;
            // tf::poseEigenToMsg(cs_T_grasp_cable, cs_T_grasp_cable_msg);

            // Eigen::Affine3d cs_T_pregrasp_cable = cs_T_ending_connector_hole * ending_connector_hole_T_pregrasp_cable;
            // tf::poseEigenToMsg(cs_T_pregrasp_cable, cs_T_pregrasp_cable_msg);

            // // Cable Stower: move to pregrasp cable
            // execution_time = 7.0;
            // ROS_INFO("[Cable Stower] Moving to pregrasp cable pose");
            // if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_pregrasp_cable_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.cable_stower_robot_id+" failed to move to grasp cable pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // // Cable Stower: move to grasp cable
            // ROS_INFO("[Cable Stower] Moving to grasp cable pose");
            // if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_grasp_cable_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.cable_stower_robot_id+" failed to move to grasp cable pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }
            // 
            // // Cable Stower: grasp the cable
            // // if (!probe_holder_robot_gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            //if (!cable_stower_robot_gripper_->close(default_closing_gripper_speed_)) ROS_WARN("Gripper: close failed...");

            // // Cable Stower: move down from grasp cable
            // geometry_msgs::Pose cs_T_grasp_cable_down_msg;
            // Eigen::Affine3d cs_T_grasp_cable_down = cs_T_grasp_cable;
            // // Eigen::Affine3d cs_T_grasp_cable_down_test = cs_T_grasp_cable_down;

            // ROS_INFO_STREAM("cs_T_grasp_cable_down before:\n" << cs_T_grasp_cable_down.matrix());
            // cs_T_grasp_cable_down.pretranslate(Eigen::Vector3d(0.0,0.0,-0.30)); // move down 20cm on z-axis
            // ROS_INFO_STREAM("cs_T_grasp_cable_down after:\n" << cs_T_grasp_cable_down.matrix());


            // // ROS_INFO_STREAM("cs_T_grasp_cable_down_test before:\n" << cs_T_grasp_cable_down_test.matrix());
            // // cs_T_grasp_cable_down_test.pretranslate(Eigen::Vector3d(0.0,0.0,0.20)); // move down 20cm on z-axis
            // // ROS_INFO_STREAM("cs_T_grasp_cable_down_test after:\n" << cs_T_grasp_cable_down_test.matrix());

            // // ROS_INFO_STREAM("cs_T_grasp_cable_down:\n" << cs_T_grasp_cable_down.matrix());

            // tf::poseEigenToMsg(cs_T_grasp_cable_down, cs_T_grasp_cable_down_msg);

            // ROS_INFO("[Cable Stower] Moving down from grasp cable pose");
            // if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_grasp_cable_down_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.cable_stower_robot_id+" failed to move down from grasp cable pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // TODO: perform stowing



            // Open the gripper to release the probe
            // if (!probe_holder_robot_gripper_->graspFromOutside(default_closing_gripper_speed_, default_grasping_gripper_force_)) return false;
            // if (!probe_holder_robot_gripper_->open(default_closing_gripper_speed_)) ROS_WARN("Gripper: open failed...");


            // Task frames back to original one
            hrii_robot_msgs::SetPose original_EE_T_task_frame_srv;
            original_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 1.0; 

            // Probe Holder
            if (!controller_set_EE_T_task_frame_client_.call(original_EE_T_task_frame_srv))
            {
                ROS_ERROR("Error calling set_EE_T_task_frame ROS service (Probe Holder).");
                return false;
            }
            else if (!original_EE_T_task_frame_srv.response.success)
            {
                ROS_ERROR("Failure setting EE_T_task_frame (Probe Holder). Exiting.");
                return false;
            }

            // Cable Stower
            // if (!cs_controller_set_EE_T_task_frame_client_.call(original_EE_T_task_frame_srv))
            // {
            //     ROS_ERROR("Error calling set_EE_T_task_frame ROS service (Cable Stower).");
            //     return false;
            // }
            // else if (!original_EE_T_task_frame_srv.response.success)
            // {
            //     ROS_ERROR("Failure setting EE_T_task_frame (Cable Stower). Exiting.");
            //     return false;
            // }

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
