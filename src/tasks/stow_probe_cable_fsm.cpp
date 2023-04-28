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

            ROS_ERROR_STREAM("Tmp sleep to wait for controllers activation (2s). To be removed!");
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

            // Probe Holder: move to up pose
            geometry_msgs::Pose ph_T_up;
            ph_T_up.position.x = 0.527;
            ph_T_up.position.y = 0.021;
            ph_T_up.position.z = 0.725;
            ph_T_up.orientation.x = 0.707;
            ph_T_up.orientation.y = -0.000;
            ph_T_up.orientation.z = 0.707;
            ph_T_up.orientation.w = 0.000;

            // Probe Holder: move to up pose
            ROS_INFO("[Probe Holder] Moving to up pose.");
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(ph_T_up, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach up pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }



            

            /****************************/
            // TO-DO probe holder change task frame and go down slowly to allow stowing with other robot
            /****************************/

            // Probe Holder: change task frame

            // Probe Holder: move down to allow other robot stowing








            // Cable Stower: preclose the gripper with width 0.01
            // if (!cable_stower_robot_gripper_->setWidth(default_closing_gripper_speed_, 0.01)) return false;
            if (!cable_stower_robot_gripper_->setWidth(default_closing_gripper_speed_, 0.01)) ROS_WARN("Gripper: set width failed...");

            // Cable stower: compute grasp and pregrasp cable pose
            geometry_msgs::Pose cs_T_ending_connector_hole_msg, cs_T_grasp_cable_msg, cs_T_pregrasp_cable_msg;
            cs_T_ending_connector_hole_msg = req.cable_stower_robot_to_ending_connector_hole.pose;

            Eigen::Affine3d cs_T_ending_connector_hole, ending_connector_hole_T_grasp_cable;
            ending_connector_hole_T_grasp_cable = Eigen::Affine3d::Identity();
            // ending_connector_hole_T_grasp_cable.translate(Eigen::Vector3d(-0.045,0.0,-0.004));
            ending_connector_hole_T_grasp_cable.translate(Eigen::Vector3d(-0.053,0.0,-0.005));
            ending_connector_hole_T_grasp_cable.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0,0,1)));

            tf::poseMsgToEigen(cs_T_ending_connector_hole_msg, cs_T_ending_connector_hole);
            Eigen::Affine3d cs_T_grasp_cable = cs_T_ending_connector_hole * ending_connector_hole_T_grasp_cable;
            tf::poseEigenToMsg(cs_T_grasp_cable, cs_T_grasp_cable_msg);

            // Set pregrasp just above the grasp pose
            cs_T_pregrasp_cable_msg = cs_T_grasp_cable_msg;
            cs_T_pregrasp_cable_msg.position.z += 0.1;

            // Cable Stower: move to pregrasp cable
            execution_time = 5.0;
            ROS_INFO("[Cable Stower] Moving to pregrasp cable pose");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_pregrasp_cable_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to grasp cable pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to grasp cable
            ROS_INFO("[Cable Stower] Moving to grasp cable pose");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_grasp_cable_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to grasp cable pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }
            
            // Cable Stower: grasp the cable
            // if (!cable_stower_robot_gripper_->close(default_closing_gripper_speed_)) return false;
            if (!cable_stower_robot_gripper_->close(default_closing_gripper_speed_)) ROS_WARN("Gripper: close failed...");


            // Cable Stower: move to left 1 prepare (step 1)
            geometry_msgs::Pose cs_T_left_1_prepare_msg;
            Eigen::Affine3d cs_T_left_1_prepare, ending_connector_hole_T_left_1_prepare;
            ending_connector_hole_T_left_1_prepare = ending_connector_hole_T_grasp_cable;

            ending_connector_hole_T_left_1_prepare.translate(Eigen::Vector3d(0.10,0.0,0.0));

            cs_T_left_1_prepare = cs_T_ending_connector_hole * ending_connector_hole_T_left_1_prepare;
            tf::poseEigenToMsg(cs_T_left_1_prepare, cs_T_left_1_prepare_msg);

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to left 1 prepare pose (step 1).");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_1_prepare_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to left 1 prepare pose (step 1).";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to left 1 prepare (step 2 - only change orientation)
            // Set fixed rotation in world frame to avoid joint limits (check if it works in other board configurations)
            geometry_msgs::Pose cs_initial_pose;
            cs_initial_pose.position.x = 0.113;
            cs_initial_pose.position.y = -0.284;
            cs_initial_pose.position.z = 0.488;
            cs_initial_pose.orientation.x = 1.000;
            cs_initial_pose.orientation.y = 0.000;
            cs_initial_pose.orientation.z = 0.000;
            cs_initial_pose.orientation.w = 0.000;

            cs_T_left_1_prepare_msg.orientation = cs_initial_pose.orientation;

            execution_time = 3.0;
            ROS_INFO("[Cable Stower] Moving to left 1 prepare pose (step 2).");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_1_prepare_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to left 1 prepare pose (step 2).";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to left 1 stowing
            geometry_msgs::Pose cs_T_left_1_stowing_msg;
            Eigen::Affine3d base_link_T_left_1_stowing, cs_T_base_link;

            base_link_T_left_1_stowing = Eigen::Affine3d::Identity();
            // base_link_T_left_1_stowing.translate(Eigen::Vector3d(-0.151,0.094,0.082));
            base_link_T_left_1_stowing.translate(Eigen::Vector3d(-0.151,0.094,0.090)); // added at 16.08

            tf::poseMsgToEigen(req.cable_stower_robot_to_base_link.pose, cs_T_base_link);
            Eigen::Affine3d cs_T_left_1_stowing = cs_T_base_link * base_link_T_left_1_stowing;
            tf::poseEigenToMsg(cs_T_left_1_stowing, cs_T_left_1_stowing_msg);

            // Fixed orientation
            cs_T_left_1_stowing_msg.orientation = cs_initial_pose.orientation;

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to left 1 stowing pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_1_stowing_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to left 1 stowing pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to left 1 stowing down
            geometry_msgs::Pose cs_T_left_1_stowing_down_msg;
            Eigen::Affine3d base_link_T_left_1_stowing_down;

            base_link_T_left_1_stowing_down = Eigen::Affine3d::Identity();
            base_link_T_left_1_stowing_down.translate(Eigen::Vector3d(-0.150,0.067,0.034));

            Eigen::Affine3d cs_T_left_1_stowing_down = cs_T_base_link * base_link_T_left_1_stowing_down;
            tf::poseEigenToMsg(cs_T_left_1_stowing_down, cs_T_left_1_stowing_down_msg);

            // Fixed orientation
            cs_T_left_1_stowing_down_msg.orientation = cs_initial_pose.orientation;

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to left 1 stowing down pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_1_stowing_down_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to left 1 stowing down pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move left to right 1
            geometry_msgs::Pose cs_T_left_to_right_1_msg;
            Eigen::Affine3d base_link_T_left_to_right_1;

            base_link_T_left_to_right_1 = Eigen::Affine3d::Identity();
            base_link_T_left_to_right_1.translate(Eigen::Vector3d(-0.130,-0.210,0.034));

            Eigen::Affine3d cs_T_left_to_right_1 = cs_T_base_link * base_link_T_left_to_right_1;
            tf::poseEigenToMsg(cs_T_left_to_right_1, cs_T_left_to_right_1_msg);

            // Fixed orientation
            cs_T_left_to_right_1_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_left_to_right_1:\n" << base_link_T_left_to_right_1.matrix());
            ROS_INFO_STREAM("cs_T_left_to_right_1:\n" << cs_T_left_to_right_1.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving left to right 1 pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_to_right_1_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move left to right 1 pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move left to right 1 adj
            geometry_msgs::Pose cs_T_left_to_right_1_adj_msg;
            Eigen::Affine3d base_link_T_left_to_right_1_adj;

            base_link_T_left_to_right_1_adj = Eigen::Affine3d::Identity();
            // base_link_T_left_to_right_1_adj.translate(Eigen::Vector3d(-0.110,-0.210,0.034)); not going inside infrared
            base_link_T_left_to_right_1_adj.translate(Eigen::Vector3d(-0.120,-0.210,0.034));

            Eigen::Affine3d cs_T_left_to_right_1_adj = cs_T_base_link * base_link_T_left_to_right_1_adj;
            tf::poseEigenToMsg(cs_T_left_to_right_1_adj, cs_T_left_to_right_1_adj_msg);

            // Fixed orientation
            cs_T_left_to_right_1_adj_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_left_to_right_1_adj:\n" << base_link_T_left_to_right_1_adj.matrix());
            ROS_INFO_STREAM("cs_T_left_to_right_1_adj:\n" << cs_T_left_to_right_1_adj.matrix());

            execution_time = 2.0;
            ROS_INFO("[Cable Stower] Moving left to right 1 adj pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_to_right_1_adj_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move left to right 1 adj pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to right 1 stowing
            geometry_msgs::Pose cs_T_right_1_stowing_msg;
            Eigen::Affine3d base_link_T_right_1_stowing;

            base_link_T_right_1_stowing = Eigen::Affine3d::Identity();
            base_link_T_right_1_stowing.translate(Eigen::Vector3d(-0.120,-0.210,0.180));


            Eigen::Affine3d cs_T_right_1_stowing = cs_T_base_link * base_link_T_right_1_stowing;
            tf::poseEigenToMsg(cs_T_right_1_stowing, cs_T_right_1_stowing_msg);

            // Fixed orientation
            cs_T_right_1_stowing_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_right_1_stowing:\n" << base_link_T_right_1_stowing.matrix());
            ROS_INFO_STREAM("cs_T_right_1_stowing:\n" << cs_T_right_1_stowing.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to right 1 stowing pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_right_1_stowing_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to right 1 stowing pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move right to center 1 (diagonally upward) - raise positively y and z
            geometry_msgs::Pose cs_T_right_to_center_1_msg;
            Eigen::Affine3d base_link_T_right_to_center_1;

            base_link_T_right_to_center_1 = Eigen::Affine3d::Identity();
            base_link_T_right_to_center_1.translate(Eigen::Vector3d(-0.120,0.000,0.300));

            Eigen::Affine3d cs_T_right_to_center_1 = cs_T_base_link * base_link_T_right_to_center_1;
            tf::poseEigenToMsg(cs_T_right_to_center_1, cs_T_right_to_center_1_msg);

            // Fixed orientation
            cs_T_right_to_center_1_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_right_to_center_1:\n" << base_link_T_right_to_center_1.matrix());
            ROS_INFO_STREAM("cs_T_right_to_center_1:\n" << cs_T_right_to_center_1.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving right to center 1 pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_right_to_center_1_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move right to center 1 pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move center to left 1
            geometry_msgs::Pose cs_T_center_to_left_1_msg;
            Eigen::Affine3d base_link_T_center_to_left_1;

            base_link_T_center_to_left_1 = Eigen::Affine3d::Identity();
            base_link_T_center_to_left_1.translate(Eigen::Vector3d(-0.087,0.131,0.124));

            Eigen::Affine3d cs_T_center_to_left_1 = cs_T_base_link * base_link_T_center_to_left_1;
            tf::poseEigenToMsg(cs_T_center_to_left_1, cs_T_center_to_left_1_msg);

            // Fixed orientation
            cs_T_center_to_left_1_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_center_to_left_1:\n" << base_link_T_center_to_left_1.matrix());
            ROS_INFO_STREAM("cs_T_center_to_left_1:\n" << cs_T_center_to_left_1.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving center to left 1 pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_center_to_left_1_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move center to left 1 pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            /******************************/
            /******* SECOND STOWING *******/
            /******************************/

            // Cable Stower: move to left 2 prepare
            geometry_msgs::Pose cs_T_left_2_prepare_msg;
            Eigen::Affine3d base_link_T_left_2_prepare;

            base_link_T_left_2_prepare = Eigen::Affine3d::Identity();
            base_link_T_left_2_prepare.translate(Eigen::Vector3d(-0.109,0.164,0.090));

            Eigen::Affine3d cs_T_left_2_prepare = cs_T_base_link * base_link_T_left_2_prepare;
            tf::poseEigenToMsg(cs_T_left_2_prepare, cs_T_left_2_prepare_msg);

            // Fixed orientation
            cs_T_left_2_prepare_msg.orientation = cs_initial_pose.orientation;

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to left 2 prepare pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_2_prepare_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to left 2 prepare pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // // Cable Stower: move to left 2 stowing
            // geometry_msgs::Pose cs_T_left_2_stowing_msg;
            // Eigen::Affine3d base_link_T_left_2_stowing;

            // base_link_T_left_2_stowing = Eigen::Affine3d::Identity();
            // // base_link_T_left_2_stowing.translate(Eigen::Vector3d(-0.151,0.094,0.082));
            // base_link_T_left_2_stowing.translate(Eigen::Vector3d(-0.151,0.094,0.090)); // added at 16.08

            // Eigen::Affine3d cs_T_left_2_stowing = cs_T_base_link * base_link_T_left_2_stowing;
            // tf::poseEigenToMsg(cs_T_left_2_stowing, cs_T_left_2_stowing_msg);

            // // Fixed orientation
            // cs_T_left_2_stowing_msg.orientation = cs_initial_pose.orientation;

            // execution_time = 7.0;
            // ROS_INFO("[Cable Stower] Moving to left 2 stowing pose.");
            // if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_2_stowing_msg, execution_time))
            // {
            //     res.success = false;
            //     res.message = req.cable_stower_robot_id+" failed to left 2 stowing pose.";
            //     ROS_ERROR_STREAM(res.message);
            //     return true;
            // }

            // Cable Stower: move to left 2 stowing down
            geometry_msgs::Pose cs_T_left_2_stowing_down_msg;
            Eigen::Affine3d base_link_T_left_2_stowing_down;

            base_link_T_left_2_stowing_down = Eigen::Affine3d::Identity();
            base_link_T_left_2_stowing_down.translate(Eigen::Vector3d(-0.140,0.159,0.041));

            Eigen::Affine3d cs_T_left_2_stowing_down = cs_T_base_link * base_link_T_left_2_stowing_down;
            tf::poseEigenToMsg(cs_T_left_2_stowing_down, cs_T_left_2_stowing_down_msg);

            // Fixed orientation
            cs_T_left_2_stowing_down_msg.orientation = cs_initial_pose.orientation;

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to left 2 stowing down pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_2_stowing_down_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to left 2 stowing down pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move left to right 2
            geometry_msgs::Pose cs_T_left_to_right_2_msg;
            Eigen::Affine3d base_link_T_left_to_right_2;

            base_link_T_left_to_right_2 = Eigen::Affine3d::Identity();
            base_link_T_left_to_right_2.translate(Eigen::Vector3d(-0.130,-0.210,0.034));

            Eigen::Affine3d cs_T_left_to_right_2 = cs_T_base_link * base_link_T_left_to_right_2;
            tf::poseEigenToMsg(cs_T_left_to_right_2, cs_T_left_to_right_2_msg);

            // Fixed orientation
            cs_T_left_to_right_2_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_left_to_right_2:\n" << base_link_T_left_to_right_2.matrix());
            ROS_INFO_STREAM("cs_T_left_to_right_2:\n" << cs_T_left_to_right_2.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving left to right 2 pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_to_right_2_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move left to right 2 pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move left to right 2 adj
            geometry_msgs::Pose cs_T_left_to_right_2_adj_msg;
            Eigen::Affine3d base_link_T_left_to_right_2_adj;

            base_link_T_left_to_right_2_adj = Eigen::Affine3d::Identity();
            base_link_T_left_to_right_2_adj.translate(Eigen::Vector3d(-0.120,-0.210,0.034));

            Eigen::Affine3d cs_T_left_to_right_2_adj = cs_T_base_link * base_link_T_left_to_right_2_adj;
            tf::poseEigenToMsg(cs_T_left_to_right_2_adj, cs_T_left_to_right_2_adj_msg);

            // Fixed orientation
            cs_T_left_to_right_2_adj_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_left_to_right_2_adj:\n" << base_link_T_left_to_right_2_adj.matrix());
            ROS_INFO_STREAM("cs_T_left_to_right_2_adj:\n" << cs_T_left_to_right_2_adj.matrix());

            execution_time = 2.0;
            ROS_INFO("[Cable Stower] Moving left to right 2 adj pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_left_to_right_2_adj_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move left to right 2 adj pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: move to right 2 stowing
            geometry_msgs::Pose cs_T_right_2_stowing_msg;
            Eigen::Affine3d base_link_T_right_2_stowing;

            base_link_T_right_2_stowing = Eigen::Affine3d::Identity();
            base_link_T_right_2_stowing.translate(Eigen::Vector3d(-0.120,-0.210,0.180));


            Eigen::Affine3d cs_T_right_2_stowing = cs_T_base_link * base_link_T_right_2_stowing;
            tf::poseEigenToMsg(cs_T_right_2_stowing, cs_T_right_2_stowing_msg);

            // Fixed orientation
            cs_T_right_2_stowing_msg.orientation = cs_initial_pose.orientation;

            ROS_INFO_STREAM("base_link_T_right_2_stowing:\n" << base_link_T_right_2_stowing.matrix());
            ROS_INFO_STREAM("cs_T_right_2_stowing:\n" << cs_T_right_2_stowing.matrix());

            execution_time = 7.0;
            ROS_INFO("[Cable Stower] Moving to right 2 stowing pose.");
            if(!cable_stower_robot_traj_helper_->moveToTargetPoseAndWait(cs_T_right_2_stowing_msg, execution_time))
            {
                res.success = false;
                res.message = req.cable_stower_robot_id+" failed to move to right 2 stowing pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Cable Stower: open the gripper to release the cable
            // if (!cable_stower_robot_gripper_->open(default_closing_gripper_speed_)) return false;
            // if (!cable_stower_robot_gripper_->open(default_closing_gripper_speed_)) ROS_WARN("Gripper: open failed...");

            res.success = true;
            res.message = "";
            return true;










            /******************************************/
            /****** Probe Holder: PUT PROBE BACK ******/
            /******************************************/

            // Store probe handle pose and compute insertion and approach poses
            geometry_msgs::Pose approach_probe_insertion_pose, probe_insertion_pose, probe_handle_pose;
            probe_handle_pose = req.probe_handle_pose.pose;

            Eigen::Affine3d ph_T_probe_handle;
            tf::poseMsgToEigen(probe_handle_pose, ph_T_probe_handle);

            Eigen::Affine3d probe_handle_T_probe_insertion;
            probe_handle_T_probe_insertion = Eigen::Affine3d::Identity();
            probe_handle_T_probe_insertion.translate(Eigen::Vector3d(0.0,0.0,-0.03));

            Eigen::Affine3d probe_insertion_T_approach_probe_insertion;
            probe_insertion_T_approach_probe_insertion = Eigen::Affine3d::Identity();
            probe_insertion_T_approach_probe_insertion.translate(Eigen::Vector3d(0.10,0.0,0.0));


            Eigen::Affine3d ph_T_probe_insertion = ph_T_probe_handle * probe_handle_T_probe_insertion;

            Eigen::Affine3d ph_T_approach_probe_insertion = ph_T_probe_insertion * probe_insertion_T_approach_probe_insertion;

            // probe_handle_insertion_pose = probe_handle_pose;

            // approach_probe_handle_pose = probe_handle_pose;
            // above_approach_probe_handle_pose

            tf::poseEigenToMsg(ph_T_approach_probe_insertion, approach_probe_insertion_pose);
            tf::poseEigenToMsg(ph_T_probe_insertion, probe_insertion_pose);

            // Move to approach probe insertion pose
            execution_time = 7.0;
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(approach_probe_insertion_pose, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach the approach probe insertion pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move to probe insertion pose
            execution_time = 4.0;
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(probe_insertion_pose, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach the probe insertion pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

            // Move to probe handle pose
            execution_time = 7.0;
            if(!probe_holder_robot_traj_helper_->moveToTargetPoseAndWait(probe_handle_pose, execution_time))
            {
                res.success = false;
                res.message = req.probe_holder_robot_id+" failed to reach the probe handle pose.";
                ROS_ERROR_STREAM(res.message);
                return true;
            }

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

            // Probe Holder: open the gripper to release the probe
            // if (!probe_holder_robot_gripper_->open(default_closing_gripper_speed_)) return false;
            // if (!probe_holder_robot_gripper_->open(default_closing_gripper_speed_)) ROS_WARN("Gripper: open failed...");

            // Probe Holder: task frame back to original one
            hrii_robot_msgs::SetPose original_EE_T_task_frame_srv;
            original_EE_T_task_frame_srv.request.pose_stamped.pose.orientation.w = 1.0; 

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
