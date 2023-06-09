#include "ros/ros.h"
#include "hrii_robothon_msgs/BoardDetection.h"
#include "hrii_robothon_msgs/Homing.h"
#include "hrii_robothon_msgs/MovePlug.h"
#include "hrii_robothon_msgs/MoveSlider.h"
#include "hrii_robothon_msgs/OpenDoor.h"
#include "hrii_robothon_msgs/PressButton.h"
#include "hrii_robothon_msgs/OpenDoor.h"
#include "hrii_robothon_msgs/ProbeCircuit.h"
#include "hrii_robothon_msgs/StowProbeCable.h"
#include "hrii_robothon_msgs/AssignTask.h"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "hrii_task_board_fsm/utils/ControllerUtils.h"
#include "hrii_task_board_fsm/utils/GeometryMsgs.h"

class MainFSM
{
    public:
        MainFSM() : nh_priv_("~"), tf_listener_{tf_buffer_}, prev_robot_id_("")
        {
            homing_service_name_ = "homing_fsm/activate";
            homing_client_ = nh_.serviceClient<hrii_robothon_msgs::Homing>(homing_service_name_);

            board_detection_activation_service_name_ = "board_detection_fsm/activate";
            board_detection_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::BoardDetection>(board_detection_activation_service_name_);

            press_button_activation_service_name_ = "press_button_fsm/activate";
            press_button_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::PressButton>(press_button_activation_service_name_);

            move_slider_activation_service_name_ = "move_slider_fsm/activate";
            move_slider_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::MoveSlider>(move_slider_activation_service_name_);

            move_plug_activation_service_name_ = "move_plug_fsm/activate";
            move_plug_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::MovePlug>(move_plug_activation_service_name_);

            open_door_activation_service_name_ = "open_door_fsm/activate";
            open_door_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::OpenDoor>(open_door_activation_service_name_);

            probe_circuit_activation_service_name_ = "probe_circuit_fsm/activate";
            probe_circuit_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::ProbeCircuit>(probe_circuit_activation_service_name_);

            stow_probe_cable_activation_service_name_ = "stow_probe_cable_fsm/activate";
            stow_probe_cable_activation_client_ = nh_.serviceClient<hrii_robothon_msgs::StowProbeCable>(stow_probe_cable_activation_service_name_);
    
            assign_task_service_name_ = "task_manager/assign_task";
            assign_task_client_ = nh_.serviceClient<hrii_robothon_msgs::AssignTask>(assign_task_service_name_);

        }

        bool init()
        {
            if(!nh_priv_.getParam("left_robot_id", left_robot_id_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("left_robot_id") <<" ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("right_robot_id", right_robot_id_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("right_robot_id") << " ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("task_order", task_order_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("task_order") << " ROS param found");
                return false;
            }
            for (int cnt = 0; cnt < task_order_.size(); cnt++)
            {
                ROS_INFO_STREAM(cnt << ": " << task_order_[cnt]);
            }
            if(!nh_priv_.getParam("left_robot_launch", left_robot_launch_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("left_robot_launch") << " ROS param found");
                return false;
            }
            if(!nh_priv_.getParam("right_robot_launch", right_robot_launch_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("right_robot_launch") << " ROS param found");
                return false;
            }
            
            if(left_robot_launch_)
            {
                left_robot_controller_manager_status_service_name_ = "/" + left_robot_id_ + "/controller_manager/list_controllers";
                if (!waitForRunningController(nh_, 
                                            left_robot_controller_manager_status_service_name_,
                                            "cart_hybrid_motion_force_controller")) return false;
                ROS_INFO("Left robot controller running.");
            }else
                ROS_INFO("Left robot controller not activated.");

            if(right_robot_launch_)
            {
                right_robot_controller_manager_status_service_name_ = "/" + right_robot_id_ + "/controller_manager/list_controllers";
                if (!waitForRunningController(nh_, 
                                            right_robot_controller_manager_status_service_name_,
                                            "cart_hybrid_motion_force_controller")) return false;
                ROS_INFO("Right robot controller running.");
            }else
                ROS_INFO("Right robot controller not activated.");
            
            return true;
        }

        void spin()
        {
            task_cnt_ = 0;
            state_ = resolveStateString(task_order_[task_cnt_]);
            while (ros::ok() &&
                   task_cnt_ < task_order_.size() &&
                   state_ != MainFSM::States::EXIT &&
                   state_ != MainFSM::States::ERROR)
            {
                switch (state_)
                {
                case MainFSM::States::HOMING:
                {
                    // Move to homing pose and wait for start
                    ROS_INFO("- - - HOMING STATE - - -");
                    if (!homing())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                case MainFSM::States::OTHER_HOMING:
                {
                    // Move to other pose and wait for start
                    ROS_INFO("- - - OTHER HOMING STATE - - -");
                    if (!otherHoming())
                        state_ = MainFSM::States::ERROR;
                    break;
                }
                    
                case MainFSM::States::BOARD_DETECTION:
                {
                    // Board detection
                    ROS_INFO("- - - BOARD DETECTION STATE - - -");
                    if (!boardDetection())
                        state_ = MainFSM::States::ERROR;
                    break;
                }
                    
                case MainFSM::States::PRESS_BLUE_BUTTON:
                {
                    ROS_INFO("- - - PRESS BLUE BUTTON STATE - - -");
                    if (!pressBlueButton())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                case MainFSM::States::MOVE_SLIDER:
                {
                    ROS_INFO("- - - MOVE SLIDER STATE - - -");
                    if (!moveSlider())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                case MainFSM::States::MOVE_PLUG_CABLE:
                {
                    ROS_INFO("- - - MOVE PLUG STATE - - -");
                    if (!movePlug())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                case MainFSM::States::OPEN_DOOR:
                {
                    ROS_INFO("- - - OPEN DOOR STATE - - -");
                    if (!openDoor())
                        state_ = MainFSM::States::ERROR;
                    break;
                }
                    
                case MainFSM::States::PROBE:
                {
                    ROS_INFO("- - - PROBE STATE - - -");
                    if (!probeCircuit())
                        state_ = MainFSM::States::ERROR;
                    break;
                }
                
                case MainFSM::States::STOW_PROBE_CABLE:
                {
                    ROS_INFO("- - - STOW PROBE CABLE STATE - - -");
                    if (!stowProbeCable())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                case MainFSM::States::PRESS_RED_BUTTON:
                {
                    ROS_INFO("- - - PRESS RED BUTTON STATE - - -");
                    if (!pressRedButton())
                        state_ = MainFSM::States::ERROR;
                    break;
                }

                default:
                {
                    state_ = MainFSM::States::ERROR;
                    ROS_INFO("State not implemented. Error.");
                    break;
                }
                }
                if (state_ != MainFSM::States::EXIT && state_ != MainFSM::States::ERROR)
                {
                    task_cnt_++;
                    state_ = resolveStateString(task_order_[task_cnt_]);
                }
            }
            if (state_ == MainFSM::States::EXIT)
            {
                ROS_INFO("Tasks completed. Congratulations!");
            }
            else if (state_ == MainFSM::States::ERROR)
            {
                ROS_ERROR("Main FSM finished with error.");
            }
        }
    
    private:
        // ROS attributes
        ros::NodeHandle nh_, nh_priv_;

        std::vector<std::string> task_order_;
        int task_cnt_;
        std::string prev_robot_id_;
        
        std::string left_robot_controller_manager_status_service_name_;
        ros::ServiceClient left_robot_controller_manager_status_client_;
        
        std::string right_robot_controller_manager_status_service_name_;
        ros::ServiceClient right_robot_controller_manager_status_client_;

        std::string homing_service_name_;
        ros::ServiceClient homing_client_;

        std::string board_detection_activation_service_name_;
        ros::ServiceClient board_detection_activation_client_;

        std::string press_button_activation_service_name_;
        ros::ServiceClient press_button_activation_client_;

        std::string move_slider_activation_service_name_;
        ros::ServiceClient move_slider_activation_client_;

        std::string move_plug_activation_service_name_;
        ros::ServiceClient move_plug_activation_client_;

        std::string open_door_activation_service_name_;
        ros::ServiceClient open_door_activation_client_;

        std::string probe_circuit_activation_service_name_;
        ros::ServiceClient probe_circuit_activation_client_;

        std::string stow_probe_cable_activation_service_name_;
        ros::ServiceClient stow_probe_cable_activation_client_;

        std::string assign_task_service_name_;
        ros::ServiceClient assign_task_client_;
        
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        // Robots attributs
        std::string left_robot_id_, right_robot_id_;
        bool left_robot_launch_, right_robot_launch_;

        // FSM states declaration
        enum class States {HOMING, 
                            OTHER_HOMING, 
                            PRESS_RED_BUTTON, 
                            BOARD_DETECTION,
                            MOVE_SLIDER,
                            MOVE_PLUG_CABLE, 
                            OPEN_DOOR,
                            PROBE,
                            STOW_PROBE_CABLE,
                            PRESS_BLUE_BUTTON,
                            EXIT,
                            ERROR} state_;

        // Homing pose
        geometry_msgs::Pose homing_pose, other_homing_pose;

        bool homing()
        {
            ROS_INFO("Go to home position...");

            // Wait for task services activation
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(homing_service_name_) << " ROS service...");
            homing_client_.waitForExistence();

            // Define homing pose
            homing_pose.position.x = 0.351;
            // homing_pose.position.y = -0.233;
            homing_pose.position.y = 0.040; //new homing to avoid collision
            homing_pose.position.z = 0.441;
            homing_pose.orientation.x = -0.693;
            homing_pose.orientation.y = 0.706;
            homing_pose.orientation.z = -0.104;
            homing_pose.orientation.w = -0.104;

            hrii_robothon_msgs::Homing homing_srv;
            homing_srv.request.robot_id = left_robot_id_;
            homing_srv.request.home_pose.pose = homing_pose;

            if (!homing_client_.call(homing_srv))
            {
                ROS_ERROR("Error calling homing service.");
                return false;
            }
            else if (!homing_srv.response.success)
            {
                ROS_ERROR("Failure going home. Exiting.");
                return false;
            }
            ROS_INFO("Homing succeded.");
            return true;
        }

        bool otherHoming()
        {
            ROS_INFO("Go to other home position...");

            // Wait for task services activation
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(homing_service_name_) << " ROS service...");
            homing_client_.waitForExistence();

            // Define homing pose
            other_homing_pose.position.x = 0.113;
            other_homing_pose.position.y = -0.284;
            other_homing_pose.position.z = 0.488;
            other_homing_pose.orientation.x = 1.000;
            other_homing_pose.orientation.y = 0.000;
            other_homing_pose.orientation.z = 0.000;
            other_homing_pose.orientation.w = 0.000;

            hrii_robothon_msgs::Homing other_homing_srv;
            other_homing_srv.request.robot_id = right_robot_id_;
            other_homing_srv.request.home_pose.pose = other_homing_pose;

            if (!homing_client_.call(other_homing_srv))
            {
                ROS_ERROR("Error calling homing service.");
                return false;
            }
            else if (!other_homing_srv.response.success)
            {
                ROS_ERROR("Failure going home. Exiting.");
                return false;
            }
            ROS_INFO("Other homing succeded.");
            return true;
        }

        bool boardDetection()
        {
            ROS_INFO("Detecting board...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(board_detection_activation_service_name_) << " ROS service...");
            board_detection_activation_client_.waitForExistence();

            hrii_robothon_msgs::BoardDetection board_detection_srv;
            board_detection_srv.request.robot_id = left_robot_id_;
            prev_robot_id_ = left_robot_id_;
            board_detection_srv.request.homing_pose.pose = homing_pose;

            if (!board_detection_activation_client_.call(board_detection_srv))
            {
                ROS_ERROR("Error calling board detection activation service.");
                return false;
            }
            else if (!board_detection_srv.response.success)
            {
                ROS_ERROR("Failure detecting board. Exiting.");
                return false;
            }
            
            ROS_INFO("Board detected.");
            return true;
        }

        bool pressBlueButton()
        {
            ROS_INFO("Pressing the blue button...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(press_button_activation_service_name_) << " ROS service...");
            press_button_activation_client_.waitForExistence();

            hrii_robothon_msgs::PressButton press_button_srv;
            press_button_srv.request.robot_id = left_robot_id_; //UNCOMMENT
            // press_button_srv.request.robot_id = right_robot_id_;
            prev_robot_id_ = left_robot_id_;

            // Real button pose
            geometry_msgs::TransformStamped blueButtonTransform;
            try{
                blueButtonTransform = tf_buffer_.lookupTransform(press_button_srv.request.robot_id+"_link0", "task_board_blue_button_link", ros::Time(0), ros::Duration(3));
                // blueButtonTransform = tf_buffer_.lookupTransform(press_button_srv.request.robot_id+"_link0", "task_board_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << press_button_srv.request.robot_id << "_link0 and task_board_blue_button_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << press_button_srv.request.robot_id <<"_link0 and task_board_blue_button_link NOT found!");
            }
            geometry_msgs::Pose blue_button_pose;
            // blue_button_pose.orientation = blueButtonTransform.transform.rotation;

            // Use fixed orientation to avoid joint limits
            blue_button_pose.orientation.x = -0.707;
            blue_button_pose.orientation.y = 0.707;
            blue_button_pose.orientation.z = 0.000;
            blue_button_pose.orientation.w = 0.000;

            blue_button_pose.position.x = blueButtonTransform.transform.translation.x;
            blue_button_pose.position.y = blueButtonTransform.transform.translation.y;
            // blue_button_pose.position.z = blueButtonTransform.transform.translation.z;
            blue_button_pose.position.z = 0.095; // Set it manually to fix detection potential failure (not to break the board)



            ROS_INFO_STREAM("Blue button pose: " << blue_button_pose.position.x << ", " << blue_button_pose.position.y << ", " << blue_button_pose.position.z);
            ROS_INFO_STREAM("Blue button orientation: " << blue_button_pose.orientation.x << ", " << blue_button_pose.orientation.y << ", " << blue_button_pose.orientation.z << ", " << blue_button_pose.orientation.w);

            press_button_srv.request.button_pose.pose = blue_button_pose;

            if (!press_button_activation_client_.call(press_button_srv))
            {
                ROS_ERROR("Error calling press button activation service.");
                return false;
            }
            else if (!press_button_srv.response.success)
            {
                ROS_ERROR("Failure pressing button. Exiting.");
                return false;
            }
            ROS_INFO("Button pressed.");
            return true;
        }

        bool pressRedButton()
        {
            ROS_INFO("Pressing the red button...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(press_button_activation_service_name_) << " ROS service...");
            press_button_activation_client_.waitForExistence();

            hrii_robothon_msgs::PressButton press_button_srv;
            if (prev_robot_id_.compare(right_robot_id_) == 0)
            {
                if (!otherHoming()) return false;
            } 
            press_button_srv.request.robot_id = left_robot_id_;
            prev_robot_id_ = left_robot_id_;

            // Real button pose
            geometry_msgs::TransformStamped redButtonTransform;
            try{
                redButtonTransform = tf_buffer_.lookupTransform(press_button_srv.request.robot_id+"_link0", "task_board_red_button_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << press_button_srv.request.robot_id << "_link0 and task_board_red_button_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << press_button_srv.request.robot_id <<"_link0 and task_board_red_button_link NOT found!");
            }
            geometry_msgs::Pose red_button_pose;
            // red_button_pose.orientation = redButtonTransform.transform.rotation;

            // Use fixed orientation to avoid joint limits
            red_button_pose.orientation.x = -0.707;
            red_button_pose.orientation.y = 0.707;
            red_button_pose.orientation.z = 0.000;
            red_button_pose.orientation.w = 0.000;

            red_button_pose.position.x = redButtonTransform.transform.translation.x;
            red_button_pose.position.y = redButtonTransform.transform.translation.y;
            red_button_pose.position.z = redButtonTransform.transform.translation.z;

            ROS_INFO_STREAM("Red button pose: " << red_button_pose.position.x << ", " << red_button_pose.position.y << ", " << red_button_pose.position.z);
            // ROS_INFO_STREAM("Red button or r: " << redButtonTransform.transform.rotation.x << ", " << redButtonTransform.transform.rotation.y << ", " << redButtonTransform.transform.rotation.z << ", " << redButtonTransform.transform.rotation.w);
            ROS_INFO_STREAM("Red button orientation: " << red_button_pose.orientation.x << ", " << red_button_pose.orientation.y << ", " << red_button_pose.orientation.z << ", " << red_button_pose.orientation.w);

            press_button_srv.request.button_pose.pose = red_button_pose;

            if (!press_button_activation_client_.call(press_button_srv))
            {
                ROS_ERROR("Error calling press button activation service.");
                return false;
            }
            else if (!press_button_srv.response.success)
            {
                ROS_ERROR("Failure pressing button. Exiting.");
                return false;
            }
            ROS_INFO("Button pressed.");
            return true;
        }

        bool moveSlider()
        {
            ROS_INFO("Moving the slider...");

            hrii_robothon_msgs::MoveSlider move_slider_srv;
            move_slider_srv.request.robot_id = left_robot_id_;
            prev_robot_id_ = left_robot_id_;
            
            // Real slider pose
            geometry_msgs::TransformStamped sliderTransform;
            try{
                sliderTransform = tf_buffer_.lookupTransform(move_slider_srv.request.robot_id+"_link0", "task_board_starting_slider_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << move_slider_srv.request.robot_id << "_link0 and task_board_starting_slider_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << move_slider_srv.request.robot_id <<"_link0 and task_board_starting_slider_link NOT found!");
            }
            geometry_msgs::Pose slider_pose;
            slider_pose.orientation = sliderTransform.transform.rotation;
            slider_pose.position.x = sliderTransform.transform.translation.x;
            slider_pose.position.y = sliderTransform.transform.translation.y;
            slider_pose.position.z = sliderTransform.transform.translation.z;
            move_slider_srv.request.slider_pose.pose = slider_pose;

            if (!move_slider_activation_client_.call(move_slider_srv))
            {
                ROS_ERROR("Error calling moving slider activation service.");
                return false;
            }
            else if (!move_slider_srv.response.success)
            {
                ROS_ERROR("Failure moving slider. Exiting.");
                return false;
            }
            ROS_INFO("SLIDER MOVED!.");

            return true;
        }

        bool movePlug()
        {
            ROS_INFO("Moving the plug...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(move_plug_activation_service_name_) << " ROS service...");
            move_plug_activation_client_.waitForExistence();

            hrii_robothon_msgs::MovePlug move_plug_srv;
            // move_plug_srv.request.robot_id = left_robot_id_; // now done by task manager


            // Look up task pose w.r.t. world frame
            geometry_msgs::TransformStamped w_T_black_hole_transf;
            try{
                w_T_black_hole_transf = tf_buffer_.lookupTransform("world", "task_board_starting_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw world and task_board_starting_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw world and task_board_starting_connector_hole_link NOT found!");
            }
            geometry_msgs::Pose w_T_black_hole_pose = geometry_msgs::toPose(w_T_black_hole_transf.transform);



            // Call task manager to assign task
            hrii_robothon_msgs::AssignTask assign_task_srv;
            assign_task_srv.request.task_name = hrii_robothon_msgs::AssignTask::Request::MOVE_PLUG_CABLE;
            assign_task_srv.request.task_pose = w_T_black_hole_pose;

            if (!assign_task_client_.call(assign_task_srv))
            {
                ROS_ERROR("Error calling assign task service.");
                return false;
            }
            else if (!assign_task_srv.response.success)
            {
                ROS_ERROR("Failure assigning task. Exiting.");
                ROS_ERROR_STREAM(assign_task_srv.response.message);
                return false;
            }
            ROS_INFO_STREAM(assign_task_srv.response.message);
            
            move_plug_srv.request.robot_id = assign_task_srv.response.robot_id;

            ROS_INFO_STREAM("Previous robot used:" << prev_robot_id_);
            if (prev_robot_id_.compare(move_plug_srv.request.robot_id) != 0)
            {
                if (prev_robot_id_.compare(left_robot_id_) == 0)
                {
                    if (!homing()) return false;
                }
                else if (prev_robot_id_.compare(right_robot_id_) == 0)
                {
                    if (!otherHoming()) return false;
                }
            }
            prev_robot_id_ = move_plug_srv.request.robot_id;

            // ----------------------------- LOOKING FOR THE POSITION OF THE STARTING CONNECTOR HOLE -----------------------------

            // Real black hole pose
            geometry_msgs::TransformStamped startingConnectorHoleTransform;
            try{
                startingConnectorHoleTransform = tf_buffer_.lookupTransform(move_plug_srv.request.robot_id+"_link0", "task_board_starting_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << move_plug_srv.request.robot_id << "_link0 and task_board_starting_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << move_plug_srv.request.robot_id <<"_link0 and task_board_starting_connector_hole_link NOT found!");
            }
            geometry_msgs::Pose starting_connector_hole_pose;
            starting_connector_hole_pose.orientation = startingConnectorHoleTransform.transform.rotation;

            starting_connector_hole_pose.position.x = startingConnectorHoleTransform.transform.translation.x;
            starting_connector_hole_pose.position.y = startingConnectorHoleTransform.transform.translation.y;
            starting_connector_hole_pose.position.z = startingConnectorHoleTransform.transform.translation.z;

            ROS_INFO_STREAM("Starting connector hole position: " << starting_connector_hole_pose.position.x << ", " << starting_connector_hole_pose.position.y << ", " << starting_connector_hole_pose.position.z);
            ROS_INFO_STREAM("Starting connector hole orientation: " << starting_connector_hole_pose.orientation.x << ", " << starting_connector_hole_pose.orientation.y << ", " << starting_connector_hole_pose.orientation.z << ", " << starting_connector_hole_pose.orientation.w);

            move_plug_srv.request.starting_plug_pose.pose = starting_connector_hole_pose;



            

            // ----------------------------- LOOKING FOR THE POSITION OF THE ENDING CONNECTOR HOLE -----------------------------

            // Real black hole pose
            geometry_msgs::TransformStamped endingConnectorHoleTransform;
            try{
                endingConnectorHoleTransform = tf_buffer_.lookupTransform(move_plug_srv.request.robot_id+"_link0", "task_board_ending_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << move_plug_srv.request.robot_id << "_link0 and task_board_ending_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << move_plug_srv.request.robot_id <<"_link0 and task_board_ending_connector_hole_link NOT found!");
            }
            geometry_msgs::Pose ending_connector_hole_pose;
            ending_connector_hole_pose.orientation = endingConnectorHoleTransform.transform.rotation;

            ending_connector_hole_pose.position.x = endingConnectorHoleTransform.transform.translation.x;
            ending_connector_hole_pose.position.y = endingConnectorHoleTransform.transform.translation.y;
            ending_connector_hole_pose.position.z = endingConnectorHoleTransform.transform.translation.z;

            ROS_INFO_STREAM("Ending connector hole position: " << ending_connector_hole_pose.position.x << ", " << ending_connector_hole_pose.position.y << ", " << ending_connector_hole_pose.position.z);
            ROS_INFO_STREAM("Ending connector hole orientation: " << ending_connector_hole_pose.orientation.x << ", " << ending_connector_hole_pose.orientation.y << ", " << ending_connector_hole_pose.orientation.z << ", " << ending_connector_hole_pose.orientation.w);

            move_plug_srv.request.ending_plug_pose.pose = ending_connector_hole_pose;

            if (!move_plug_activation_client_.call(move_plug_srv))
            {
                ROS_ERROR("Error calling move plug activation service.");
                return false;
            }
            else if (!move_plug_srv.response.success)
            {
                ROS_ERROR("Failure moving the plug. Exiting.");
                return false;
            }
            ROS_INFO("Plug moved.");
            return true;
        }

        bool openDoor()
        {
            ROS_INFO("Opening door...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(open_door_activation_service_name_) << " ROS service...");
            open_door_activation_client_.waitForExistence();


            hrii_robothon_msgs::OpenDoor open_door_srv;
            // open_door_srv.request.robot_id = left_robot_id_; // now done by task manager

            // Look up task pose w.r.t. world frame
            geometry_msgs::TransformStamped w_T_red_hole_transf;
            try{
                w_T_red_hole_transf = tf_buffer_.lookupTransform("world", "task_board_ending_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw world and task_board_ending_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw world and task_board_ending_connector_hole_link NOT found!");
            }
            geometry_msgs::Pose w_T_red_hole_pose = geometry_msgs::toPose(w_T_red_hole_transf.transform);

            // Call task manager to assign task
            hrii_robothon_msgs::AssignTask assign_task_srv;
            assign_task_srv.request.task_name = hrii_robothon_msgs::AssignTask::Request::OPEN_DOOR;
            assign_task_srv.request.task_pose = w_T_red_hole_pose;

            if (!assign_task_client_.call(assign_task_srv))
            {
                ROS_ERROR("Error calling assign task service.");
                return false;
            }
            else if (!assign_task_srv.response.success)
            {
                ROS_ERROR("Failure assigning task. Exiting.");
                ROS_ERROR_STREAM(assign_task_srv.response.message);
                return false;
            }
            ROS_INFO_STREAM(assign_task_srv.response.message);
            
            open_door_srv.request.robot_id = assign_task_srv.response.robot_id;

            ROS_INFO_STREAM("Previous robot used:" << prev_robot_id_);
            if (prev_robot_id_.compare(open_door_srv.request.robot_id) != 0)
            {
                if (prev_robot_id_.compare(left_robot_id_) == 0)
                {
                    if (!homing()) return false;
                }
                else if (prev_robot_id_.compare(right_robot_id_) == 0)
                {
                    if (!otherHoming()) return false;
                }
            }
            prev_robot_id_ = open_door_srv.request.robot_id;






            // Get door handle pose
            geometry_msgs::TransformStamped door_handle_transform;
            try
            {
                door_handle_transform = tf_buffer_.lookupTransform(open_door_srv.request.robot_id+"_link0", "task_board_door_handle_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << open_door_srv.request.robot_id << "_link0 and task_board_door_handle_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << open_door_srv.request.robot_id <<"_link0 and task_board_door_handle_link NOT found!");
                return false;
            }

            // Get door center of rotation pose
            geometry_msgs::TransformStamped center_of_rotation_pose_transform;
            try
            {
                center_of_rotation_pose_transform = tf_buffer_.lookupTransform(open_door_srv.request.robot_id+"_link0", "task_board_door_center_of_rotation_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << open_door_srv.request.robot_id <<"_link0 and task_board_door_center_of_rotation_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << open_door_srv.request.robot_id <<"_link0 and task_board_door_center_of_rotation_link NOT found!");
                return false;
            }


            

            
            // Fill service call
            // open_door_srv.request.execution_time = 5.0;
            open_door_srv.request.execution_time = 6.5;
            // open_door_srv.request.final_desired_angle = 1.5707;
            open_door_srv.request.final_desired_angle = 1.6580; // 95 deg
            // open_door_srv.request.final_desired_angle = 1.8326; // 105 deg
            // open_door_srv.request.final_desired_angle = 2.007; // 115 deg
            open_door_srv.request.sampling_time = 0.001;
            open_door_srv.request.door_handle_pose.pose = geometry_msgs::toPose(door_handle_transform.transform);
            open_door_srv.request.center_of_rotation_pose.pose = geometry_msgs::toPose(center_of_rotation_pose_transform.transform);

            if (!open_door_activation_client_.call(open_door_srv))
            {
                ROS_ERROR("Error calling open door service.");
                return false;
            }
            else if (!open_door_srv.response.success)
            {
                ROS_ERROR("Failure opening door. Exiting.");
                return false;
            }
            ROS_INFO("Opening door succeded.");
            return true;
        }

        bool probeCircuit()
        {
            ROS_INFO("Probing the desired circuit...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(probe_circuit_activation_service_name_) << " ROS service...");
            probe_circuit_activation_client_.waitForExistence();

            hrii_robothon_msgs::ProbeCircuit probe_circuit_srv;
            // probe_circuit_srv.request.robot_id = left_robot_id_; now done by task manager

            // Look up task pose w.r.t. world frame
            geometry_msgs::TransformStamped w_T_probe_transf;
            try{
                w_T_probe_transf = tf_buffer_.lookupTransform("world", "task_board_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw world and task_board_probe_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw world and task_board_probe_link NOT found!");
            }
            geometry_msgs::Pose w_T_probe_pose = geometry_msgs::toPose(w_T_probe_transf.transform);

            // Call task manager to assign task
            hrii_robothon_msgs::AssignTask assign_task_srv;
            assign_task_srv.request.task_name = hrii_robothon_msgs::AssignTask::Request::PROBE;
            assign_task_srv.request.task_pose = w_T_probe_pose;

            if (!assign_task_client_.call(assign_task_srv))
            {
                ROS_ERROR("Error calling assign task service.");
                return false;
            }
            else if (!assign_task_srv.response.success)
            {
                ROS_ERROR("Failure assigning task. Exiting.");
                ROS_ERROR_STREAM(assign_task_srv.response.message);
                return false;
            }
            ROS_INFO_STREAM(assign_task_srv.response.message);
            
            probe_circuit_srv.request.robot_id = assign_task_srv.response.robot_id;

            ROS_INFO_STREAM("Previous robot used:" << prev_robot_id_);
            if (prev_robot_id_.compare(probe_circuit_srv.request.robot_id) != 0)
            {
                if (prev_robot_id_.compare(left_robot_id_) == 0)
                {
                    if (!homing()) return false;
                }
                else if (prev_robot_id_.compare(right_robot_id_) == 0)
                {
                    if (!otherHoming()) return false;
                }
            }
            prev_robot_id_ = probe_circuit_srv.request.robot_id;

            // Get probe pose
            geometry_msgs::TransformStamped probe_handle_transform;
            try
            {
                probe_handle_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.robot_id+"_link0", "task_board_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.robot_id << "_link0 and task_board_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.robot_id << "_link0 and task_board_probe_link NOT found!");
                return false;
            }

            // Get circuit to probe pose
            geometry_msgs::TransformStamped circuit_to_probe_transform;
            try
            {
                circuit_to_probe_transform = tf_buffer_.lookupTransform(probe_circuit_srv.request.robot_id+"_link0", "task_board_circuit_to_probe_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << probe_circuit_srv.request.robot_id << "_link0 and task_board_circuit_to_probe_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR_STREAM("Tranform btw " << probe_circuit_srv.request.robot_id << "_link0 and task_board_circuit_to_probe_link NOT found!");
                return false;
            }



            probe_circuit_srv.request.probe_handle_pose.pose = geometry_msgs::toPose(probe_handle_transform.transform);
            probe_circuit_srv.request.circuit_pose.pose = geometry_msgs::toPose(circuit_to_probe_transform.transform);

            if (!probe_circuit_activation_client_.call(probe_circuit_srv))
            {
                ROS_ERROR("Error calling probe circuit activation service.");
                return false;
            }
            else if (!probe_circuit_srv.response.success)
            {
                ROS_ERROR("Failure pressing button. Exiting.");
                return false;
            }
            ROS_INFO("Circuit probed.");

            return true;
        }

        bool stowProbeCable()
        {
            ROS_INFO("Stowing probe cable...");

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(stow_probe_cable_activation_service_name_) << " ROS service...");
            stow_probe_cable_activation_client_.waitForExistence();

            hrii_robothon_msgs::StowProbeCable stow_probe_cable_srv;
            stow_probe_cable_srv.request.probe_holder_robot_id = left_robot_id_;
            stow_probe_cable_srv.request.cable_stower_robot_id = right_robot_id_;

            // Get left wire reel poses in probe_holder_robot frame and cable_stower_robot frame
            geometry_msgs::TransformStamped leftWireReelProbeHolderRobotTransform, leftWireReeCableStowerRobotTransform;
            try
            {
                leftWireReelProbeHolderRobotTransform = tf_buffer_.lookupTransform(stow_probe_cable_srv.request.probe_holder_robot_id+"_link0", "task_board_left_wire_reel_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << stow_probe_cable_srv.request.probe_holder_robot_id << "_link0 and task_board_left_wire_reel_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << stow_probe_cable_srv.request.probe_holder_robot_id <<"_link0 and task_board_left_wire_reel_link NOT found!");
                return false;
            }
            try
            {
                leftWireReeCableStowerRobotTransform = tf_buffer_.lookupTransform(stow_probe_cable_srv.request.cable_stower_robot_id+"_link0", "task_board_left_wire_reel_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id << "_link0 and task_board_left_wire_reel_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id <<"_link0 and task_board_left_wire_reel_link NOT found!");
                return false;
            }
            geometry_msgs::Pose left_wire_reel_probe_holder_robot_pose, left_wire_reel_cable_stower_robot_pose;
            left_wire_reel_probe_holder_robot_pose.orientation = leftWireReelProbeHolderRobotTransform.transform.rotation;
            left_wire_reel_probe_holder_robot_pose.position.x = leftWireReelProbeHolderRobotTransform.transform.translation.x;
            left_wire_reel_probe_holder_robot_pose.position.y = leftWireReelProbeHolderRobotTransform.transform.translation.y;
            left_wire_reel_probe_holder_robot_pose.position.z = leftWireReelProbeHolderRobotTransform.transform.translation.z;

            left_wire_reel_cable_stower_robot_pose.orientation = leftWireReeCableStowerRobotTransform.transform.rotation;
            left_wire_reel_cable_stower_robot_pose.position.x = leftWireReeCableStowerRobotTransform.transform.translation.x;
            left_wire_reel_cable_stower_robot_pose.position.y = leftWireReeCableStowerRobotTransform.transform.translation.y;
            left_wire_reel_cable_stower_robot_pose.position.z = leftWireReeCableStowerRobotTransform.transform.translation.z;

            ROS_INFO_STREAM("Left wire reel (probe holder) pose: " << left_wire_reel_probe_holder_robot_pose.position.x << ", " << left_wire_reel_probe_holder_robot_pose.position.y << ", " << left_wire_reel_probe_holder_robot_pose.position.z);
            ROS_INFO_STREAM("Left wire reel (probe holder) orientation: " << left_wire_reel_probe_holder_robot_pose.orientation.x << ", " << left_wire_reel_probe_holder_robot_pose.orientation.y << ", " << left_wire_reel_probe_holder_robot_pose.orientation.z << ", " << left_wire_reel_probe_holder_robot_pose.orientation.w);

            ROS_INFO_STREAM("Left wire reel (cable stower) pose: " << left_wire_reel_cable_stower_robot_pose.position.x << ", " << left_wire_reel_cable_stower_robot_pose.position.y << ", " << left_wire_reel_cable_stower_robot_pose.position.z);
            ROS_INFO_STREAM("Left wire reel (cable stower) orientation: " << left_wire_reel_cable_stower_robot_pose.orientation.x << ", " << left_wire_reel_cable_stower_robot_pose.orientation.y << ", " << left_wire_reel_cable_stower_robot_pose.orientation.z << ", " << left_wire_reel_cable_stower_robot_pose.orientation.w);

            stow_probe_cable_srv.request.left_wire_reel_probe_holder_robot_pose.pose = left_wire_reel_probe_holder_robot_pose;
            stow_probe_cable_srv.request.left_wire_reel_cable_stower_robot_pose.pose = left_wire_reel_cable_stower_robot_pose;

            // Get task_board_ending_connector_hole_link pose w.r.t probe holder robot
            geometry_msgs::TransformStamped probeHolderRobotToEndingConnectorHoleTransform;
            try
            {
                probeHolderRobotToEndingConnectorHoleTransform = tf_buffer_.lookupTransform(stow_probe_cable_srv.request.probe_holder_robot_id+"_link0", "task_board_ending_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << stow_probe_cable_srv.request.probe_holder_robot_id << "_link0 and task_board_ending_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << stow_probe_cable_srv.request.probe_holder_robot_id <<"_link0 and task_board_ending_connector_hole_link NOT found!");
                return false;
            }
            ROS_INFO_STREAM("Probe holder robot to ending connector hole:\n" << probeHolderRobotToEndingConnectorHoleTransform.transform);
            stow_probe_cable_srv.request.probe_holder_robot_to_ending_connector_hole.pose = geometry_msgs::toPose(probeHolderRobotToEndingConnectorHoleTransform.transform);

            // Get task_board_ending_connector_hole_link pose w.r.t cable stower robot
            geometry_msgs::TransformStamped cableStowerRobotToEndingConnectorHoleTransform;
            try
            {
                cableStowerRobotToEndingConnectorHoleTransform = tf_buffer_.lookupTransform(stow_probe_cable_srv.request.cable_stower_robot_id+"_link0", "task_board_ending_connector_hole_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id << "_link0 and task_board_ending_connector_hole_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id <<"_link0 and task_board_ending_connector_hole_link NOT found!");
                return false;
            }
            ROS_INFO_STREAM("Cable stower robot to ending connector hole:\n" << cableStowerRobotToEndingConnectorHoleTransform.transform);
            stow_probe_cable_srv.request.cable_stower_robot_to_ending_connector_hole.pose = geometry_msgs::toPose(cableStowerRobotToEndingConnectorHoleTransform.transform);

            // Get task_board_base_link pose w.r.t cable stower robot
            geometry_msgs::TransformStamped cableStowerRobotToBaseLinkTransform;
            try
            {
                cableStowerRobotToBaseLinkTransform = tf_buffer_.lookupTransform(stow_probe_cable_srv.request.cable_stower_robot_id+"_link0", "task_board_base_link", ros::Time(0), ros::Duration(3));
                ROS_INFO_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id << "_link0 and task_board_base_link found!");
            }
            catch (tf2::TransformException &ex) 
            {
                ROS_WARN("%s",ex.what());
                ROS_ERROR_STREAM("Tranform btw " << stow_probe_cable_srv.request.cable_stower_robot_id <<"_link0 and task_board_base_link NOT found!");
                return false;
            }
            ROS_INFO_STREAM("Cable stower robot to base link:\n" << cableStowerRobotToBaseLinkTransform.transform);
            stow_probe_cable_srv.request.cable_stower_robot_to_base_link.pose = geometry_msgs::toPose(cableStowerRobotToBaseLinkTransform.transform);

            // Call stow probe cable FSM activation
            if (!stow_probe_cable_activation_client_.call(stow_probe_cable_srv))
            {
                ROS_ERROR("Error calling stow probe cable activation service.");
                return false;
            }
            else if (!stow_probe_cable_srv.response.success)
            {
                ROS_ERROR("Failure stowing probe cable. Exiting.");
                return false;
            }
            ROS_INFO("Probe cable stowed.");
            return true;
        }

        States resolveStateString(const std::string& state_str)
        {
            if (state_str.compare("homing") == 0 || state_str.compare("HOMING") == 0) return States::HOMING;
            if (state_str.compare("other_homing") == 0 || state_str.compare("OTHER_HOMING") == 0) return States::OTHER_HOMING;
            if (state_str.compare("board_detection") == 0 || state_str.compare("BOARD_DETECTION") == 0) return States::BOARD_DETECTION;
            if (state_str.compare("press_red_button") == 0 || state_str.compare("PRESS_RED_BUTTON") == 0) return States::PRESS_RED_BUTTON;
            if (state_str.compare("move_slider") == 0 || state_str.compare("MOVE_SLIDER") == 0) return States::MOVE_SLIDER;
            if (state_str.compare("move_plug_cable") == 0 || state_str.compare("MOVE_PLUG_CABLE") == 0) return States::MOVE_PLUG_CABLE;
            if (state_str.compare("open_door") == 0 || state_str.compare("OPEN_DOOR") == 0) return States::OPEN_DOOR;
            if (state_str.compare("probe") == 0 || state_str.compare("PROBE") == 0) return States::PROBE;
            if (state_str.compare("stow_probe_cable") == 0 || state_str.compare("STOW_PROBE_CABLE") == 0) return States::STOW_PROBE_CABLE;
            if (state_str.compare("press_blue_button") == 0 || state_str.compare("PRESS_BLUE_BUTTON") == 0) return States::PRESS_BLUE_BUTTON;
            if (state_str.compare("exit") == 0 || state_str.compare("EXIT") == 0) return States::EXIT;
            ROS_ERROR("State string not recognired. Return ERROR state.");
            return States::ERROR;
        }

}; // class MainFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_fsm");
    
    MainFSM fsm;

    if (!fsm.init()) return -1;
    fsm.spin();

    return 0;
}