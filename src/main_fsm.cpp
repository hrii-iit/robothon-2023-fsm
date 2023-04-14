#include "ros/ros.h"
#include "hrii_task_board_fsm/PressButton.h"
#include "hrii_task_board_fsm/MoveSlider.h"
#include "hrii_task_board_fsm/Homing.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <controller_manager_msgs/ListControllers.h>

class MainFSM
{
    public:
        MainFSM() : nh_priv_("~")
        {
            homing_service_name_ = "homing_fsm/activate";
            homing_client_ = nh_.serviceClient<hrii_task_board_fsm::Homing>(homing_service_name_);

            press_button_activation_service_name_ = "press_button_fsm/activate";
            press_button_activation_client_ = nh_.serviceClient<hrii_task_board_fsm::PressButton>(press_button_activation_service_name_);

            move_slider_activation_service_name_ = "move_slider_fsm/activate";
            move_slider_activation_client_ = nh_.serviceClient<hrii_task_board_fsm::MoveSlider>(move_slider_activation_service_name_);

            state_ = MainFSM::states::HOMING;
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

            // Initialize controller status service
            left_robot_controller_manager_status_service_name_ = "/" + left_robot_id_ + "/controller_manager/list_controllers";
            left_robot_controller_manager_status_client_ =  nh_.serviceClient<controller_manager_msgs::ListControllers>(left_robot_controller_manager_status_service_name_);

            // Wait for controllers status service
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(left_robot_controller_manager_status_service_name_) << " ROS service...");
            left_robot_controller_manager_status_client_.waitForExistence();

            // Check if controller status is running
            controller_manager_msgs::ListControllers left_robot_controller_list_srv;
            bool controller_started_flag = false;
            do{

                if (!left_robot_controller_manager_status_client_.call(left_robot_controller_list_srv))
                {
                    ROS_ERROR("Error calling left robot controller manager status service.");
                    return false;
                }
                else
                {
                    for (const auto& controller : left_robot_controller_list_srv.response.controller){
                        if(controller.name == "cart_hybrid_motion_force_controller" && controller.state == "running")
                            controller_started_flag = true;
                    }
                }

            }while(!controller_started_flag);

            // Wait for task services activation
            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(homing_service_name_) << " ROS service...");
            homing_client_.waitForExistence();

            ROS_INFO_STREAM("Waiting for " << nh_.resolveName(press_button_activation_service_name_) << " ROS service...");
            press_button_activation_client_.waitForExistence();

            ROS_INFO("All services are available.");
            return true;
        }

        void spin()
        {
            while (ros::ok() && state_ != MainFSM::states::EXIT && state_ != MainFSM::states::ERROR)
            {
                switch (state_)
                {
                case MainFSM::states::HOMING:
                {
                    // Move to homing pose and wait for start
                    ROS_INFO("- - - HOMING STATE - - -");
                    if (homing())
                        state_ = MainFSM::states::BOARD_DETECTION;
                    else
                        state_ = MainFSM::states::ERROR;
                        // or PRESS_RED_BUTTON
                    break;
                }
                    
                case MainFSM::states::BOARD_DETECTION:
                {
                    // Board detection
                    ROS_INFO("- - - BOARD DETECTION STATE - - -");
                    state_ = MainFSM::states::PRESS_RED_BUTTON;
                    break;
                }
                    
                case MainFSM::states::PRESS_RED_BUTTON:
                {
                    ROS_INFO("- - - PRESS RED BUTTON STATE - - -");
                    if (pressRedButton())
                        state_ = MainFSM::states::MOVE_SLIDER;
                    else
                        state_ = MainFSM::states::ERROR;
                        // or PRESS_RED_BUTTON
                    break;
                }

                case MainFSM::states::MOVE_SLIDER:
                {
                    ROS_INFO("- - - MOVE SLIDER STATE - - -");
                    if (moveSlider())
                        state_ = MainFSM::states::OPEN_DOOR;
                    else
                        state_ = MainFSM::states::ERROR;
                        // or MOVE_SLIDER
                    break;
                }

                case MainFSM::states::OPEN_DOOR:
                {
                    /* code */
                    break;
                }
                    
                case MainFSM::states::PROBE:
                {
                    /* code */
                    break;
                }
                    
                
                case MainFSM::states::STOW_PROBE_CABLE:
                {
                    /* code */
                    break;
                }

                case MainFSM::states::PRESS_BLUE_BUTTON:
                {
                    /* code */
                    break;
                }

                case MainFSM::states::EXIT:
                {
                    ROS_INFO("Tasks completed. Congratulation!");
                    break;
                }

                default:
                {
                    state_ = MainFSM::states::ERROR;
                    ROS_INFO("State not implemented. Error.");
                    break;
                }
                }
            }
            
        }
    
    private:
        // ROS attributes
        ros::NodeHandle nh_, nh_priv_;
        
        std::string left_robot_controller_manager_status_service_name_;
        ros::ServiceClient left_robot_controller_manager_status_client_;

        std::string homing_service_name_;
        ros::ServiceClient homing_client_;

        std::string press_button_activation_service_name_;
        ros::ServiceClient press_button_activation_client_;

        std::string move_slider_activation_service_name_;
        ros::ServiceClient move_slider_activation_client_;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{tfBuffer};

        // Robots attributs
        std::string left_robot_id_, right_robot_id_;

        // FSM states declaration
        enum class states {HOMING, 
                            PRESS_RED_BUTTON, 
                            BOARD_DETECTION,
                            MOVE_SLIDER, 
                            OPEN_DOOR,
                            PROBE,
                            STOW_PROBE_CABLE,
                            PRESS_BLUE_BUTTON,
                            EXIT,
                            ERROR} state_;

        bool homing()
        {
            ROS_INFO("Go to home position...");

            hrii_task_board_fsm::Homing homing_srv;
            homing_srv.request.robot_id = left_robot_id_;

            // Home pose
            geometry_msgs::Pose home_pose;
            home_pose.position.x = 0.383;
            home_pose.position.y = -0.253;
            home_pose.position.z = 0.379;
            home_pose.orientation.x = -0.693;
            home_pose.orientation.y = 0.706;
            home_pose.orientation.z = -0.104;
            home_pose.orientation.w = -0.104;
            homing_srv.request.home_pose.pose = home_pose;

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

        bool pressRedButton()
        {
            ROS_INFO("Pressing the red button...");

            hrii_task_board_fsm::PressButton press_button_srv;
            press_button_srv.request.robot_id = left_robot_id_;

            // Fake button pose
            // geometry_msgs::Pose fake_button_pose;
            // fake_button_pose.position.x = 0.3;
            // fake_button_pose.position.y = 0.0;
            // fake_button_pose.position.z = 0.3;
            // fake_button_pose.orientation.x = 1.0;
            // fake_button_pose.orientation.y = 0.0;
            // fake_button_pose.orientation.z = 0.0;
            // fake_button_pose.orientation.w = 0.0;
            // press_button_srv.request.button_pose.pose = fake_button_pose;

            // Real button pose
            geometry_msgs::TransformStamped redButtonTransform;
            try{
                redButtonTransform = tfBuffer.lookupTransform("franka_left_link0", "task_board_red_button_link", ros::Time(0), ros::Duration(3));
                ROS_INFO("Tranform btw franka_left_link0 and task_board_red_button_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR("Tranform btw franka_left_link0 and task_board_red_button_link NOT found!");
                return false;
            }
            geometry_msgs::Pose red_button_pose;
            //red_button_pose.orientation = redButtonTransform.transform.rotation;
            red_button_pose.position.x = redButtonTransform.transform.translation.x;
            red_button_pose.position.y = redButtonTransform.transform.translation.y;
            red_button_pose.position.z = redButtonTransform.transform.translation.z;
            red_button_pose.orientation.x = 1.0;
            red_button_pose.orientation.y = 0.0;
            red_button_pose.orientation.z = 0.0;
            red_button_pose.orientation.w = 0.0;

            ROS_INFO_STREAM("Red button pose: " << red_button_pose.position.x << ", " << red_button_pose.position.y << ", " << red_button_pose.position.z);
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

            hrii_task_board_fsm::MoveSlider move_slider_srv;
            move_slider_srv.request.robot_id = right_robot_id_;
            geometry_msgs::Pose fake_button_pose;
            fake_button_pose.position.x = 0.3;
            fake_button_pose.position.y = 0.0;
            fake_button_pose.position.z = 0.0;
            fake_button_pose.orientation.x = 1.0;
            fake_button_pose.orientation.y = 0.0;
            fake_button_pose.orientation.z = 0.0;
            fake_button_pose.orientation.w = 0.0;
            move_slider_srv.request.slider_pose.pose = fake_button_pose;

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
            ROS_INFO("Slide Moved.");
            return true;
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