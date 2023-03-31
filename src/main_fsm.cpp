#include "ros/ros.h"
#include "hrii_task_board_fsm/PressButton.h"

class MainFSM
{
    public:
        MainFSM() : nh_priv_("~")
        {
            press_button_activation_service_name_ = "press_button_fsm/activate";
            press_button_activation_client_ = nh_.serviceClient<hrii_task_board_fsm::PressButton>(press_button_activation_service_name_);
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
                    state_ = MainFSM::states::BOARD_DETECTION;
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
                    if (pressRedButton())
                        state_ = MainFSM::states::MOVE_SLIDER;
                    else
                        state_ = MainFSM::states::ERROR;
                        // or PRESS_RED_BUTTON
                    break;
                }

                case MainFSM::states::MOVE_SLIDER:
                {
                    /* code */
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
        std::string press_button_activation_service_name_;
        ros::ServiceClient press_button_activation_client_;

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

        bool pressRedButton()
        {
            ROS_INFO("Pressing the red button...");

            hrii_task_board_fsm::PressButton press_button_srv;
            press_button_srv.request.robot_id = left_robot_id_;
            geometry_msgs::Pose fake_button_pose;
            fake_button_pose.position.x = 0.3;
            fake_button_pose.position.y = 0.0;
            fake_button_pose.position.z = 0.3;
            fake_button_pose.orientation.x = 1.0;
            fake_button_pose.orientation.y = 0.0;
            fake_button_pose.orientation.z = 0.0;
            fake_button_pose.orientation.w = 0.0;
            press_button_srv.request.button_pose.pose = fake_button_pose;

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
}; // class MainFSM

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main_fsm");
    
    MainFSM fsm;

    if (!fsm.init()) return -1;

    fsm.spin();

    return 0;
}