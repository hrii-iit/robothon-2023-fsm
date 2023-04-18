#include "hrii_task_board_fsm/utils/ControllerUtils.h"
#include "ros/ros.h"
#include "hrii_task_board_fsm/DesiredSliderDisplacement.h"
#include "hrii_task_board_fsm/Homing.h"
#include "hrii_task_board_fsm/MoveSlider.h"
#include "hrii_task_board_fsm/PressButton.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

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

            slider_displacement_service_name_ = "/slider_desired_pose";
            slider_displacement_client_ = nh_.serviceClient<hrii_task_board_fsm::DesiredSliderDisplacement>(slider_displacement_service_name_);

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
            if(!nh_priv_.getParam("task_order", task_order_))
            {
                ROS_ERROR_STREAM("No " << nh_priv_.resolveName("task_order") << " ROS param found");
                return false;
            }
            for (int cnt = 0; cnt < task_order_.size(); cnt++)
            {
                ROS_INFO_STREAM(cnt << ": " << task_order_[cnt]);
            }

            left_robot_controller_manager_status_service_name_ = "/" + left_robot_id_ + "/controller_manager/list_controllers";
            if (!waitForRunningController(nh_, 
                                          left_robot_controller_manager_status_service_name_,
                                          "cart_hybrid_motion_force_controller")) return false;
            ROS_INFO("Left robot controller running.");
            
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
                        // or PRESS_RED_BUTTON
                    break;
                }
                    
                case MainFSM::States::BOARD_DETECTION:
                {
                    // Board detection
                    ROS_INFO("- - - BOARD DETECTION STATE - - -");
                    break;
                }
                    
                case MainFSM::States::PRESS_RED_BUTTON:
                {
                    ROS_INFO("- - - PRESS RED BUTTON STATE - - -");
                    if (!pressRedButton())
                        state_ = MainFSM::States::ERROR;
                        // or PRESS_RED_BUTTON
                    break;
                }

                case MainFSM::States::MOVE_SLIDER:
                {
                    ROS_INFO("- - - MOVE SLIDER STATE - - -");
                    if (!moveSlider())
                        state_ = MainFSM::States::ERROR;
                        // or MOVE_SLIDER
                    break;
                }

                case MainFSM::States::OPEN_DOOR:
                {
                    /* code */
                    break;
                }
                    
                case MainFSM::States::PROBE:
                {
                    /* code */
                    break;
                }
                
                case MainFSM::States::STOW_PROBE_CABLE:
                {
                    /* code */
                    break;
                }

                case MainFSM::States::PRESS_BLUE_BUTTON:
                {
                    /* code */
                    break;
                }

                // case MainFSM::States::EXIT:
                // {
                //     ROS_INFO("Tasks completed. Congratulation!");
                //     break;
                // }

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
                ROS_INFO("Tasks completed. Congratulation!");
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
        
        std::string left_robot_controller_manager_status_service_name_;
        ros::ServiceClient left_robot_controller_manager_status_client_;

        std::string homing_service_name_;
        ros::ServiceClient homing_client_;

        std::string press_button_activation_service_name_;
        ros::ServiceClient press_button_activation_client_;

        std::string move_slider_activation_service_name_;
        ros::ServiceClient move_slider_activation_client_;

        std::string slider_displacement_service_name_;
        ros::ServiceClient slider_displacement_client_;
        
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener{tfBuffer};

        // Robots attributs
        std::string left_robot_id_, right_robot_id_;

        // FSM states declaration
        enum class States {HOMING, 
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
            home_pose.position.x = 0.351;
            home_pose.position.y = -0.233;
            home_pose.position.z = 0.441;
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

        bool pressBlueButton()
        {
            ROS_INFO("Pressing the blue button...");

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
            // ROS_WARN_STREAM("Using fake button pose!");

            // Real button pose
            geometry_msgs::TransformStamped blueButtonTransform;
            try{
                blueButtonTransform = tfBuffer.lookupTransform("franka_left_link0", "task_board_blue_button_link", ros::Time(0), ros::Duration(3));
                ROS_INFO("Tranform btw franka_left_link0 and task_board_blue_button_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR("Tranform btw franka_left_link0 and task_board_blue_button_link NOT found!");
                return false;
            }
            geometry_msgs::Pose blue_button_pose;
            blue_button_pose.position.x = blueButtonTransform.transform.translation.x;
            blue_button_pose.position.y = blueButtonTransform.transform.translation.y;
            blue_button_pose.position.z = blueButtonTransform.transform.translation.z;
            blue_button_pose.orientation.x = 1.0;
            blue_button_pose.orientation.y = 0.0;
            blue_button_pose.orientation.z = 0.0;
            blue_button_pose.orientation.w = 0.0;

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
            // ROS_WARN_STREAM("Using fake button pose!");

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

            hrii_task_board_fsm::DesiredSliderDisplacement slider_displacement_srv;
            hrii_task_board_fsm::MoveSlider move_slider_srv;
            move_slider_srv.request.robot_id = left_robot_id_;
            
            // Real slider pose
            geometry_msgs::TransformStamped sliderTransform;
            try{
                sliderTransform = tfBuffer.lookupTransform("franka_left_link0", "task_board_starting_slider_link", ros::Time(0), ros::Duration(3));
                ROS_INFO("Tranform btw franka_left_link0 and task_board_starting_slider_link found!");
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());
                ros::Duration(1.0).sleep();
                ROS_ERROR("Tranform btw franka_left_link0 and task_board_starting_slider_link NOT found!");
                return false;
            }
            geometry_msgs::Pose slider_pose;
            //slider_pose.orientation = sliderTransform.transform.rotation;
            slider_pose.position.x = sliderTransform.transform.translation.x;
            slider_pose.position.y = sliderTransform.transform.translation.y;
            slider_pose.position.z = sliderTransform.transform.translation.z;
            slider_pose.orientation.x = 1.0;
            slider_pose.orientation.y = 0.0;
            slider_pose.orientation.z = 0.0;
            slider_pose.orientation.w = 0.0;
            move_slider_srv.request.slider_pose.pose = slider_pose;

            // We call the service to get the first slider displacement from imaging side
            slider_displacement_client_.call(slider_displacement_srv);
            double displacement_1 = slider_displacement_srv.response.displacement;
            
            ROS_INFO_STREAM("First displacement: " << displacement_1);

            // Pose of the first reference point where we have to move the slider
            geometry_msgs::Pose reference_pose_1;
            reference_pose_1.position = slider_pose.position; //we assign to the first reference point the same initial pose of the slider
            reference_pose_1.position.y += displacement_1;          //then we add the dispplacement along y-axis
            reference_pose_1.orientation = slider_pose.orientation;     //keeping the same orientation of the initial slider pose

            move_slider_srv.request.reference_pose.pose = reference_pose_1; 
            move_slider_srv.request.times = 1;

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
            ROS_INFO("Slide Moved for the first time.");

            // We call the service to get the second slider displacement from imaging side
            slider_displacement_client_.call(slider_displacement_srv);
            double displacement_2 = slider_displacement_srv.response.displacement;

            ROS_INFO_STREAM("Second displacement: " << displacement_2);

            geometry_msgs::Pose reference_pose_2;
            reference_pose_2.position = slider_pose.position; //we assign to the first reference point the same initial pose of the slider
            reference_pose_2.position.y += displacement_2;          //then we add the dispplacement along y-axis
            reference_pose_2.orientation = slider_pose.orientation;     //keeping the same orientation of the initial slider pose

            move_slider_srv.request.reference_pose.pose = reference_pose_2;
            move_slider_srv.request.times = 2; 

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
            ROS_INFO("Slide Moved for the second time.");



            return true;
        }


        States resolveStateString(const std::string& state_str)
        {
            if (state_str.compare("homing") == 0 || state_str.compare("HOMING") == 0) return States::HOMING;
            if (state_str.compare("board_detection") == 0 || state_str.compare("BOARD_DETECTION") == 0) return States::BOARD_DETECTION;
            if (state_str.compare("press_red_button") == 0 || state_str.compare("PRESS_RED_BUTTON") == 0) return States::PRESS_RED_BUTTON;
            if (state_str.compare("move_slider") == 0 || state_str.compare("MOVE_SLIDER") == 0) return States::MOVE_SLIDER;
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