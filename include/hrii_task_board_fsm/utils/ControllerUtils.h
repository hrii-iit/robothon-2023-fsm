#ifndef __HRII_TASK_BOARD_FSM_CONTROLLER_UTILS_H__
#define __HRII_TASK_BOARD_FSM_CONTROLLER_UTILS_H__

#include <ros/ros.h>
#include <controller_manager_msgs/ListControllers.h>
#include "hrii_robot_msgs/TaskSelection.h"
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>

inline bool checkThresholdSgn(const double& value, const double& threshold)
{ 
    if (threshold > 0 && (value > threshold)) return true;
    if (threshold < 0 && (value < threshold)) return true;
    return false;
}

inline bool waitForRunningController(ros::NodeHandle& nh, 
                                     const std::string& service_name, 
                                     const std::string& controller_name, 
                                     const double& timeout=-1)
{
    // Initialize controller status service
    ros::ServiceClient controller_manager_status_client_ =  nh.serviceClient<controller_manager_msgs::ListControllers>(service_name);

    // Wait for controllers status service
    ROS_INFO_STREAM("Waiting for " << nh.resolveName(service_name) << " ROS service...");
    controller_manager_status_client_.waitForExistence();

    // Check if controller status is running
    controller_manager_msgs::ListControllers controller_list_srv;

    do{
        if (!controller_manager_status_client_.call(controller_list_srv))
        {
            ROS_ERROR_STREAM("Error calling " << nh.resolveName(service_name) << " ROS service.");
            return false;
        }
        else
        {
            for (const auto& controller : controller_list_srv.response.controller){
                if(controller.name == controller_name && controller.state == "running")
                    return true;
            }
        }

    }while(ros::ok());

    return false;
}

inline bool applyContactForce(ros::NodeHandle& nh,
                              const std::string& robot_id,
                              const std::string& controller_name,
                              const int& axis,
                              const geometry_msgs::WrenchStamped& desired_wrench_msg,
                              const double& timeout=-1)
{
    // Robot controller task selection (motion-force task switch)
    // ROS subscriber and publisher definition
    ros::Publisher desired_wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("/"+robot_id+"/"+controller_name+"/desired_wrench", 1);
    while (desired_wrench_pub.getNumSubscribers() > 0)
    {
        ROS_WARN_STREAM_THROTTLE(5, "Wait for subscribers in " << nh.resolveName("/"+robot_id+"/"+controller_name+"/desired_wrench") << " ROS topic...");
        ros::spinOnce();
    }
    
    Eigen::Matrix<double, 6, 1> ext_wrench;
    ros::Subscriber ext_wrench_sub = nh.subscribe<geometry_msgs::WrenchStamped>("/"+robot_id+"/franka_state_controller/F_ext", 1, 
                [&](const geometry_msgs::WrenchStampedConstPtr& msg)
                {
                    tf::wrenchMsgToEigen(msg->wrench, ext_wrench);
                    // Change sign to the ext wrench
                    // ext_wrench = -ext_wrench;
                });
    
    // Define the controller task selection ROS client 
    ros::ServiceClient controller_task_selection_client = nh.serviceClient<hrii_robot_msgs::TaskSelection>("/"+robot_id+"/"+controller_name+"/task_selection");
    ROS_INFO_STREAM("Waiting for " << nh.resolveName("/"+robot_id+"/"+controller_name+"/task_selection") << " ROS service.");
    controller_task_selection_client.waitForExistence();

    // Set desired force to 10N
    ROS_INFO_STREAM("Set desired wrench: " << desired_wrench_msg);
    desired_wrench_pub.publish(desired_wrench_msg);
    Eigen::Matrix<double, 6, 1> desired_wrench_eigen;
    tf::wrenchMsgToEigen(desired_wrench_msg.wrench, desired_wrench_eigen);

    // Set force task in selected axis
    ROS_INFO_STREAM("Set desired axis to force task: " << axis);
    hrii_robot_msgs::TaskSelection task_selection_srv;
    task_selection_srv.request.axis = axis;
    task_selection_srv.request.task = hrii_robot_msgs::TaskSelection::Request::FORCE;
    if (!controller_task_selection_client.call(task_selection_srv))
    {
        ROS_ERROR_STREAM("Error calling " << nh.resolveName("/"+robot_id+"/cart_hybrid_motion_force_controller/task_selection") << " ROS service.");
        return false;
    }
    else if (!task_selection_srv.response.result)
    {
        ROS_ERROR_STREAM(task_selection_srv.response.message);
        return false;
    }
    
    ROS_INFO("Desired axis correctly set in force mode.");

    // Check ext wrench and check stamped. If too old => failure
    ros::Time starting_time = ros::Time::now();
    ros::Rate check_rate(100);

    while (ros::ok())
    {
        desired_wrench_pub.publish(desired_wrench_msg);
        ROS_INFO_STREAM_THROTTLE(1, "Ext force: " << ext_wrench[axis]);
        ROS_INFO_STREAM_THROTTLE(1, "Desired force: " << desired_wrench_eigen[axis]);

        // Check timeout
        if (timeout != -1.0)
        {
            if ((ros::Time::now() - starting_time).toSec() >= timeout)
            {
                ROS_ERROR("Timeout during waiting for contact.");
                return false;
            }
        }

        if (checkThresholdSgn(ext_wrench[axis], desired_wrench_eigen[axis]))
        {
            ROS_INFO("Contact detected.");
            break;
        }

        ros::spinOnce();
        check_rate.sleep();
    }

    // Reset desired wrench
    geometry_msgs::WrenchStamped null_wrench;
    null_wrench.header.stamp = ros::Time::now();
    desired_wrench_pub.publish(null_wrench);

    // Set motion task in selected axis
    ROS_INFO_STREAM("Set desired axis to motion task: " << axis);
    task_selection_srv.request.axis = axis;
    task_selection_srv.request.task = hrii_robot_msgs::TaskSelection::Request::MOTION;
    if (!controller_task_selection_client.call(task_selection_srv))
    {
        ROS_ERROR_STREAM("Error calling " << nh.resolveName("/"+robot_id+"/cart_hybrid_motion_force_controller/task_selection") << " ROS service.");
        return false;
    }
    else if (!task_selection_srv.response.result)
    {
        ROS_ERROR_STREAM(task_selection_srv.response.message);
        return false;
    }

    ROS_INFO("Contact force successfully applied.");

    return true;
}



#endif //__HRII_TASK_BOARD_FSM_CONTROLLER_UTILS_H__