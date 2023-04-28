#! /usr/bin/env python3

import rospy
import tf2_ros
from hrii_robothon_msgs.srv import AssignTask, AssignTaskResponse, AssignTaskRequest
import hrii_board_localization.utils.geometry_msgs_operations as gmo

class TaskManager:
    def __init__(self, reference_frame, available_robots):
        self.assign_task_server = rospy.Service('~assign_task', AssignTask, self.assign_task_callback)
        self.available_robots = available_robots
        self.reference_frame_ = reference_frame
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def assign_task_callback(self, req):
        rospy.loginfo("Assigning task to the proper robot: "+req.task_name)

        res = AssignTaskResponse()

        if req.task_name == AssignTaskRequest.MOVE_PLUG_CABLE:

            task_rpy = gmo.get_rpy(req.task_pose.orientation)
            rospy.loginfo("Task orientation:"+str(task_rpy[2]))

            for robot in self.available_robots:
                rospy.loginfo("Candidate: "+robot)
                pose = self.get_robot_pose(robot+"_link0")
                rpy = gmo.get_rpy(pose.orientation)
                rospy.loginfo("Robot orientation:"+str(rpy[2]))

                if abs(rpy[2] - task_rpy[2]) < 3.1415:
                    res.robot_id = robot
                    res.success = True
                    res.message = "Task: "+req.task_name+" assigned to "+res.robot_id
                    rospy.loginfo(res.message)
                    return res
        
        elif req.task_name == AssignTaskRequest.PROBE:
            task_rpy = gmo.get_rpy(req.task_pose.orientation)
            rospy.loginfo("Task orientation:"+str(task_rpy[2]))

            for robot in self.available_robots:
                rospy.loginfo("Candidate: "+robot)
                pose = self.get_robot_pose(robot+"_link0")
                rpy = gmo.get_rpy(pose.orientation)
                rospy.loginfo("Robot orientation:"+str(rpy[2]))
                
                if abs(rpy[2] - task_rpy[2]) < 3.1415:
                    res.robot_id = robot
                    res.success = True
                    res.message = "Task: "+req.task_name+" assigned to "+res.robot_id
                    rospy.loginfo(res.message)
                    return res

        elif req.task_name == AssignTaskRequest.OPEN_DOOR:
            task_rpy = gmo.get_rpy(req.task_pose.orientation)
            rospy.loginfo("Task orientation:"+str(task_rpy[2]))

            for robot in self.available_robots:
                rospy.loginfo("Candidate: "+robot)
                pose = self.get_robot_pose(robot+"_link0")
                rpy = gmo.get_rpy(pose.orientation)
                rospy.loginfo("Robot orientation:"+str(rpy[2]))
                
                if abs(rpy[2] - task_rpy[2]) < 3.1415:
                    res.robot_id = robot
                    res.success = True
                    res.message = "Task: "+req.task_name+" assigned to "+res.robot_id
                    rospy.loginfo(res.message)
                    return res

        else:
            res.success = False
            res.message = "Task not recognired."
            rospy.logerr(res.message)
            return res

        res.success = False
        res.message = "Task not assigned"
        rospy.logerr(res.message)
        return res

    def get_robot_pose(self, robot_base_link):
        try:
            transf = self.tf_buffer.lookup_transform(self.reference_frame_, robot_base_link, rospy.Time(0), timeout=rospy.Duration(10))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No tf from "+self.reference_frame_+" to "+robot_base_link+". Exiting")
            return False
        return gmo.transform_to_pose(transf.transform)
  
if __name__ == '__main__':
    try:
        rospy.init_node("task_manager")

        reference_frame = rospy.get_param("~reference_frame")
        available_robots = rospy.get_param("~available_robots")

        rospy.loginfo("Available robots:")
        for available_robot in available_robots:
            rospy.loginfo(available_robot)
            
        task_manager = TaskManager(reference_frame, available_robots)
        
        rospy.loginfo("Task manager available.")

        rospy.spin()
    except rospy.ROSInterruptException:
        pass