#!/usr/bin/env python
import rospy
import numpy
from std_msgs.msg import Float32
import tf2_ros
import geometry_msgs.msg
from hrii_robothon_msgs.srv import BoardDetection,BoardDetectionResponse,DesiredSliderDisplacement,DesiredSliderDisplacementResponse

count = 1

# Send relative fake slider displacement
def slider_desired_displacement(req):
    global count
    global task_accomplished
    if (count == 1):
        desired_displacement = 0.013 # Absolute movement w.r.t. initial pose
        task_accomplished = False
        
    if (count == 2):
        desired_displacement = 0.012 # Absolute movement w.r.t. initial pose - out of scale
        task_accomplished = False
                
    if (count == 3):
        desired_displacement = -0.22 # Absolute movement w.r.t. initial pose
        task_accomplished = False

    if (count == 4):
        desired_displacement = 0.0
        task_accomplished = True

    print("Returning [%s]"%(desired_displacement))
    count = count + 1
    return DesiredSliderDisplacementResponse(task_accomplished, desired_displacement)

# Detecting fake board pose
def board_detection(req):
    # Creating fake tf from world to task_board_base_link
    w_T_bl = geometry_msgs.msg.TransformStamped()
    w_T_bl.header.stamp = rospy.Time.now()
    w_T_bl.header.frame_id = "world"
    w_T_bl.child_frame_id = "task_board_base_link"
    w_T_bl.transform.translation.x = 0.0
    w_T_bl.transform.translation.y = 0.0
    w_T_bl.transform.translation.z = 0.0
    w_T_bl.transform.rotation.x = 0.0
    w_T_bl.transform.rotation.y = 0.0
    w_T_bl.transform.rotation.z = 0.383
    w_T_bl.transform.rotation.w = 0.924

    # Getting tf from task_board_m5_display_link to task_board_base_link
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)
    # try:
    #     # (d_T_bl_trans, d_T_bl_rot) = tfBuffer.lookup_transform('task_board_m5_display_link', 'task_board_base_link', rospy.Time(0), rospy.Duration(3.0))
    #     d_T_bl = tfBuffer.lookup_transform('task_board_m5_display_link', 'task_board_base_link', rospy.Time(0), rospy.Duration(3.0))
    # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #     print("Tranform btw task_board_m5_display_link and task_board_base_link NOT found!")
    #     success = False
    #     return BoardDetectionResponse(success)

    # print("Tranform btw task_board_m5_display_link and task_board_base_link found!")

    # Publishing static tf from world to task_board_base_link
    br = tf2_ros.StaticTransformBroadcaster()
    br.sendTransform(w_T_bl)

    success = True
    return BoardDetectionResponse(success)
    

if __name__ == "__main__":
    rospy.init_node('fake_percpetion_server')
    slider_srv = rospy.Service('slider_desired_pose', DesiredSliderDisplacement, slider_desired_displacement)
    board_detection_srv = rospy.Service('/fsm/board_detection', BoardDetection, board_detection)
    print("Fake perception node initialized.")
    rospy.spin()