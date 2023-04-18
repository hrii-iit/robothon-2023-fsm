#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from hrii_task_board_fsm.srv import BoardDetection,BoardDetectionResponse,DesiredSliderDisplacement,DesiredSliderDisplacementResponse

count = 1

def slider_desired_displacement(req):
    #Computation of the desired pose from perception section (ToDo)
    global count
    if (count == 1):
        desired_displacement = 0.04
        
    if (count == 2):
        desired_displacement = 0.02

    print("Returning [%s]"%(desired_displacement))
    count = count + 1
    return DesiredSliderDisplacementResponse(desired_displacement)

# Detecting fake board pose
def board_detection(req):
    # Getting tf from task_board_m5_display_link to task_board_base_link

    # Applying tf w_T_d * d_T_bl

    # Publishing tf from world to task_board_m5_display_link

    success = True
    return BoardDetectionResponse(success)
    

def fake_percpetion_server():
    rospy.init_node('fake_percpetion_server')
    slider_srv = rospy.Service('slider_desired_pose', DesiredSliderDisplacement, slider_desired_displacement)
    board_detection_srv = rospy.Service('/fsm/board_detection', BoardDetection, board_detection)
    print("Ready to communicate the desired displacement.")
    rospy.spin()

if __name__ == "__main__":
    fake_percpetion_server()