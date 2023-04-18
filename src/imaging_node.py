#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from hrii_task_board_fsm.srv import DesiredSliderDisplacement,DesiredSliderDisplacementResponse

count = 1

def slider_desired_displacement(req):
    #Computation of the desired pose from imaging section (ToDo)
    global count
    if (count == 1):
        desired_displacement = 0.04
        
    if (count == 2):
        desired_displacement = 0.02

    print("Returning [%s]"%(desired_displacement))
    count = count + 1
    return DesiredSliderDisplacementResponse(desired_displacement)
    

def imaging_server():
    rospy.init_node('imaging_server')
    s = rospy.Service('slider_desired_pose', DesiredSliderDisplacement, slider_desired_displacement)
    print("Ready to communicate the desired displacement.")
    rospy.spin()

if __name__ == "__main__":
    imaging_server()