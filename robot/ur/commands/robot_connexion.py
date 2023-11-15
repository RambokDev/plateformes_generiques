#!/usr/bin/env python3
import logging
import rospy
from std_srvs.srv import Trigger, TriggerRequest


def connexion_state(state: bool):
    """
    This function allow you to connect or disconnect the robot
    Example : True the robot is connected
    @param: state       a bool
    """
    if state == True:
        service = "play"
    elif state == False:
        service = "stop"
    rospy.wait_for_service('/ur_hardware_interface/dashboard/{}'.format(service))
    robot_connexion_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/{}'.format(service),
                                               Trigger)

    try:
        resp = robot_connexion_state()
        return resp.success, resp.message
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    print("Set robot Speed")
    success, message = connexion_state(state=False)
    print(success, message)
