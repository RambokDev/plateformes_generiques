#!/usr/bin/python3

import rospy
from ur_msgs.msg import IOStates
from std_srvs.srv import Trigger, TriggerRequest


def set_robot_state(state: bool):
    """
    This function allowed you to control the robot state ( ON , OFF )
    Example : True      in order to power_on the robot controller and brakes
    Example : False     in order to power_off the robot controller
    @param: state       a Boolean
    """
    print(state)
    if state:
        robot_state_str = 'brake_release'
    elif not state:
        robot_state_str = 'power_off'

    rospy.wait_for_service('/ur_hardware_interface/dashboard/{}'.format(robot_state_str))
    robot_change_state = rospy.ServiceProxy('/ur_hardware_interface/dashboard/{}'.format(robot_state_str), Trigger)

    try:
        resp = robot_change_state()
        return resp.success, resp.message
    except rospy.ServiceException as exc:
        print("Service did not process request: " + str(exc))


if __name__ == '__main__':
    print("this is a file for testing the robot connexion ")

    success, message = set_robot_state(state=False)
    print("success: " + str(success))
    print("message: " + str(message))
