#!/usr/bin/python3
import rospy
import geometry_msgs.msg as geometry_msgs

from ur.external.contact_sensor import contact_sensor
from ur.commands.ur_commands import RobotUR
import threading


def go_to_position(robot_command):
    print(robot_command)
    myRobot = RobotUR()
    # myRobot.go_to_initial_position(5)

    myRobot.go_to_pose(geometry_msgs.Pose(
        geometry_msgs.Vector3(robot_command[0], robot_command[1], robot_command[2]),
        RobotUR.tool_down_pose
    ))

def thread_function1(command1):
    print("Thread 1 is running")
    print("This thread will wait for the robot to detect any changement in trajectory")

    while True:
        msg = contact_sensor()
        # print(msg)
        if msg is True:
            go_to_position(command1)
            print(msg)
            break

def thread_function2(command2):
    print("Thread 2 is running")
    go_to_position(command2)


def main():
    """
    This function is main test function
    """
    rospy.init_node("test_robot")

    command1 = [-0.736, 0.356, 0.222]
    command2 = [-0.736, 0.100, 0.222]


        # création de thread
    t1 = threading.Thread(target=thread_function1, args=(command1,))
    t2 = threading.Thread(target=thread_function2, args=(command2,))

    # démarrer le thread t1
    t1.start()
    # démarrer le thread t2
    t2.start()

    # # attendre que t1 soit exécuté
    # t1.join()
    # # attendre que t2 soit exécuté
    # t2.join()
    #





if __name__ == '__main__':
    main()
