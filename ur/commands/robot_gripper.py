#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    print("1 ==== OPEN")
    print("2 ==== CLOSE")

    while True:
        user_input = input("Enter a number (or 'q' to quit): ")
        if user_input == 'q':
            break  # Exit the loop if the user enters 'q'

        # Perform some operation with the user input
        choice = int(user_input)
        if choice == 1:
            str_choice = "open"
            rospy.loginfo(str_choice)
            pub.publish(str_choice)
            rate.sleep()
        else:
            str_choice = "close"
            rospy.loginfo(str_choice)
            pub.publish(str_choice)
            rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
