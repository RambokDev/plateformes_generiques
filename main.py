#!/usr/bin/python3
import time
import cv2
import numpy as np
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QMessageBox
import sys
import rospy
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtWidgets import QApplication
from ur_msgs.srv import SetIO
import os
from pypylon import pylon
from numpy import size
import geometry_msgs.msg as geometry_msgs
from ur.commands.ur_commands import RobotUR
from sensor_loop import sensor_loop
from ur.commands.robot_state import set_robot_state
from robot.ur.commands.robot_connexion import connexion_state

x_coef = 1 - 0.0182
x_offset = (-0.0658) / 10  # en cm
y_coef = 1 + 0.01835
y_offset = (-0.35) / 10  # en cm
z_offset = 0.5
board_vector = [-0.64961, -0.15675, -0.45695, 0.00086, 0.00434, 0.00028]
lines_coef = np.load(f'{os.getcwd()}/ur/ihm_tests/CalibrationRobot/up_and_down_img_folder/lines_coef.npy')
i, j, image_circle, image_vierge = 0, 0, 0, 0


class Ui(QtWidgets.QMainWindow, ):
    def __init__(self):
        super(Ui, self).__init__()
        self.imageDeBase = None
        uic.loadUi(f'{os.getcwd()}/ur/ihm_tests/ui/main.ui', self)
        rospy.init_node("test_robotUR")

        self.myRobot = None
        self.PIN_CAM_DEVRACAGE = 5
        self.PIN_VENTURI_VIDE = 0
        self.PIN_CAM_ORIENTATION = 4
        self.ON, self.OFF = 1, 0
        self.set_io_interface = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
        self.take_image.clicked.connect(self.show_image)
        self.take_image_angle.clicked.connect(self.show_image_angle)
        self.angle_state.clicked.connect(self.action_slider_button_clicked)
        self.go_to_box.clicked.connect(self.go_to_box_traj)
        self.stop_robot.clicked.connect(lambda: self.robot_activation(False))
        self.brakes.clicked.connect(lambda: self.robot_activation(True))
        self.quit_button.clicked.connect(QApplication.instance().quit)
        self.init.clicked.connect(lambda: self.myRobot.go_to_initial_position(10))
        self.start_stop_connexion.clicked.connect(self.robot_connexion)
        self.showMaximized()
        self.robot_connexion_state = False
        self.filename = None
        self.sensor_contact = None
        # self.slider_angle.valueChanged.connect(self.slider_state)
        self.show()

    def robot_connexion(self):
        if self.robot_connexion_state == False:
            success, message = connexion_state(True)
            if success:
                self.robot_connexion_state = True
                self.state_connexion_robot.setText("Connected")
                self.state_connexion_robot.setStyleSheet("background-color: green;padding :15px;color:white")
                time.sleep(5)
                self.myRobot = RobotUR()
            else:
                self.pop_up_screen(message)
        else:
            success, message = connexion_state(False)
            if success:
                self.robot_connexion_state = False
                self.state_connexion_robot.setText("Disconnected")
                self.state_connexion_robot.setStyleSheet("background-color: red;padding :15px;color:white")

            else:
                self.pop_up_screen(message)

    def pop_up_screen(self, message):

        # Create a QMessageBox instance for the pop-up
        message_box = QMessageBox()
        message_box.setIcon(QMessageBox.Critical)  # Set the icon (other options: Critical, Warning)
        message_box.setWindowTitle("Error Message")
        message_box.setText("{}".format(message))
        message_box.setStandardButtons(QMessageBox.Ok)  # Set the available buttons (Ok button in this case)
        # Show the pop-up
        message_box.exec_()

    def robot_activation(self, state):
        print(state)
        success, message = set_robot_state(state)
        if not success:
            self.pop_up_screen(message)

    def camera_basler(self, type_camera):
        """
        This function is called for the camera Basler connexion
        :param  type_camera : 0 angle Camera , 1 pickup Camera
        """
        tlFactory = pylon.TlFactory.GetInstance()
        devices = tlFactory.EnumerateDevices()
        print(devices)
        camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(devices[type_camera]))
        camera.Open()
        if type_camera == 0:
            camera.Width = 2590
            camera.Height = 1942
            light = self.PIN_CAM_ORIENTATION

        else:
            camera.Width = 3000
            camera.Height = 2000
            light = self.PIN_CAM_DEVRACAGE

        camera.CenterX.SetValue(True)
        camera.CenterY.SetValue(True)
        camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
        converter = pylon.ImageFormatConverter()
        converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        self.set_io_interface(1, light, self.ON)
        if type_camera == 0:
            time.sleep(2)
        if camera.IsGrabbing():
            grab = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grab.GrabSucceeded():
                img = pylon.PylonImage()
                img.AttachGrabResultBuffer(grab)
                self.filename = "ur/ihm_tests/images/saved_pypylon_img_{}.png".format(type_camera)
                img.Save(pylon.ImageFileFormat_Png, self.filename)
                self.set_io_interface(1, light, self.OFF)
                return self.filename
            camera.Close()

    def show_image(self):
        """
        This function is called in order to display the image in pickup tab
        """
        print("=====Display image =====")
        img_name = self.camera_basler(1)
        self.image.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(1)
        image = cv2.imread(img_name)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image.setPixmap(QPixmap.fromImage(qImg))
        self.image.mousePressEvent = self.getPos

    def show_image_angle(self):
        """
        This function is called in order to display the image in angle tab
        """
        print("=====Display image Angle =====")
        img_name_angle = self.camera_basler(0)
        self.image_angle.setFixedSize(1385, 923)
        ratio, width, height = self.compute_ratio(0)
        image = cv2.imread(img_name_angle)
        image = cv2.resize(image, (round(ratio * width), round(ratio * height)))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        height, width, channels = image.shape
        print(height, width, channels)
        step = channels * width
        qImg = QImage(image.data, width, height, step, QImage.Format_RGB888)
        self.image_angle.setPixmap(QPixmap.fromImage(qImg))

    def action_slider_button_clicked(self):
        print("The slider angle is : ", self.slider_angle.value())
        wrist_angle = self.slider_angle.value()
        pose_camera = [-61.86, -48.79, 65.73, -198.09, -32.20, wrist_angle]
        self.move_wrist_angle(pose_camera)

    def move_wrist_angle(self, pose):
        """
        This function is called when you have validated the angle with the slider, it send the wrist angle command
        """
        self.myRobot.switch_controler_robot("pos_joint_traj_controller")
        self.myRobot.send_joint_trajectory(self.myRobot.convert_deg_to_rad(pose))

    def go_to_box_traj(self):
        """
        This function is called in order to go to the box trajectory,
        normally the venturi is already start
        """
        print("Current pose : {}".format(self.myRobot.get_current_pose()))
        current_pose_orientation = self.myRobot.get_current_pose().orientation
        camera_command = [-0.883, 0.775, 0.300]
        current_quaternions = geometry_msgs.Quaternion(current_pose_orientation.x, current_pose_orientation.y,
                                                       current_pose_orientation.z, current_pose_orientation.w)
        self.myRobot.switch_controler_robot("pose_based_cartesian_traj_controller")
        self.myRobot.go_to_pose(geometry_msgs.Pose(
            geometry_msgs.Vector3(camera_command[0], camera_command[1], camera_command[2]),
            current_quaternions
        ))
        self.set_io_interface(1, self.PIN_VENTURI_VIDE, self.OFF)
        #
        # prepare_command_wrist = [-62.53, -28.76, 70.20, -222.05, -27.66, 180.70] old
        prepare_command_wrist = [-61.88, -35.19, 73.40, -219.29, -32.29, 190]
        self.move_wrist_angle(prepare_command_wrist)
        self.myRobot.switch_controler_robot("pose_based_cartesian_traj_controller")
        self.myRobot.go_to_initial_position(10)

    def compute_ratio(self, camera_type):
        """
        This function compute for you the ratio in order to display the right image in the qt label.
        :param camera_type: the camera type 0 or 1
        """
        if camera_type == 0:
            W = self.image_angle.width()
            H = self.image_angle.height()
        else:
            W = self.image.width()
            H = self.image.height()

        self.imageDeBase = cv2.imread(self.filename)

        width = size(self.imageDeBase, 1)
        height = size(self.imageDeBase, 0)
        ratio = min(W / width, H / height)
        print(ratio, W, H, width, height)
        return ratio, width, height

    def getPos(self, event):
        """
        This function compute the mouse position and convert the image x,y in camera coordinates
        :param event: the mouse event
        """
        x = event.pos().x()
        y = event.pos().y()
        ratio, width, height = self.compute_ratio(1)
        print(ratio)
        print("coordinates in IHM picture: ", x, y)
        realX = round(x / ratio)
        realY = round(y / ratio)
        print("coordinates in Real : ", realX, realY)

        start_pt, vect = self.compute_trajectory(realX, realY, 40)
        print(start_pt, vect)
        print('start : ', start_pt)
        print('vect : ', vect)
        robot_command, vector = self.formatting_commands(start_pt, vect)
        self.go_to_position(robot_command, vector)

    def go_to_position(self, robot_command, vector):
        """
        This function allowed you to go to the position of the object that you have selected
        :param robot_command: a list of integers representing the x,y,z
        :param vector: a list of integers representing the Rx,Ry,Rz
        """
        self.sensor_contact = sensor_loop()
        self.myRobot.go_to_initial_position(5)
        # self.go_to_camera()
        self.myRobot.go_to_pose(geometry_msgs.Pose(
            geometry_msgs.Vector3(robot_command[0], robot_command[1], robot_command[2]),
            RobotUR.tool_down_pose
        ))
        if self.sensor_contact != 1:
            self.set_io_interface(1, self.PIN_VENTURI_VIDE, self.ON)
            self.myRobot.relative_move(vector[0], vector[1], vector[2])
            self.go_to_camera()

    def go_to_camera(self):
        """
        This function allowed you to go to the position of the angle camera
        """
        camera_command = [-0.883, 0.775, 0.594]
        self.myRobot.go_to_pose(geometry_msgs.Pose(
            geometry_msgs.Vector3(camera_command[0], camera_command[1], camera_command[2]),
            RobotUR.tool_horizontal_pose_camera
        ))

    def formatting_commands(self, pos_0, vect):
        """
        This function allowed you to format the commands vector and position
        :param pos_0: a list of integers representing the x,y,z
        :param vect: a list of integers representing the Rx,Ry,Rz
        """
        vecteur = [vect[0][0] * 0.01, vect[1][0] * 0.01, vect[2][0] * 0.01]
        data_1 = (pos_0[0][0] * 0.01) + ((0.0519 * (pos_0[0][0] * 0.01)) + 0.0324) + 0.002
        data_2 = (pos_0[1][0] * 0.01) + ((0.0583 * (pos_0[1][0] * 0.01)) + 0.0036) + 0.0015
        robot_command = [data_1, data_2, pos_0[2][0] * 0.01]
        return robot_command, vecteur

    def compute_trajectory(self, i, j, z0):
        """
        This function compute the trajectory of the robot
        """
        r0 = lines_coef[j][i][0]
        d = lines_coef[j][i][1]
        end_t = 0
        start_t = (z0 - r0[2]) / d[2]

        end_pt = np.array(
            [[((r0[0] + d[0] * end_t) + x_offset) * x_coef], [((r0[1] + d[1] * end_t) + y_offset) * y_coef],
             [(r0[2] + d[2] * end_t) + z_offset]])

        start_pt = np.array(
            [[((r0[0] + d[0] * start_t) + x_offset) * x_coef], [((r0[1] + d[1] * start_t) + y_offset) * y_coef],
             [(r0[2] + d[2] * start_t) + z_offset]])

        board_rot, jac = cv2.Rodrigues(np.array([[board_vector[3]], [board_vector[4]], [board_vector[5]]]))
        board_trans = np.array([[board_vector[0] * 100], [board_vector[1] * 100], [board_vector[2] * 100]])

        end_pt = np.dot(board_rot, end_pt) + board_trans
        start_pt = np.dot(board_rot, start_pt) + board_trans

        vect = end_pt - start_pt

        return start_pt, vect


def main():
    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()


if __name__ == '__main__':
    main()
