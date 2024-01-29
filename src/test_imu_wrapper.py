#! /usr/bin/env python3
# This file subscribes to the raw IMU data, and corrects for the bias

import rospy
import time
from ros_bno055.msg import OrientationEuler
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import numpy as np

class ImuWrapper:
    def __init__(self):
        rospy.init_node('imu_wrapper_node', anonymous=True)
        rospy.Subscriber('imu/orientation_euler', OrientationEuler, self.imu_callback)
        self.euler_pub = rospy.Publisher('imu/orientation_euler/calibrated', OrientationEuler)
        self.quat_pub = rospy.Publisher('imu/orientation_quat/calibrated', Quaternion)
        self.bias_rotation_matrix = R.from_euler('xyz', [0, 0, 0], degrees=True).as_matrix()
        self.yaw = Float64()
        self.roll = Float64()
        self.pitch = Float64()
        
    def imu_callback(self, msg):
        #print(f'Imu: {msg}')
        self.yaw = msg.heading
        self.roll = msg.roll
        self.pitch = msg.pitch
        rotation_matrix_from_reading = R.from_euler('zyx', [self.yaw, self.pitch, self.roll], degrees=True).as_matrix()
        rotation_angles = R.from_matrix(self.bias_rotation_matrix @ rotation_matrix_from_reading).as_euler('zyx', degrees=True)
        new_msg = OrientationEuler()
        new_msg.header = msg.header
        new_msg.heading = rotation_angles[0]
        new_msg.pitch = rotation_angles[1]
        new_msg.roll = rotation_angles[2]
        self.euler_pub.publish(new_msg)

        quat_data = rotation_angles.as_quat()
        quat_msg = Quaternion()
        quat_msg.x = quat_data[0]
        quat_msg.y = quat_data[1]   
        quat_msg.z = quat_data[2]
        quat_msg.w = quat_data[3]
        self.quat_pub.publish(quat_msg)
        


def main():
    imu = ImuWrapper()
    bias_yaw = 28
    while True:
        input('Press enter to set bias')
        imu.bias_rotation_matrix = R.from_euler('xyz', [1.3, -1, 28], degrees=True).as_matrix()
        print(bias_yaw)
        bias_yaw = (bias_yaw + 1) % 360


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# pitch = 13.06
# roll = -0.8125
# yaw = 186.4