# This file subscribes to the raw IMU data, and corrects for the bias

import rospy
import time
from ros_bno055.msg import OrientationEuler
from std_msgs.msg import Float64
from scipy.spatial.transform import Rotation as R
import numpy as np

class ImuWrapper:
    def __init__(self):
        rospy.init_node('imu_wrapper_node', anonymous=True)
        rospy.Subscriber('imu/orientation_euler', OrientationEuler, self.imu_callback)
        self.pub = rospy.Publisher('imu/orientation_euler/calibrated', OrientationEuler)
        self.rate = rospy.Rate(10)
        self.bias_rotation_matrix = R.from_euler('xyz', [0.8125, -13.06, 45], degrees=True).as_matrix()
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
        new_msg.heading = rotation_angles[0]+180
        new_msg.pitch = rotation_angles[1]
        new_msg.roll = rotation_angles[2]
        self.pub.publish(new_msg)

imu = ImuWrapper()
if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


# pitch = 13.06
# roll = -0.8125
# yaw = 186.4
