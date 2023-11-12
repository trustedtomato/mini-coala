#! /usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float32MultiArray
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from random import random

i2c_bus = busio.I2C(SCL, SDA)
pca = PCA9685(i2c_bus)
pca.frequency = 50
# servo = servo.ContinuousServo(pca.channels[0], min_pulse=1150, max_pulse=1830)
servo = servo.ContinuousServo(pca.channels[0], min_pulse=1100, max_pulse=1900)

print("arming: clean up")
servo.throttle = -1
time.sleep(10)