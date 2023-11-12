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
pca.channels[0].duty_cycle = 0
time.sleep(1)
print("arming: set high throttle")
servo.throttle = 0.1
time.sleep(1)
print("arming: set stop throttle")
servo.throttle = 0
time.sleep(1)
print("control: start throttle")
# 0.45 gets to 4 and sometimes 4.1
# 0.44 get to 3.7 and 3.8, about same occurances
servo.throttle = 0.0
min_throttle = 0.16
max_throttle = 0.5
throttle_range = max_throttle - min_throttle
increment = throttle_range / 20

try:
    while True:
        for i in range(20):
            # servo.throttle = -(min_throttle + i * increment)
            servo.throttle = 0.3
            print(f'[{time.time()}] throttle: {servo.throttle} throttle command: {-(min_throttle + i * increment)}')
            time.sleep(20)
        
except KeyboardInterrupt:
    pca.channels[0].duty_cycle = 0
    pca.deinit()

# rasp
#   arming
#     amplitude 5v
#     width 1.5ms
#     cycle 20ms
#   running
#     amplitude 5v
#     width 1.7ms
#     cycle 20ms
#   stopping
#     amplitude 0v
# arduino
#   arming
#     amplitude 5v
#     width 1.5ms
#     cycle 20ms
#   running
#     amplitude 5v
#     width 
#     cycle 20ms
#   stopping
#     amplitude 0v
