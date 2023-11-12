#! /usr/bin/env python3
import rospy
import time
from std_msgs.msg import Float32MultiArray
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

class PCA9685Subscriber:
    def __init__(self):
        rospy.init_node('pca9685_subscriber')
        i2c_bus = busio.I2C(SCL, SDA)

        self.pca = PCA9685(i2c_bus)
        self.pca.frequency = 50
        self.servos = [servo.ContinuousServo(self.pca.channels[i], min_pulse=1100, max_pulse=1900) for i in range(16)]

        # arm the servos
        for servo_instance in self.servos:
            servo_instance.throttle = 0
        time.sleep(7)

        sub = rospy.Subscriber("motor_cmd", Float32MultiArray, self.callback)

    def callback(self, msg):
        data = msg.data
        print(data)
        for i, datum in enumerate(data):
            self.servos[i].throttle = datum
    
    def spin (self):
        rospy.spin()
    
    def destroy (self):
        # set pca channels directly to 0
        for i in range(16):
            self.pca.channels[i].duty_cycle = 0
        self.pca.deinit()

if __name__ == '__main__':
    subscriber = PCA9685Subscriber()
    try:
        subscriber.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        subscriber.destroy()
