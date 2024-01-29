#! /usr/bin/env python3

from std_msgs.msg import Float32MultiArray, Float32
import rospy
import ms5837
from pca9685 import Device
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import smbus2 as smbus

class MS5837Publisher:
    def __init__(self, bus):
        self.pub = rospy.Publisher('heave_data', Float32, queue_size=1)
        self.rate = rospy.Rate(10)
        self.ms5837 = ms5837.MS5837_30BA(bus)

        self.init()
        self.initial_read()

        self.initial_pressure = self.ms5837.pressure(ms5837.UNITS_Pa)
    
    def init(self):
        successful_init = False
        while not successful_init:
            try:
                self.ms5837.init()
                successful_init = True
            except IOError:
                rospy.logerr('I2C init error')

    def initial_read(self):
        successful_read = False
        while not successful_read:
            try:
                self.ms5837.read()
                successful_read = True
            except IOError:
                rospy.logerr('I2C read error')

    def run(self):
        while not rospy.is_shutdown():
            try:
                self.ms5837.read()
            except IOError:
                rospy.logerr('I2C read error')
                self.rate.sleep()
                continue
            ms5837_data = Float32()
            ms5837_data.data = -self.ms5837.relative_depth(self.initial_pressure)
            self.pub.publish(ms5837_data)
            # log
            #rospy.loginfo('Publishing: %s', ms5837_data.data)
            self.rate.sleep()


class PCA9685Subscriber:
    def __init__(self, bus, min_pulse=1100, max_pulse=1900, frequency=50):
        # self.pca = PCA9685(bus)
        # self.pca.frequency = 50
        # self.servos = [servo.ContinuousServo(self.pca.channels[i], min_pulse=1100, max_pulse=1900) for i in range(16)]
        self.frequency = frequency
        self._min_duty = int((min_pulse * self.frequency) / 1000000 * 0x0FFF)
        self._max_duty = int((max_pulse * self.frequency) / 1000000 * 0x0FFF)
        self._duty_range = self._max_duty - self._min_duty
        # print(f'min: {self._min_duty}, max: {self._max_duty}, range: {self._duty_range}')
        self.pca = Device(bus, 0x40)
        self.pca.set_pwm_frequency(self.frequency)

        # arm the servos
        self.set_all_value(0.2)
        time.sleep(0.5)
        self.set_all_value(0)
        time.sleep(5)
        rospy.loginfo("Armed")

        sub = rospy.Subscriber("motor_cmd", Float32MultiArray, self.callback)

    def callback(self, msg):
        data = msg.data
        # print(f'{msg}')
        for i, datum in enumerate(data):
            self.set_throttle(i, datum)

    def set_throttle(self, thruster_num, value):
        if not (-1.0 <= value <= 1.0):
            raise ValueError("Must be -1.0 to 1.0")
        value = (value + 1) / 2
        duty_cycle = int(self._min_duty + value * self._duty_range)
        # print(f'{thruster_num}: {duty_cycle}')
        try:
            self.pca.set_pwm(thruster_num, duty_cycle)
        except IOError:
            rospy.logerr('I2C write error')
            return

    def spin (self):
        rospy.spin()
    
    def set_all_value(self, value):
        for i  in range(16):
            self.set_throttle(i, value)

def main():
    rospy.init_node('hardware_i2c', anonymous=True)
    i2c_bus = smbus.SMBus(1)
    
    ms5837_publisher = MS5837Publisher(i2c_bus)
    pca9685_subscriber = PCA9685Subscriber(i2c_bus)
    try:
        ms5837_publisher.run()    
    except rospy.ROSInterruptException:
        pass
    finally:
        pca9685_subscriber.set_all_value(0)



if __name__ == '__main__':
    main()