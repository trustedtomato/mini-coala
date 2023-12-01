from std_msgs.msg import Float32MultiArray, Float32
import rospy
from ms5837 import MS5837_30BA
from device import Device
import time
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import smbus2 as smbus

class PCA9685Subscriber:
    def __init__(self, bus, min_pulse=1100, max_pulse=1900, frequency=50):
        # self.pca = PCA9685(bus)
        # self.pca.frequency = 50
        # self.servos = [servo.ContinuousServo(self.pca.channels[i], min_pulse=1100, max_pulse=1900) for i in range(16)]
        self.frequency = frequency
        self._min_duty = int((min_pulse * self.frequency) / 1000000 * 0x0FFF)
        self._max_duty = int((max_pulse * self.frequency) / 1000000 * 0x0FFF)
        self._duty_range = self._max_duty - self._min_duty
        print(f'min: {self._min_duty}, max: {self._max_duty}, range: {self._duty_range}')
        self.pca = Device(bus, 0x40)
        self.pca.set_pwm_frequency(self.frequency)

        # arm the servos
        #self.set_all_value(0.1)
        #time.sleep(0.5)
        #self.set_all_value(0)
        #time.sleep(5)

        sub = rospy.Subscriber("motor_cmd", Float32MultiArray, self.callback)

    def callback(self, msg):
        data = msg.data
        print(f'{msg}')
        for i, datum in enumerate(data):
            self.set_throttle(i, datum)

    def set_throttle(self, thruster_num, value):
        if not (-1.0 <= value <= 1.0):
            raise ValueError("Must be 0.0 to 1.0")
        value = (value + 1) / 2
        duty_cycle = int(self._min_duty + value * self._duty_range)
        self.pca.set_pwm(thruster_num, duty_cycle)

    def spin (self):
        rospy.spin()
    
    def set_all_value(self, value):
        for i  in range(16):
            self.set_throttle(i, value)

pin = 10
try:
    rospy.init_node('test_i2c', anonymous=True)
    bus = smbus.SMBus(1)
    pca = PCA9685Subscriber(bus)
    pca.set_throttle(pin, 0)
    time.sleep(0.5)
    pca.set_throttle(pin, 0.1)
    time.sleep(0.5)
    pca.set_throttle(pin, 0)
    time.sleep(2)
    # 0x40 from i2cdetect -y 1 (1 if Raspberry pi 2)
    i = 0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        print(((i/10) % 2)/10)
        pca.set_throttle(pin, 0.2)
        i += 1
        rate.sleep()
finally:
    pca.set_throttle(pin, 0)
