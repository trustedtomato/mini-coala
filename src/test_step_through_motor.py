# This file is useful for testing the thrusters one by one, with different PWM values.

import rospy
from std_msgs.msg import Float32MultiArray, Float64, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ros_bno055.msg import OrientationEuler
import time

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True, disable_signals=True)
        self.pub = rospy.Publisher('motor_cmd', Float32MultiArray, queue_size=1)
        rospy.Subscriber('joystick_data', Float32MultiArray, self.joystick_callback)
        rospy.Subscriber('imu/orientation_euler', OrientationEuler, self.imu_callback)
        rospy.Subscriber('heave_data', Float32, self.pressure_callback)
        self.rate = rospy.Rate(10) # 10hz
        self.msg = Float32MultiArray()
        self.msg.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.yaw_setpoint = 0
        self.heave_setpoint = 0
        self.yaw = Float64()
        self.roll = Float64()
        self.pitch = Float64()
        self.pressure = 0
        self.temperature = 0
        self.heave = 0
        self.altitude = 0

    def joystick_callback(self, msg):
        #print(f'Joy: {msg}')
        self.yaw_setpoint = msg.data[0]
        self.heave_setpoint = msg.data[1]

    def imu_callback(self, msg):
        #print(f'Imu: {msg}')
        self.yaw = msg.heading
        self.roll = msg.roll
        self.pitch = msg.pitch

    def pressure_callback(self, msg):
        #print(f'Pressure: {msg}')
        self.heave = msg.data

    def stop(self):
        self.msg.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pub.publish(self.msg)

    def set_pwm(self, data):
        self.msg.data = data
        self.pub.publish(self.msg)


controller_node = ControllerNode()
heave_thruster_indices = [2, 3, 5, 7, 8, 9]
yaw_thruster_indices = [0, 1, 4, 6]
thruster_indices = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
heave_thrusters = [{'idx': 2, 'down': -0.62, 'up': 0.90},
                   {'idx': 3, 'down': -0.61, 'up': 0.65},
                   {'idx': 5, 'down': -0.85 * 0.72, 'up': 0.72},
                   {'idx': 7, 'down': 0.67 * 0.72, 'up': -0.93},
                   {'idx': 8, 'down': 0.5 * 0.72, 'up': -0.72},
                   {'idx': 9, 'down': 0.69 * 0.72, 'up': -1}]

yaw_thrusters = [{'idx': 0, 'forward': 0.8, 'backward': -0.5},#'forward': 0.67
                 {'idx': 1, 'forward': -0.93, 'backward': 0.33},#'forward': -0.9
                 {'idx': 4, 'forward': -0.82, 'backward': 0.43},#'forward': -1,
                 {'idx': 6, 'forward': -1, 'backward': 0.4}]#'forward': -0.9

forward_thruster_coefficients = [0.67, -0.9, -0.95, -1, -1, -0.85, -0.9, 0.67, 0.50, 0.69]
backward_thruster_coefficients = [0.95, -1, -1, -0.71, -1, -0.86, -1, 1, 0.77, 0.95]
def main():
    rate = rospy.Rate(20)
    while True:
        # pwm_num = max(0x1000 * (i % 0x11) - 1, 0)
        # rospy.loginfo('PWM: %s', pwm_num)
        
        
        pwm_data = [0 for _ in range(16)]
        controller_node.set_pwm(pwm_data)
        for thruster in heave_thrusters:
            pwm_data = [0 for _ in range(16)]
            # pwm_data[i] = 0.2
            # input("Press Enter to continue...")
            # print(i)
            # controller_node.set_pwm(pwm_data)
            print(f'Pin: {thruster["idx"]}')
            for j in range(9):
                # pwm_data = [j/10 * thruster_coefficients[i] for _ in range(16)]
                pwm_data[thruster['idx']] = j * 0.05 * thruster['down']
                # input("Press Enter to continue...")
                print(f'PWM: {j * 0.05 * thruster["down"]}')
                controller_node.set_pwm(pwm_data)
                time.sleep(5)
                # input("Press Enter to continue...")
            
        # log
        
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt) as error:
        print("Interrupted!")
    finally:
        print("Stopping...")
        controller_node.stop()
        rospy.signal_shutdown("Interrupted or finished") 
