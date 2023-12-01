import rospy
from std_msgs.msg import Float32MultiArray, Float64, Float32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from ros_bno055.msg import OrientationEuler

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
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
heave_thruster_indices = [2, 3, 5, 7, 8, 9] #, 2, 3, 8, 9, 5, 7]
thruster_indices = [4, 5, 6, 7]
def main():
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        # pwm_num = max(0x1000 * (i % 0x11) - 1, 0)
        # rospy.loginfo('PWM: %s', pwm_num)
        
        
        pwm_data = [0 for i in range(16)]
        controller_node.set_pwm(pwm_data)
        for i in range(10):
            for j in heave_thruster_indices:
                pwm_data[j] = (i+1)/10
            input("Press Enter to continue...")
            print(f'Thrust: {-(i+1)/10}')
            controller_node.set_pwm(pwm_data)
            

        # log
        
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        controller_node.stop()
