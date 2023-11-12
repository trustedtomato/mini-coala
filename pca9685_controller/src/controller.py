import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3

class ControllerNode:
    def __init__(self):
        rospy.init_node('controller_node', anonymous=True)
        self.pub = rospy.Publisher('motor_cmd', Float32MultiArray, queue_size=1)
        rospy.Subscriber('joystick', Float32MultiArray, self.joystick_callback)
        rospy.Subscriber('imu/data', Imu, self.imu_callback)
        rospy.Subscriber('pressure_sensor', Imu, self.imu_callback)
        self.rate = rospy.Rate(10) # 10hz
        self.msg = Float32MultiArray()
        self.msg.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.yaw_setpoint = 0
        self.heave_setpoint = 0
        self.orientation = Quaternion()
        self.angular_velocity = Vector3()
        self.linear_acceleration = Vector3()

    def joystick_callback(self, msg):
        self.yaw_setpoint = msg.data[0]
        self.heave_setpoint = msg.data[1]

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration

    def stop(self):
        self.msg.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        self.pub.publish(self.msg)

    def set_pwm(self, data):
        self.msg.data = data
        self.pub.publish(self.msg)

def main():
    controller_node = ControllerNode()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        # pwm_num = max(0x1000 * (i % 0x11) - 1, 0)
        # rospy.loginfo('PWM: %s', pwm_num)
        pwm_data = [0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        controller_node.set_pwm(pwm_data)
        # log
        rospy.loginfo('Publishing: %s', pwm_data)
        rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass