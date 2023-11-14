import rospy
import rosbag
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import Imu
from ros_bno055.msg import OrientationEuler

rospy.init_node('logger_node', anonymous=True)

with rosbag.Bag('outbag.bag', 'w') as outbag:
    def log_motor(msg):
        outbag.write('motor_cmd', msg)
    def log_heave(msg):
        outbag.write('heave_data', msg)
    def log_joystick(msg):
        outbag.write('joystick', msg)
    def log_imu(msg):
        outbag.write('imu/data', msg)
    def log_imu_euler(msg):
        outbag.write('imu/orientation_euler', msg)

    rospy.Subscriber("motor_cmd", Float32MultiArray, log_motor)
    rospy.Subscriber("heave_data", Float32, log_heave)
    rospy.Subscriber("joystick", Float32MultiArray, log_joystick)
    rospy.Subscriber("imu/data", Imu, log_imu)
    rospy.Subscriber("imu/orientation_euler", OrientationEuler, log_imu_euler)
    rospy.spin()
    