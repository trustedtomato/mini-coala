import rospy
import rosbag
from std_msgs.msg import Float32, Float32MultiArray
from ros_bno055.msg import OrientationEuler
import sys
import datetime

def main():
    time = datetime.datetime.now().strftime('%b-%d_%H:%M:%S')
    if len(sys.argv) < 2:
        print('Usage: python logger.py <file_name>')
        return
    file_name = sys.argv[1]
    rospy.init_node('logger_node', anonymous=True)
    with rosbag.Bag(f'{file_name}_{time}.bag', 'w') as bag:
        rospy.Subscriber('heave_data', Float32, lambda msg: bag.write('heave_data', msg))
        rospy.Subscriber('imu/orientation_euler', OrientationEuler, lambda msg: bag.write('imu/orientation_euler', msg))
        rospy.Subscriber('motor_cmd', Float32MultiArray, lambda msg: bag.write('motor_cmd', msg))
        rospy.spin()
if __name__ == '__main__':
    main()