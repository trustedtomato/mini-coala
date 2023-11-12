from std_msgs.msg import Float32MultiArray
import rospy
from ms5837 import MS5837_30BA

def main():
    rospy.init_node('ms5837_publisher', anonymous=True)
    pub = rospy.Publisher('pressure_sensor', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(0.2)
    ms5837 = MS5837_30BA()
    if not ms5837.init():
        rospy.logerr('Failed to initialize MS5837 sensor')
        return
    if not ms5837.read():
        rospy.logerr('Failed to read MS5837 sensor')
        return
    while not rospy.is_shutdown():
        ms5837.read()
        ms5837_data = Float32MultiArray()
        ms5837_data.data = [
            ms5837.pressure(),
            ms5837.temperature(),
            ms5837.depth(),
            ms5837.altitude()
        ]
        pub.publish(ms5837_data)
        # log
        rospy.loginfo('Publishing: %s', ms5837_data.data)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass