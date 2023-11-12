from std_msgs.msg import Float32MultiArray
import rospy

def main():
    rospy.init_node('pca9685_publisher', anonymous=True)
    pub = rospy.Publisher('motor_cmd', Float32MultiArray, queue_size=1)
    # imu_sub = rospy.Subscriber('imu', Imu, imu_callback)
    rate = rospy.Rate(0.2)
    i = 0
    while not rospy.is_shutdown():
        pwm = Float32MultiArray()
        # pwm_num = max(0x1000 * (i % 0x11) - 1, 0)
        # rospy.loginfo('PWM: %s', pwm_num)
        pwm.data = [
            0.5,
            # the other 15 channels are 0
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
        ]
        pub.publish(pwm)
        # log
        rospy.loginfo('Publishing: %s', pwm.data)
        rate.sleep()
        i += 1

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass