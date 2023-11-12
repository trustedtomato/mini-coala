from std_msgs.msg import String
import rospy

def chatter():
    rospy.init_node('chatter', anonymous=True)
    pub = rospy.Publisher('asd', String, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
    print("chatter.py is shutting down")

if __name__ == '__main__':
    try:
        chatter()
    except rospy.ROSInterruptException:
        pass