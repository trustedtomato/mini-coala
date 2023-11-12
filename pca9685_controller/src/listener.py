from std_msgs.msg import String
import rospy

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("asd", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
