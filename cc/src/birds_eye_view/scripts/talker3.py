#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def callback(data):
    global pub
    rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.pose)
    pub.Publish()
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('fake_lo', anonymous=True)

    rospy.Subscriber("odom", PoseStamped, callback)

    rospy.spin()

if __name__ == '__main__':
    pub = rospy.Publisher('base_pose_ground_truth', Odometry, queue_size=10)
    listener()