#!/usr/bin/env python
# Written by Songyan XIN <xinsongyan@gmail.com>, July 2017.


import sys
import rospy
from trajectory_utils.msg import segmentTrj
from ADVR_ROS.srv import advr_segment_control

class CartesianTrjParser2:
    def __init__(self, topic_name, service_name):
        self.node = rospy.init_node('CartesianTrjParser2', anonymous=True)
        self.sub = rospy.Subscriber(topic_name, segmentTrj, self.callback)
        # rospy.wait_for_service(service_name)
        self.srv_client = rospy.ServiceProxy(service_name, advr_segment_control)
        rospy.spin()

    def callback(self, topic_msg):
        print "-"*10, "CartesianTrj Received!", "-"*10
        print topic_msg
        respond = self.srv_client(topic_msg)
        print "srv respond: ", respond
        print "-"*10, "CartesianTrj Parsed!", "-"*10


if __name__ == '__main__':
    print "CartesianTrjParser2 node running..."
    if len(sys.argv) < 2:
        print "Please specify topic name and service name! \n  Example: ./cartesianTrjParser2.py /segments /segment_control"
        # print 'Num of args:', len(sys.argv)
        # print 'Args List:', str(sys.argv)
    else:
        topic_name = sys.argv[1]  # /LSoftHandLink_segments
        service_name = sys.argv[2]  # /segment_control
        parser = CartesianTrjParser2(topic_name, service_name)
        while not rospy.is_shutdown():
            pass



