#! /bin/env python3

import rospy

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose


class poseArrayPubber:
    def __init__(self):
        pa = PoseArray()

        paPub = rospy.Publisher("/hummingbird0/track/bounding_box", PoseArray, queue_size=1)
        nSamples = 3
        for i in range(nSamples):
            p = Pose()
            p.position.x = 10*(i+1)
            p.position.y = 10*(i+1)
            pa.poses.append(p)
        
        print(pa)
        
        rate = rospy.Rate(8)
        for i in range(100):
            print(pa)
            paPub.publish(pa)
            rate.sleep()

rospy.init_node("pa_pubber")
pap = poseArrayPubber()

