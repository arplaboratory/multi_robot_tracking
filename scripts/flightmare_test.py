#! /bin/env python3

import time

import rospy

from geometry_msgs.msg import PoseStamped as ps
from std_msgs.msg import Empty as e
from std_msgs.msg import Bool as bo

class flightmareTest:
    def __init__(self):
        self.num_drones = 3

        self.setpoints = [  [0, 0, 2], [4, 1, 2], [2, -1, 2],
                            [0, 0, 2], [4, -1, 2], [2, 1 ,2],
                            [0, 0, 2], [4, 0, 3], [2, 0, 1], 
                            [0, 0, 2], [4, 0, 1], [2, 0, 3], 
                            [0, 0, 2], [4, 0, 3], [2, 0, 1], 
                            [4, 4, 2], [8, 4, 3], [6, 4, 1]]
        
        self.sleep_times = [14, 10, 10, 10, 10, 10, 10 ,10 ,10 ,10 ,10 ,10 ,10 ,10 ,10]

        self.position_publishers = []
        self.takeoff_publishers = []
        self.land_publishers = []
        self.arm_publishers = []

        self.drone_namespace = "/hummingbird"
        self.position_topic = "/autopilot/pose_command"
        self.takeoff_topic = "/autopilot/start"
        self.land_topic = "/autopilot/land"
        self.arm_topic = "/bridge/arm"

        for i in range(self.num_drones):
            self.position_publishers.append(rospy.Publisher(self.drone_namespace+str(i)+self.position_topic, ps, queue_size=1))
            self.takeoff_publishers.append(rospy.Publisher(self.drone_namespace+str(i)+self.takeoff_topic, e, queue_size=1))
            self.land_publishers.append(rospy.Publisher(self.drone_namespace+str(i)+self.land_topic, e, queue_size=1))
            self.arm_publishers.append(rospy.Publisher(self.drone_namespace+str(i)+self.arm_topic, bo, queue_size=1))
        
        time.sleep(3)

        true_msg = bo()
        true_msg.data = True

        for i in range(self.num_drones):
            self.arm_publishers[i].publish(true_msg)
        
        time.sleep(1)
        for i in range(self.num_drones):
            self.takeoff_publishers[i].publish(e())
        
        time.sleep(2)
        

        for i in range(int(len(self.setpoints)/3)):
            for ii in range(self.num_drones):
                pose_command = ps()
                pose_command.header.stamp = rospy.Time.now()
                pose_command.pose.position.x = self.setpoints[(i*3)+ii][0]
                pose_command.pose.position.y = self.setpoints[(i*3)+ii][1]
                pose_command.pose.position.z = self.setpoints[(i*3)+ii][2]
                self.position_publishers[ii].publish(pose_command)
            print("sent command ", i)
            time.sleep(self.sleep_times[i])
            if(rospy.is_shutdown()):
                break


rospy.init_node("flightmare_controller")
ft = flightmareTest()
