#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from std_msgs.msg import String, Int64MultiArray
import sys
import numpy

# formation patterns
if sys.argv[3] == '6':
    from formation_dict import formation_dict_6 as formation_dict
elif sys.argv[3] == '9':
    from formation_dict import formation_dict_9 as formation_dict
elif sys.argv[3] == '18':
    from formation_dict import formation_dict_18 as formation_dict
else:
    print("Only 6, 9 and 18 UAVs are supported.")

class Follower:

    def __init__(self, uav_type, uav_id):
        self.hover = "HOVER"
        self.uav_type = uav_type
        self.id = uav_id
        self.f = 30 
        self.local_pose = PoseStamped()
        self.local_pose_queue = [PoseStamped()] * (self.f/10) # time duration = 0.1s
        self.cmd_vel_enu = Twist()
        self.avoid_vel = Vector3()
        self.formation_config = 'waiting'
        self.formation_pattern = formation_dict['origin']
        self.changed_id = numpy.arange(0, int(sys.argv[3])-1)
        self.kp = 1 
        self.vel_max = 1
        self.leader_local_pose = PoseStamped()
        self.wait_cmd = 'HOVER'

        self.local_pose_sub = rospy.Subscriber(self.uav_type+'_'+str(self.id)+"/mavros/local_position/pose", PoseStamped, self.local_pose_callback, queue_size=1)
        self.avoid_vel_sub = rospy.Subscriber("/xtdrone/"+self.uav_type+'_'+str(self.id)+"/avoid_vel", Vector3, self.avoid_vel_callback, queue_size=1)
        self.formation_switch_sub = rospy.Subscriber("/xtdrone/formation_switch",String, self.formation_switch_callback, queue_size=1)
        self.changed_id_sub = rospy.Subscriber("/xtdrone/changed_id", Int64MultiArray, self.changed_id_callback, queue_size=1)

        self.vel_enu_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd_vel_enu', Twist, queue_size=1)
        self.info_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/info', String, queue_size=1)
        self.cmd_pub = rospy.Publisher('/xtdrone/'+self.uav_type+'_'+str(self.id)+'/cmd',String,queue_size=1)
        self.leader_local_pose_sub = rospy.Subscriber(self.uav_type+"_0/mavros/local_position/pose", PoseStamped, self.leader_local_pose_callback, queue_size=1)


    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_pose_queue.pop()
        self.local_pose_queue.append(self.local_pose)

    def leader_local_pose_callback(self, msg):
        self.leader_local_pose = msg 

    def formation_switch_callback(self, msg):
        if not self.formation_config == msg.data:
            self.formation_config = msg.data   
            print("Follower"+str(self.id-1)+": Switch to Formation " + msg.data)
            if not self.formation_config == "waiting":
                self.formation_pattern = formation_dict[self.formation_config]

    def changed_id_callback(self, msg):
        self.changed_id = numpy.array(msg.data)             

    def avoid_vel_callback(self, msg):
        self.avoid_vel = msg

    def loop(self):
        rospy.init_node('follower'+str(self.id-1))
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            if self.formation_config == 'waiting':
                self.cmd_pub.publish(self.wait_cmd)
            else:
                self.cmd_vel_enu.linear.x = self.kp * ((self.leader_local_pose.pose.position.x + self.formation_pattern[0, self.changed_id[self.id - 1]]) - self.local_pose.pose.position.x)
                self.cmd_vel_enu.linear.y = self.kp * ((self.leader_local_pose.pose.position.y + self.formation_pattern[1, self.changed_id[self.id - 1]]) - self.local_pose.pose.position.y) 
                self.cmd_vel_enu.linear.z = self.kp * ((self.leader_local_pose.pose.position.z + self.formation_pattern[2, self.changed_id[self.id - 1]]) - self.local_pose.pose.position.z) 
                self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x + self.vel_max * self.avoid_vel.x
                self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y + self.vel_max * self.avoid_vel.y
                self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z + self.vel_max * self.avoid_vel.z 
                cmd_vel_magnitude = (self.cmd_vel_enu.linear.x**2 + self.cmd_vel_enu.linear.y**2 + self.cmd_vel_enu.linear.z**2)**0.5
                if cmd_vel_magnitude > 3**0.5 * self.vel_max:
                    self.cmd_vel_enu.linear.x = self.cmd_vel_enu.linear.x / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.y = self.cmd_vel_enu.linear.y / cmd_vel_magnitude * self.vel_max
                    self.cmd_vel_enu.linear.z = self.cmd_vel_enu.linear.z / cmd_vel_magnitude * self.vel_max

                self.vel_enu_pub.publish(self.cmd_vel_enu)

            rate.sleep()

if __name__ == '__main__':
    follower = Follower(sys.argv[1],int(sys.argv[2]))
    follower.loop()   