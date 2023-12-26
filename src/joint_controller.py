#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class JointsPublisher():
    def __init__(self):
        
        # rospy.init_node('ShiroJointsController', anonymous=False)

        #Publishers
        self.FL_hip_controller_pub = rospy.Publisher('/shiro/FL_hip_controller/command', Float64, queue_size=10)
        self.FL_thigh_controller_pub = rospy.Publisher('/shiro/FL_thigh_controller/command', Float64, queue_size=10)
        self.FL_calf_controller_pub = rospy.Publisher('/shiro/FL_calf_controller/command', Float64, queue_size=10)

        self.FR_hip_controller_pub = rospy.Publisher('/shiro/FR_hip_controller/command', Float64, queue_size=10)
        self.FR_thigh_controller_pub = rospy.Publisher('/shiro/FR_thigh_controller/command', Float64, queue_size=10)
        self.FR_calf_controller_pub = rospy.Publisher('/shiro/FR_calf_controller/command', Float64, queue_size=10)

        self.RL_hip_controller_pub = rospy.Publisher('/shiro/RL_hip_controller/command', Float64, queue_size=10)
        self.RL_thigh_controller_pub = rospy.Publisher('/shiro/RL_thigh_controller/command', Float64, queue_size=10)
        self.RL_calf_controller_pub = rospy.Publisher('/shiro/RL_calf_controller/command', Float64, queue_size=10)

        self.RR_hip_controller_pub = rospy.Publisher('/shiro/RR_hip_controller/command', Float64, queue_size=10)
        self.RR_thigh_controller_pub = rospy.Publisher('/shiro/RR_thigh_controller/command', Float64, queue_size=10)
        self.RR_calf_controller_pub = rospy.Publisher('/shiro/RR_calf_controller/command', Float64, queue_size=10)
        
        #Rate
        self.rate = rospy.Rate(50) # 1000hz

        ''' FR_hip_controller FR_thigh_controller FR_calf_controller
            FL_hip_controller FL_thigh_controller FL_calf_controller 
            RL_hip_controller RL_thigh_controller RL_calf_controller
            RR_hip_controller RR_thigh_controller RR_calf_controller 

        FR_hip_controller FR_thigh_controller FR_calf_controller
        FL_hip_controller FL_thigh_controller FL_calf_controller
        RL_hip_controller RL_thigh_controller RL_calf_controller
        RR_hip_controller RR_thigh_controller RR_calf_controller     '''
    
    def PublishJoints(self, Joints):
        self.FL_hip_controller_pub.publish(Joints[0][0])
        self.FL_thigh_controller_pub.publish(Joints[0][1])
        self.FL_calf_controller_pub.publish(Joints[0][2])
        
        self.FR_hip_controller_pub.publish(Joints[0][3])
        self.FR_thigh_controller_pub.publish(Joints[0][4])
        self.FR_calf_controller_pub.publish(Joints[0][5])
        
        self.RL_hip_controller_pub.publish(Joints[0][6])
        self.RL_thigh_controller_pub.publish(Joints[0][7])
        self.RL_calf_controller_pub.publish(Joints[0][8])
        
        # self.FR_hip_controller_pub.publish(Joints[0][6])
        # self.FR_thigh_controller_pub.publish(Joints[0][7])
        # self.FR_calf_controller_pub.publish(Joints[0][8])
        
        # self.RL_hip_controller_pub.publish(Joints[0][3])
        # self.RL_thigh_controller_pub.publish(Joints[0][4])
        # self.RL_calf_controller_pub.publish(Joints[0][5])
        
        self.RR_hip_controller_pub.publish(Joints[0][9])
        self.RR_thigh_controller_pub.publish(Joints[0][10])
        self.RR_calf_controller_pub.publish(Joints[0][11])
