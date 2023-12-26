#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

class ShiroJoints():
    def __init__(self):
        
        rospy.init_node('ShiroJoints', anonymous=False)

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

    def sitting(self):
        Joints={'FL_hip_controller':0, 'FL_thigh_controller':0, 'FL_calf_controller':0, 'FR_hip_controller':0, 'FR_thigh_controller':0, 'FR_calf_controller':0, 'RL_hip_controller':0, 'RL_thigh_controller':0, 'RL_calf_controller':0, 'RR_hip_controller':0, 'RR_thigh_controller':0, 'RR_calf_controller':0}
        Joints['FL_hip_controller']     = 0
        Joints['FL_thigh_controller']   = -0.
        Joints['FL_calf_controller']    = 0.2

        Joints['FR_hip_controller']     = 0
        Joints['FR_thigh_controller']   = 0.
        Joints['FR_calf_controller']    = -0.2

        Joints['RL_hip_controller']     = 0
        Joints['RL_thigh_controller']   = 0.
        Joints['RL_calf_controller']    = -0.2

        Joints['RR_hip_controller']     = 0
        Joints['RR_thigh_controller']   = -0.
        Joints['RR_calf_controller']    = 0.2
        
        return Joints
    
    def standing(self):
        Joints={'FL_hip_controller':0, 'FL_thigh_controller':0, 'FL_calf_controller':0, 'FR_hip_controller':0, 'FR_thigh_controller':0, 'FR_calf_controller':0, 'RL_hip_controller':0, 'RL_thigh_controller':0, 'RL_calf_controller':0, 'RR_hip_controller':0, 'RR_thigh_controller':0, 'RR_calf_controller':0}
        Joints['FL_hip_controller']     = 0.1
        Joints['FL_thigh_controller']   = -0.2
        Joints['FL_calf_controller']    = 0.9

        Joints['FR_hip_controller']     = -0.1
        Joints['FR_thigh_controller']   = 0.2
        Joints['FR_calf_controller']    = -0.9

        Joints['RL_hip_controller']     = -0.1
        Joints['RL_thigh_controller']   = 0.2
        Joints['RL_calf_controller']    = -0.9

        Joints['RR_hip_controller']     = 0.1
        Joints['RR_thigh_controller']   = -0.2
        Joints['RR_calf_controller']    = 0.9
        
        return Joints

        ''' FR_hip_controller FR_thigh_controller FR_calf_controller
            FL_hip_controller FL_thigh_controller FL_calf_controller 
            RL_hip_controller RL_thigh_controller RL_calf_controller
            RR_hip_controller RR_thigh_controller RR_calf_controller 

        FR_hip_controller FR_thigh_controller FR_calf_controller
        FL_hip_controller FL_thigh_controller FL_calf_controller
        RL_hip_controller RL_thigh_controller RL_calf_controller
        RR_hip_controller RR_thigh_controller RR_calf_controller     '''
    
    def PublishJoints(self, Joints):
        self.RL_hip_controller_pub.publish(Joints['RL_hip_controller'])
        self.RL_thigh_controller_pub.publish(Joints['RL_thigh_controller'])
        self.RL_calf_controller_pub.publish(Joints['RL_calf_controller'])
        self.FL_hip_controller_pub.publish(Joints['FL_hip_controller'])
        self.FL_thigh_controller_pub.publish(Joints['FL_thigh_controller'])
        self.FL_calf_controller_pub.publish(Joints['FL_calf_controller'])
        self.RR_hip_controller_pub.publish(Joints['RR_hip_controller'])
        self.RR_thigh_controller_pub.publish(Joints['RR_thigh_controller'])
        self.RR_calf_controller_pub.publish(Joints['RR_calf_controller'])
        self.FR_hip_controller_pub.publish(Joints['FR_hip_controller'])
        self.FR_thigh_controller_pub.publish(Joints['FR_thigh_controller'])
        self.FR_calf_controller_pub.publish(Joints['FR_calf_controller'])

if __name__=='__main__':
    flag = 0
    try:
        shiro = ShiroJoints()
        sit = shiro.sitting()
        stand = shiro.standing()
        while not rospy.is_shutdown():
            if flag:
                shiro.PublishJoints(Joints=sit)
                # print('Sit')
                flag = 0
            else:
                shiro.PublishJoints(Joints=stand)
                # print('Stand')
                flag = 1
            rospy.sleep(1)
            shiro.rate.sleep()
    except rospy.ROSInterruptException:
        pass
