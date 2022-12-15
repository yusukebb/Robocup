#! /usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/mark_fbj/diff_drive_controller/cmd_vel', Twist, queue_size = 1) #Twist型で'cmd_vel'というトピックを生成

rospy.init_node('Move!_Tokimo_aa!')

move = Twist() 
move.linear.x = 0.5

rate = rospy.Rate(10)


while not rospy.is_shutdown():
    
    cmd_vel_pub.publish(move)
    
    rate.sleep()
