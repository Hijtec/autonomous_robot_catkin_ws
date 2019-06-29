#!/usr/bin/python
import rospy
import roslib
import RPi.GPIO as GPIO
import time

#Messages
from std_msgs.msg import Float32

#Init
rospy.init_node('llc_encoder')
#Publishers
l_wheel_ticks_pub = rospy.Publisher('l_wheel_ticks',Float32, queue_size = 10)
r_wheel_ticks_pub = rospy.Publisher('r_wheel_ticks',Float32, queue_size = 10)

#Setup
GPIO.setmode(GPIO.BCM)
left = 4
right = 17
GPIO.setup(left, GPIO.IN)
GPIO.setup(right, GPIO.IN)

while 1:
	l_wheel_ticks_pub.publish(GPIO.input(left))
	r_wheel_ticks_pub.publish(GPIO.input(right))

