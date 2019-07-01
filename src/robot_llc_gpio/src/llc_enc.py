#!/usr/bin/python
import rospy
import roslib
import RPi.GPIO as GPIO
import time

#Messages
from std_msgs.msg import Float32

class LLCencoder:
	def __init__(self):
		rospy.init_node('llc_encoder')
		#Publishers
		self.l_wheel_ticks_pub = rospy.Publisher('l_wheel_ticks',Float32, queue_size = 10)
		self.r_wheel_ticks_pub = rospy.Publisher('r_wheel_ticks',Float32, queue_size = 10)
		#Setup
		self.rate=50
		self.l_wheel_enc_pin = 4
		self.r_wheel_enc_pin = 17
		#GPIO
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.l_wheel_enc_pin, GPIO.IN)
		GPIO.setup(self.r_wheel_enc_pin, GPIO.IN)
		#Initialiazation
		
		self.l_wheel_ticks_count = 0
		self.r_wheel_ticks_count = 0
		self.l_wheel_state_previous = GPIO.input(self.l_wheel_enc_pin)
		self.r_wheel_state_previous = GPIO.input(self.r_wheel_enc_pin)
		
		
	def change_state(self,state,previous):
		if state != previous:
			change = 1
			previous = state
			return change, previous
			time.sleep(0.001)
		else:
			change = 0
			return change, previous
	
	def tick_count(self,state,change,previous,count_ticks):
		if int(change) == 1 and int(previous) == 1:
			count_ticks=count_ticks+1
			print(count_ticks)
		return count_ticks
	
	def spin(self):
		rospy.loginfo("Start LLC encoder")
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			#LEFT ENCODER
			self.l_wheel_state = GPIO.input(self.l_wheel_enc_pin)
			self.l_wheel_change, self.l_wheel_state_previous = self.change_state(self.l_wheel_state, self.l_wheel_state_previous)
			self.l_wheel_ticks_count = self.tick_count(self.l_wheel_state, self.l_wheel_change, self.l_wheel_state_previous, self.l_wheel_ticks_count)
			#RIGHT ENCODER
			self.r_wheel_state = GPIO.input(self.r_wheel_enc_pin)
			self.r_wheel_change, self.r_wheel_state_previous = self.change_state(self.r_wheel_state, self.r_wheel_state_previous)
			self.r_wheel_ticks_count = self.tick_count(self.r_wheel_state, self.r_wheel_change, self.r_wheel_state_previous, self.r_wheel_ticks_count)
			#PUBLISHING
			self.publish()
		rospy.spin();
			
	def shutdown(self):
		rospy.loginfo("Stop LLC encoder")
		self.l_wheel_ticks_pub.publish(0)
		self.r_wheel_ticks_pub.publish(0)
		rospy.sleep(1)
		
	def publish(self):
		self.l_wheel_ticks_pub.publish(self.l_wheel_state )
		self.r_wheel_ticks_pub.publish(self.r_wheel_ticks_count )
		
def main():
	encoder = LLCencoder();
	encoder.spin()
	
if __name__ =='__main__':
	main();

