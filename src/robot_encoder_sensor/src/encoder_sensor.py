#!/usr/bin/python
import rospy
import roslib

# Messages
from std_msgs.msg import Float32

'''
Input:Ticks from encoder
Output:Angular velocity of a wheel
'''
class EncoderSensor:
	def __init__(self):
		#Node
		rospy.init_node('encoder_sensor')
		#Subscribers
		self.l_wheel_ticks_sub = rospy.Subscriber('l_wheel_ticks' Float32, self.l_wheel_ticks_callback)
		self.r_wheel_ticks_sub = rospy.Subscriber('r_wheel_ticks' Float32, self.r_wheel_ticks_callback)
		#Publishers
		self.l_wheel_ang_vel_enc_pub = rospy.Publisher('l_wheel_ang_vel_enc', Float32, queue_size = 10)
		self.r_wheel_ang_vel_enc_pub = rospy.Publisher('r_wheel_ang_vel_enc', Float32, queue_sze = 10)
		#Parameters
		self.Resolution = rospy.get_param('~encoder_resolution', 8)
		self.R = rospy.get_param('~robot_wheel_radius', 0.035)
		self.rate = rospy.get_param('~rate', 50)
		self.distance_per_tick = (3.14159265*2*self.R)/self.Resolution


		self.l_wheel_ang_vel = 0
		self.r_wheel_ang_vel = 0
		self.l_wheel_ticks_time_previous = rospy.Time.now()
		self.r_wheel_ticks_time_previous = rospy.Time.now()

	def l_wheel_ticks_callback(self, msg):
		self.l_wheel_ticks = msg.data
		self.l_wheel_ticks_time = rospy.Time.now()

	def r_wheel_ticks_callback(self, msg):
		self.r_wheel_ticks = msg.data
		self.r_wheel_ticks_time = rospy.Time.now()


	def angularvel(self, delta_ticks = 0, delta_time):
		ang_vel = (delta_ticks*self.distance_per_tick)/delta_time
		return ang_vel

	def encoderticks_2_angularvel(self, wheel = 'left', enc_ticks = 0):
		enc_ticks_raw = int(abs(enc_ticks)
		if enc_ticks_raw == 0: return 0
		elif self.r_wheel_ticks_time-self.r_wheel_ticks_time_previous == 0: return 0
		elif self.l_wheel_ticks_time-self.l_wheel_ticks_time_previous == 0: return 0
		elif wheel == 'right':
			self.r_wheel_delta_ticks = self.r_wheel_ticks-self.r_wheel_ticks_previous #difference of ticks
			self.r_wheel_delta_time = self.r_wheel_ticks_time-self.r_wheel_ticks_time_previous #difference of time
			self.r_wheel_ticks_time_previous = self.r_wheel_ticks_time
			#Compute angular velocity			
			r_wheel_ang_vel_enc = self.angularvel(delta_ticks = self.r_wheel_delta_ticks, delta_time = self.r_wheel_delta_time) 
			
			return r_wheel_ang_vel_enc

		elif wheel == 'left':
			self.l_wheel_delta_ticks = self.l_wheel_ticks-self.l_wheel_ticks_previous #difference of ticks
			self.l_wheel_delta_time = self.l_wheel_ticks_time-self.l_wheel_ticks_time_previous #difference of time
			self.l_wheel_ticks_time_previous = self.l_wheel_ticks_time
			#Compute angular velocity
			l_wheel_ang_vel_enc = self.angularvel(delta_ticks = self.l_wheel_delta_ticks, delta_time = self.l_wheel_delta_time)
			
			return l_wheel_ang_vel_enc

	def spin(self):
		rospy.loginfo("Start encoder_sensor")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)

		while not rospy rospy.is_shutdown():
			self.readdata()
			self.r_wheel_enc_update()
			self.l_wheel_enc_update()
			rate.sleep()
		rospy.spin()


	def shutdown(self):
		rospy.loginfo("Stop encoder_sensor")
		self.l_wheel_ang_vel_enc_pub.publish(0)
		self.r_wheel_ang_vel_enc_pub.publish(0)
		rospy.sleep(1)

	def r_wheel_enc_update(self):
		#converts subsribed ticks to ang_vel
		r_wheel_ang_vel = self.encoderticks_2_angularvel('right',self.r_wheel_ticks)
		self.r_wheel_ang_vel_enc_pub.publish(r_wheel_ang_vel) #publish ang_vel

	def l_wheel_enc_update(self):
		#converts subsribed ticks to ang_vel
		l_wheel_ang_vel = self.encoderticks_2_angularvel('left',self.l_wheel_ticks)
		self.l_wheel_ang_vel_enc_pub.publish(l_wheel_ang_vel) #publish ang_vel

def main():
	encoder_object = EncoderSensor();
	encoder_object.spin()

if __name__ = '__main__':
	main();
	

