#!/usr/bin/python
import rospy
import roslib

# Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class CmdVelToDiffDriveMotors:
	def __init__(self):
		#Node
		rospy.init_node('diffdrive_controller')
		#Subscribers
		self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
		#Publishers		
		self.l_wheel_tan_vel_target_pub = rospy.Publisher('l_wheel_tan_vel_target', Float32, queue_size = 10)
		self.r_wheel_tan_vel_target_pub = rospy.Publisher('r_wheel_tan_vel_target', Float32, queue_size = 10)

		self.L = rospy.get_param('~robot_wheel_separation_distance', 0.18)
		self.R = rospy.get_param('~robot_wheel_radius', 0.035)

		self.rate = rospy.get_param('~rate', 50)
		self.timeout_idle = rospy.get_param('~timeout_idle', 2)
		self.timeout_idle = rospy.Duration(self.timeout_idle)
		self.time_prev_update = rospy.Time.now()

		self.target_v = 0;
		self.target_w = 0;

	# When not given commands for some specified time, do not move
	def spin(self):
		rospy.loginfo("Start diffdrive_controller")
		rate = rospy.Rate(self.rate)
		time_curr_update = rospy.Time.now()

		rospy.on_shutdown(self.shutdown)

		while not rospy.is_shutdown():
			time_diff_update = (time_curr_update-self.time_prev_update)
			if time_diff_update < self.timeout_idle:
				self.update(); # Only move if command was given recently
			rate.sleep()
		rospy.spin(); #repeat the loop

	def shutdown(self):
		rospy.loginfo("Stop diffdrive_controller")
		self.l_wheel_tan_vel_target_pub.publish(0)
		self.r_wheel_tan_vel_target_pub.publish(0)
		rospy.sleep(1)

	def update(self):
		#################################################
		##Input: target velocity v AND target angular velocity w
		##Relation: robot has wheels with radius R and distance between wheels L
		##Output: wheel velocity for left (vl) and right (vr) wheel
		vr = (2*self.target_v + self.target_w*self.L) / (2)
		vl = (2*self.target_v - self.target_w*self.L) / (2)

		self.r_wheel_tan_vel_target_pub.publish(vr)
		self.l_wheel_tan_vel_target_pub.publish(vl)

	def twistCallback(self, msg):
		self.target_v = msg.linear.x;
		self.target_w = msg.angular.z;
		self.time_prev_update = rospy.Time.now()

def main():
	cmdvel_to_motors = CmdVelToDiffDriveMotors();
	cmdvel_to_motors.spin()

if __name__ == '__main__':
	main();
