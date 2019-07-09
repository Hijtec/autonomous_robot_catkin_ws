#!/usr/bin/python
import rospy
import roslib
import math
import numpy
import tf

# Messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist

#############
#Input: left and right ang_vel
#Output: velocities in linear and angular Vector3
#############

class OdomPublisher:
	def __init__(self):
		rospy.init_node('diffdrive_odom')
		#Subscribers
		self.l_wheel_ang_vel_enc_sub = rospy.Subscriber('l_wheel_ang_vel_enc', Float32, self.l_wheel_ang_vel_enc_callback)
		self.r_wheel_ang_vel_enc_sub = rospy.Subscriber('r_wheel_ang_vel_enc', Float32, self.r_wheel_ang_vel_enc_callback)
		#Publishers
		self.l_wheel_tan_vel_enc_pub = rospy.Publisher('l_wheel_tan_vel_enc', Float32, queue_size = 10)
		self.r_wheel_tan_vel_enc_pub = rospy.Publisher('r_wheel_tan_vel_enc', Float32, queue_size = 10)
		self.cmd_vel_enc_pub = rospy.Publisher('cmd_vel_enc', Twist, queue_size = 10)
		self.odom_pub = rospy.Publisher('odom', Odometry, queue_size = 10)

		#Parameters
		self.L = rospy.get_param('~robot_wheel_separation_distance', 0.18)
		self.R = rospy.get_param('~robot_wheel_radius',0.035)
		self.rate = rospy.get_param('~rate',50)
		self.frame_id = rospy.get_param('~frame_id','/odom')
		self.child_frame_id = rospy.get_param('~','/base_link')
		
		#Initialization
		self.tf_broadcaster = tf.TransformBroadcaster()
		self.l_wheel_ang_vel_enc = 0;
		self.r_wheel_ang_vel_enc = 0;
		self.pose = {'x':0, 'y':0, 'th':0}
		self.time_prev_update = rospy.Time.now();

	def l_wheel_ang_vel_enc_callback(self, msg):
		self.l_wheel_ang_vel_enc = msg.data
	def r_wheel_ang_vel_enc_callback(self, msg):
		self.r_wheel_ang_vel_enc = msg.data

	def angularvel_2_tangentvel(self, ang_vel):
		tan_vel = ang_vel * self.R
		return tan_vel

	#Compute next pose based on current velocities
	def pose_next(self, l_wheel_tan_vel_enc, r_wheel_tan_vel_enc):
		x = self.pose['x'];
		y = self.pose['y'];
		th = self.pose['th'];
		time_curr_update = rospy.Time.now()
		dt = (time_curr_update - self.time_prev_update).to_sec()
		self.time_prev_update = time_curr_update

		#Special case when we move just straight
		if r_wheel_tan_vel_enc == l_wheel_tan_vel_enc:
			v = (l_wheel_tan_vel_enc + r_wheel_tan_vel_enc) / 2.0
			w = 0
			#update pose
			x = x + v*dt*numpy.cos(th)
			y = y + v*dt*numpy.sin(th)
		#Otherwise we compute rotation around 
		#the instantaneous center of curvature
		else:
			v = (l_wheel_tan_vel_enc + r_wheel_tan_vel_enc) / 2.0
			w = (r_wheel_tan_vel_enc - l_wheel_tan_vel_enc) / self.L
			r_icc = (self.L / 2.0) * (l_wheel_tan_vel_enc + r_wheel_tan_vel_enc) / (r_wheel_tan_vel_enc - l_wheel_tan_vel_enc)
 			#update pose using matrixes
			translation = numpy.matrix([[x - r_icc*numpy.sin(th)], [y + r_icc*numpy.cos(th)], [w*dt]])
			icc_pt = numpy.matrix([[r_icc*numpy.sin(th)],[-r_icc*numpy.cos(th)], [th]])
			rotation = numpy.matrix([[numpy.cos(w*dt), -numpy.sin(w*dt), 0],[numpy.sin(w*dt), numpy.cos(w*dt), 0],[0, 0, 1]])

			pose_next = rotation * icc_pt + translation

			x = pose_next[0,0]
			y = pose_next[1,0]
			th = pose_next[2,0]
		return {'x':x, 'y':y, 'th':th, 'v':v, 'w':w}

	def pose_update(self):
		l_wheel_tan_vel_enc = self.angularvel_2_tangentvel(self.l_wheel_ang_vel_enc)
		r_wheel_tan_vel_enc = self.angularvel_2_tangentvel(self.r_wheel_ang_vel_enc)
		self.l_wheel_tan_vel_enc_pub.publish(l_wheel_tan_vel_enc)
		self.r_wheel_tan_vel_enc_pub.publish(r_wheel_tan_vel_enc)
			
		pose_next = self.pose_next(l_wheel_tan_vel_enc, r_wheel_tan_vel_enc)
		cmd_vel_enc = Twist() #initiate class object
		cmd_vel_enc.linear.x = pose_next['v']
		cmd_vel_enc.angular.z = pose_next['w']

		self.cmd_vel_enc_pub.publish(cmd_vel_enc)
		return pose_next

	def pub_odometry(self, pose):
		odom_msg = Odometry() #initiate class object
		odom_msg.header.stamp = self.time_prev_update 	#TimeStamp
		odom_msg.header.frame_id = self.frame_id 	#FrameID
		odom_msg.child_frame_id = self.child_frame_id #ChildFrameID
		odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0) 
		odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
		#P = numpy.mat(numpy.diag([0.0]*3))
		#odom_msg.pose.covariance = tuple(P.ravel().tolist())
		self.odom_pub.publish(odom_msg)

	def pub_tf(self, pose):
		self.tf_broadcaster.sendTransform(\
						(pose['x'], pose['y'], 0), \
						tf.transformations.quaternion_from_euler(0, 0, pose['th']), \
						self.time_prev_update, \
						self.child_frame_id, \
						self.frame_id \
						)
	def update(self):
		self.pose = self.pose_update();
		self.pose['th'] = math.atan2(math.sin(self.pose['th']), math.cos(self.pose['th'])) #Trick to squash the orientation between <-pi,pi>
		self.pub_odometry(self.pose)
		self.pub_tf(self.pose)
	
	def spin(self):
		rospy.loginfo("Start diffdrive_odom")
		rate = rospy.Rate(self.rate)
		rospy.on_shutdown(self.shutdown)
		while not rospy.is_shutdown():
			self.update();
			rate.sleep()
		rospy.spin()

	def shutdown(self):
		rospy.loginfo("Stop diffdrive_odom")
		rospy.sleep(1)

def main():
	odom_publisher = OdomPublisher();
	odom_publisher.spin()

if __name__ == '__main__':
	main();







