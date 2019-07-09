#!/usr/bin/python
import rospy
import roslib

# Messages
from std_msgs.msg import Float32

# Issue commands to the motors to achieve the target velocity
# Use a PID that compares the error based on encoder readings - not yet implemented

class ControlsToMotors:
	def __init__(self):
		rospy.init_node('robot_vel_controller')
		self.rate = rospy.get_param('~rate', 50)

		#Wheels can turn a maximum of xxx rad/s when PWM_rate = 100
		self.motor_max_ang_vel = rospy.get_param('~motor_max_ang_vel', 2)
		#Wheels can turn a minimum of xxx rad/s when PWM_rate = 30
		self.motor_min_ang_vel = rospy.get_param('~motor_min_ang_vel', 0)

		#And their corresponding motor commands
		self.motor_cmd_max = rospy.get_param('~motor_cmd_max', 100)
		self.motor_cmd_min = rospy.get_param('~motor_cmd_min', 10)

		self.R = rospy.get_param('~robot_wheel_radius', 0.035)
		self.debug_on = rospy.get_param('~debug_on', False)
		self.pid_on = rospy.get_param('~pid_on', True)

		#PID setup
		self.Kp = rospy.get_param('~Kp', 2.2)
		self.Ki = rospy.get_param('~Ki', 0.4)
		self.Kd = rospy.get_param('~Kd', 0.2)

		###PUBLISHERS###
		#Publish the computed ang_vel targets
		self.l_wheel_ang_vel_target_pub = rospy.Publisher('l_wheel_ang_vel_target', Float32, queue_size = 10)
		self.r_wheel_ang_vel_target_pub = rospy.Publisher('r_wheel_ang_vel_target', Float32, queue_size = 10)
		#Publish the computed ang_vel control command
		self.l_wheel_ang_vel_control_pub = rospy.Publisher('l_wheel_ang_vel_control', Float32, queue_size = 10)
		self.r_wheel_ang_vel_control_pub = rospy.Publisher('r_wheel_ang_vel_control', Float32, queue_size = 10)
		#Publish the computed angular velocity motor command
		self.l_wheel_ang_vel_motor_pub = rospy.Publisher('l_wheel_ang_vel_motor', Float32, queue_size = 10)
		self.r_wheel_ang_vel_motor_pub = rospy.Publisher('r_wheel_ang_vel_motor', Float32, queue_size = 10)
		#Direction of the wheel: 1=Forwards 0=Backwards
     		self.l_wheel_direction_pub = rospy.Publisher('l_wheel_direction', Float32, queue_size = 10)
      		self.r_wheel_direction_pub = rospy.Publisher('r_wheel_direction', Float32, queue_size = 10)	     
						     
		###SUBSCRIBERS###
		#Read encoders for PID control
		self.l_wheel_ang_vel_enc_sub = rospy.Subscriber('l_wheel_ang_vel_enc', Float32, self.l_wheel_ang_vel_enc_callback)
		self.r_wheel_ang_vel_enc_sub = rospy.Subscriber('r_wheel_ang_vel_enc', Float32, self.r_wheel_ang_vel_enc_callback)
		#Read in tangential velocity targets
		self.l_wheel_tan_vel_target_sub = rospy.Subscriber('l_wheel_tan_vel_target', Float32, self.l_wheel_tan_vel_target_callback)
		self.r_wheel_tan_vel_target_sub = rospy.Subscriber('r_wheel_tan_vel_target', Float32, self.r_wheel_tan_vel_target_callback)

		###DECLARATION###
		#Tangential velocity target
		self.l_wheel_tan_vel_target = 0
		self.r_wheel_tan_vel_target = 0
		#Angular velocity target
		self.l_wheel_ang_vel_target = 0
		self.r_wheel_ang_vel_target = 0
		#Angular velocity encoder readings
		self.l_wheel_ang_vel_enc = 0
		self.r_wheel_ang_vel_enc = 0
		#PID control variables
		self.l_wheel_pid = {}
    		self.r_wheel_pid = {}

	###DEFINE READING OF DATA###
	#Read tangetnial velocity targets
	def l_wheel_tan_vel_target_callback(self, msg):
		self.l_wheel_tan_vel_target = msg.data

	def r_wheel_tan_vel_target_callback(self, msg):
		self.r_wheel_tan_vel_target = msg.data

	#Read encoder readings for PID
	def l_wheel_ang_vel_enc_callback(self, msg):
		self.l_wheel_ang_vel_enc = msg.data

  	def r_wheel_ang_vel_enc_callback(self, msg):
    		self.r_wheel_ang_vel_enc = msg.data

	###UPDATE MOTOR COMMANDS###
	#Compute angular velocity target
	def tangentvel_2_angularvel(self,tangent_vel):
		angular_vel = tangent_vel / self.R;
		return angular_vel
	

	def pid_control(self, wheel_pid, target, state):
	
		if len(wheel_pid) == 0:
			wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0, 'error_curr':0}) #INIT of dictionary
		wheel_pid['time_curr'] = rospy.Time.now()
		wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
		if wheel_pid['dt'] == 0: 
			return 0 #robustness precaution
		wheel_pid['error_curr'] = target - state
		wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])] # integral part
		wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev']) / wheel_pid['dt'] #derivative part
		wheel_pid['error_prev'] = wheel_pid['error_curr']

		#Compute control signal
		control_signal = (self.Kp*wheel_pid['error_curr'] + self.Ki*sum(wheel_pid['integral']) + self.Kd*wheel_pid['derivative'])
		#Set new target
		target_new = target + control_signal

		#Ensure proper sign of the target velocity
		if target > 0 and target_new > 0: target_new = target;
		if target < 0 and target_new < 0: target_new = target;
		if target == 0:
			target_new = 0
			return target_new
		wheel_pid['time_prev'] = wheel_pid['time_curr']
		return target_new
	
	#Mapping ang_vel targets to motor commands
	#motor commands are ints between 0-100
	#we assume motor commands are issued between motor_min_ang_vel AND motor_max_ang_vel
	def angularvel_2_motorcmd(self, angular_vel_target):
		if angular_vel_target == 0: return 0;
		#We approximate the function as a line
		slope = (self.motor_cmd_max - self.motor_cmd_min) / (self.motor_max_ang_vel - self.motor_min_ang_vel)
		intercept = self.motor_cmd_max - slope * self.motor_max_ang_vel

		if angular_vel_target > 0: # positive angular velocity
			#get the right cmd value for positive ang_vel
			motor_cmd = slope * abs(angular_vel_target) + intercept
			if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
			if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min

		else: 			   # negative angular velocity
                        #get the right cmd value for negative ang_vel
	                motor_cmd = slope * abs(angular_vel_target) + intercept
                        if motor_cmd > self.motor_cmd_max: motor_cmd = self.motor_cmd_max
                        if motor_cmd < self.motor_cmd_min: motor_cmd = self.motor_cmd_min
			motor_cmd = -motor_cmd

		return motor_cmd

	###SEND MOTOR COMMAND TO ROBOT###
	#motor1=left wheel, motor1(1,x) means forward, motor1(0,x) means backwards
	#motor2=right wheel
	def motorcmd_2_robot(self, wheel = 'left', motor_command = 0):
		if not self.debug_on:
			motor_command_raw = int(abs(motor_command))
			if wheel == 'left':
				if motor_command >= 0: self.l_wheel_direction_pub.publish(1) #send motor1(1,0)
				elif motor_command < 0: self.l_wheel_direction_pub.publish(0) #send motor1(0,0)
				self.l_wheel_ang_vel_motor_pub.publish(motor_command_raw)
			if wheel == 'right':
                                if motor_command >= 0: self.r_wheel_direction_pub.publish(1) #send motor2(1,0)
                                elif motor_command < 0: self.r_wheel_direction_pub.publish(0) #send motor2(0,0)
				self.r_wheel_ang_vel_motor_pub.publish(motor_command_raw)

	###NEED INTERFACE NODE RPI<->VEL_CONTROLLER###
	###DONT FORGET TO DETERMINE MIN AND MAX VELOCITIES!!!!###

	def l_wheel_update(self):
		#compute ang_vel
		self.l_wheel_ang_vel_target = self.tangentvel_2_angularvel(self.l_wheel_tan_vel_target)
		#publish that value
		self.l_wheel_ang_vel_target_pub.publish(self.l_wheel_ang_vel_target)
		#PID
		if self.pid_on:
			self.l_wheel_ang_vel_target = self.pid_control(self.l_wheel_pid, self.l_wheel_ang_vel_target, self.l_wheel_ang_vel_enc)
		self.l_wheel_ang_vel_control_pub.publish(self.l_wheel_ang_vel_target)
		#compute motor command
		l_wheel_motor_cmd = self.angularvel_2_motorcmd(self.l_wheel_ang_vel_target)
		#send it to the llc_GPIO
		self.motorcmd_2_robot('left',l_wheel_motor_cmd)

	def r_wheel_update(self):
		#compute ang_vel
		self.r_wheel_ang_vel_target = self.tangentvel_2_angularvel(self.r_wheel_tan_vel_target)
		#publish that value
		self.r_wheel_ang_vel_target_pub.publish(self.r_wheel_ang_vel_target)
		#PID
		if self.pid_on:
			self.r_wheel_ang_vel_target = self.pid_control(self.r_wheel_pid, self.r_wheel_ang_vel_target, self.r_wheel_ang_vel_enc)
		self.r_wheel_ang_vel_control_pub.publish(self.r_wheel_ang_vel_target)
		#compute motor command
		r_wheel_motor_cmd = self.angularvel_2_motorcmd(self.r_wheel_ang_vel_target)
		#send it to the llc_GPIO
		self.motorcmd_2_robot('right',r_wheel_motor_cmd)
		
	#The loop with timeout
	def spin(self):
		rospy.loginfo("Start robot_vel_controller")
		rate = rospy.Rate(self.rate)

		rospy.on_shutdown(self.shutdown)

		while not rospy.is_shutdown():
			self.r_wheel_update()
			self.l_wheel_update()
			rate.sleep()
		rospy.spin();

	def shutdown(self):
		rospy.loginfo("Stop robot_vel_controller")
		#Message to stop everything
		self.l_wheel_ang_vel_target_pub.publish(0)
		self.r_wheel_ang_vel_target_pub.publish(0)
		self.l_wheel_ang_vel_control_pub.publish(0)
                self.r_wheel_ang_vel_control_pub.publish(0)
		self.l_wheel_ang_vel_motor_pub.publish(0)
                self.r_wheel_ang_vel_motor_pub.publish(0)
		rospy.sleep(1)

def main():
	controls_to_motors = ControlsToMotors();
	controls_to_motors.spin()

if __name__ == '__main__':
	main();
