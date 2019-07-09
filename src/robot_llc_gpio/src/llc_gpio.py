#!/usr/bin/python
import rospy
import roslib
import RPi.GPIO as GPIO

# Messages
from std_msgs.msg import Float32

class LLCGPIO:
	def __init__(self):
		rospy.init_node('llc_gpio')
		self.rate = rospy.get_param('~rate', 50)
		#Parameters
		self.l_enable_motor = rospy.get_param('~l_enable_motor', 18) #en2
		self.l_input2_motor = rospy.get_param('~l_input1_motor', 27) #in3
		self.l_input1_motor = rospy.get_param('~l_input2_motor', 22) #in4
		self.r_enable_motor = rospy.get_param('~r_enable_motor', 25) #en1
		self.r_input1_motor = rospy.get_param('~r_input1_motor', 24) #in1
		self.r_input2_motor = rospy.get_param('~r_input2_motor', 23) #in2

		#GPIO Setup
		GPIO.setmode(GPIO.BCM)
		GPIO.setup(self.l_enable_motor,GPIO.OUT)
		GPIO.setup(self.l_input2_motor,GPIO.OUT)
		GPIO.setup(self.l_input1_motor,GPIO.OUT)
		GPIO.setup(self.r_enable_motor,GPIO.OUT)
		GPIO.setup(self.r_input1_motor,GPIO.OUT)
		GPIO.setup(self.r_input2_motor,GPIO.OUT)

		#Initial states - motor is stopped
		GPIO.output(self.l_input2_motor,GPIO.LOW)
		GPIO.output(self.l_input1_motor,GPIO.LOW)
		GPIO.output(self.r_input1_motor,GPIO.LOW)
		GPIO.output(self.r_input2_motor,GPIO.LOW)

		#PWM Configuration
		self.l_pwm = GPIO.PWM(self.l_enable_motor, 1000)
		self.r_pwm = GPIO.PWM(self.r_enable_motor, 1000)
		self.l_pwm.start(10)
		self.r_pwm.start(10)

		#Subscribers: PWM_command,Direction
		self.l_wheel_ang_vel_motor_sub = rospy.Subscriber('l_wheel_ang_vel_motor', Float32, self.l_wheel_ang_vel_motor_callback)
		self.r_wheel_ang_vel_motor_sub = rospy.Subscriber('r_wheel_ang_vel_motor', Float32, self.r_wheel_ang_vel_motor_callback)
		self.l_wheel_direction_sub = rospy.Subscriber('l_wheel_direction', Float32, self.l_wheel_direction_callback)
		self.r_wheel_direction_sub = rospy.Subscriber('r_wheel_direction', Float32, self.r_wheel_direction_callback)
     		
		#Initialization
		self.l_wheel_ang_vel_motor = 0
		self.r_wheel_ang_vel_motor = 0
		self.l_wheel_direction = 1
		self.r_wheel_direction = 1


  ###DEFINE READING OF DATA###
	#Read cmd commands
	def l_wheel_ang_vel_motor_callback(self, msg):
		self.l_wheel_ang_vel_motor = msg.data
	def r_wheel_ang_vel_motor_callback(self, msg):
    		self.r_wheel_ang_vel_motor = msg.data
 	 #Read direction commands
	def l_wheel_direction_callback(self, msg):
    		self.l_wheel_direction = msg.data
  	def r_wheel_direction_callback(self, msg):
    		self.r_wheel_direction = msg.data
  
  	#Motor command to PWM update
  	def angvelmotor_2_setPWM(self,wheel = 'left'):
    		if wheel == 'left':
    			self.l_pwm.ChangeDutyCycle(int(self.l_wheel_ang_vel_motor))
    		if wheel == 'right':
      			self.r_pwm.ChangeDutyCycle(int(self.r_wheel_ang_vel_motor))
  
  	#Motor pins config
  	def setpins(self, direction, wheel = 'left') :
		direction = int(direction)
    		if wheel == 'left' and direction == 1:
      			GPIO.output(self.l_input1_motor, GPIO.HIGH)
      			GPIO.output(self.l_input2_motor, GPIO.LOW)
    		if wheel == 'left' and direction == 0:
      			GPIO.output(self.l_input1_motor, GPIO.LOW)
      			GPIO.output(self.l_input2_motor, GPIO.HIGH)
    
   		if wheel == 'right' and direction == 1:
      			GPIO.output(self.r_input1_motor, GPIO.HIGH)
      			GPIO.output(self.r_input2_motor, GPIO.LOW)
    		if wheel == 'right' and direction == 0:
     			GPIO.output(self.r_input1_motor, GPIO.LOW)
    		  	GPIO.output(self.r_input2_motor, GPIO.HIGH)
    
  	def l_wheel_update(self):
    		self.angvelmotor_2_setPWM('left')
    		self.setpins(direction = self.l_wheel_direction, wheel = 'left')
      
  	def r_wheel_update(self):
    		self.angvelmotor_2_setPWM('right')
    		self.setpins(direction = self.r_wheel_direction, wheel = 'right')
    
  	def spin(self):
    		rospy.loginfo("Start llc_gpio")
    		rate = rospy.Rate(self.rate)
    		
    		while not rospy.is_shutdown():
      			self.r_wheel_update()
      			self.l_wheel_update()
      			rate.sleep()
		rospy.on_shutdown(self.shutdown)
  		rospy.spin();
    
	def shutdown(self):
    		rospy.loginfo("Stop llc_gpio")
    		GPIO.cleanup()
    		rospy.loginfo("GPIO cleanup complete")
    		rospy.sleep(1)
      
def main():
	llcgpio = LLCGPIO();
	llcgpio.spin()

if __name__ == '__main__':
	main();
   
