import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Bool
import time
import math

class Pid(Node):  
	def __init__(self):  
		super().__init__('pid') 
		
		self.kp = 1.0
		self.ki = 0.0
		self.kd = 0.0
		
		self.prev_time = self.get_clock().now()
		self.prev_err = [0.0, 0.0]   
		self.integral = [0.0, 0.0] 
		self.estop = False
		self.radius = 0.5
		self.separation_distance = 2.0  # distance between wheels

		self.v = [0.0, 0.0]   
		self.rpm = [0.0, 0.0]
		
		self.subs1 = self.create_subscription(Float32MultiArray,'/cmd_vel_nav',self.cmd_callback,10)
		self.subs2 = self.create_subscription(Float32MultiArray,'rpm',self.rpm_callback,10)
		self.subs3 = self.create_subscription(Float32MultiArray,'keystroke',self.keystroke_callback,10)
		self.subs4 = self.create_subscription(Bool,'estop',self.estop_callback,10)
		self.publisher = self.create_publisher(Float32MultiArray, 'Thr', 10)
        
	def estop_callback(self,msg):
		self.estop = msg.data
		if self.estop:
			self.prev_err = [0.0, 0.0]   
			self.integral = [0.0, 0.0] 
			output = [0.0, 0.0]
			 
			out_msg = Float32MultiArray()
			out_msg.data = output
			self.publisher.publish(out_msg)
			self.get_logger().info(f'Published: {output}')

	def cmd_callback(self, msg): 
		# limiting linear velocity value
		self.s = msg.data
		if self.s[0] > 2.25:
			self.s[0] = 2.25
		if self.s[0] < -2.25:
			self.s[0] = -2.25
		self.v[0] = (self.s[0]-self.separation_distance/2*self.s[1])*60 / (2 * math.pi * self.radius)  # required rpm for left   msg.data[0]--linear velocity   msg.data[1]--angular velocity
		self.v[1] = (self.s[0]+self.separation_distance/2*self.s[1])*60 / (2 * math.pi * self.radius)  # for right
    
	def keystroke_callback(self,msg):
		self.prev_err = [0.0, 0.0]   
		self.integral = [0.0, 0.0] 
		self.cmd_callback(msg)

	def rpm_callback(self, msg):
		if self.estop:
			return      # To pause it during E-Stop 
		self.rpm = msg.data
		current_time = self.get_clock().now()
		dt = (current_time - self.prev_time).nanoseconds * 1e-9
		self.prev_time = current_time

		if dt == 0:
		 	return	# Avoid division by zero
		
		output=[0.0,0.0]

		for i in range(2):
		    error = self.v[i] - self.rpm[i]
		    self.integral[i] += error * dt
		    derivative = (error - self.prev_err[i]) / dt
		        
		    output[i] = self.kp * error + self.ki * self.integral[i] + self.kd * derivative

		    self.prev_err[i] = error
		    self.get_logger().info(f"Motor {i+1}: Error={error:.3f}, Output={output[i]:.3f}")
		    
		
		out_msg = Float32MultiArray()
		out_msg.data = output
		self.publisher.publish(out_msg)
		self.get_logger().info(f'Published: {output}')

        
        
def main(args=None):
    rclpy.init(args=args)
    node = Pid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()
