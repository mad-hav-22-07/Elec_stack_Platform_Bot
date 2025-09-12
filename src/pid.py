import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray,Bool
import time
import math

class Pid(Node):  
    def __init__(self):
        super().__init__('pid')
        
        self.estop = False
        self.radius = 0.5
        self.separation_distance = 2.0  # distance between wheels
        self.timeout = 15.0   # seconds
        
        self.active ="software"
        self.last_key_time = None
        self.rpm = [0.0, 0.0]
        self.thr = [0.0, 0.0]

        self.subs1 = self.create_subscription(Float32MultiArray,'/cmd_vel_nav',self.cmd_callback,10)
        self.subs2 = self.create_subscription(Float32MultiArray,'rpm',self.rpm_callback,10)
        self.subs3 = self.create_subscription(Float32MultiArray,'keystroke',self.keystroke_callback,10)
        self.subs4 = self.create_subscription(Bool,'estop',self.estop_callback,10)
        self.publisher = self.create_publisher(Float32MultiArray, 'Thr', 10)

         # PID for setpoint (Velocity PID)
        self.pid_velL = output(output_min=-10000, output_max=10000, kp=1.0, ki=0.0, kd=0.0)
        self.pid_velR = output(output_min=-10000, output_max=10000, kp=1.0, ki=0.0, kd=0.0)

        # PID for throttle (Wheel PID)
        self.pidL = output(output_min=-300, output_max=300, kp=1.0, ki=0.0, kd=0.0)
        self.pidR = output(output_min=-300, output_max=300, kp=1.0, ki=0.0, kd=0.0)

        self.timer_1 = self.create_timer(0.1,self.control_loop)  # 0.1 seconds
        self.timer_2 = self.create_timer(0.1,self.check)

    
    def check(self):
        if self.active == "keystroke" and self.last_key_time is not None:
            if time.time() - self.last_key_time > self.timeout:
                self.get_logger().info("Keystroke deactivated")
                self.active = "software"

    def estop_callback(self,msg):
        self.estop = msg.data 

        # Reset all PID controls
        self.pidL.reset()
        self.pidR.reset()
        self.pid_velL.reset()
        self.pid_velR.reset()
        self.thr = [0.0, 0.0]

        thr_msg = Float32MultiArray()
        thr_msg.data = self.thr
        self.publisher.publish(thr_msg)
        
    def cmd_callback(self, msg):
        vl=(msg.data[0]-self.separation_distance/2*msg.data[1])*60 / (2 * math.pi * self.radius)  # required rpm for left   msg.data[0]--linear velocity   msg.data[1]--angular velocity
        vr=(msg.data[0]+self.separation_distance/2*msg.data[1])*60 / (2 * math.pi * self.radius)  # for right
        self.pid_velL.set_setpoint(vl)
        self.pid_velR.set_setpoint(vr)
        self.pid_velL.update(self.pid_velL.output())  
        self.pid_velR.update(self.pid_velR.output())  
        self.pidL.set_setpoint(self.pid_velL.output())
        self.pidR.set_setpoint(self.pid_velR.output())

    def keystroke_callback(self,msg):
        if self.active != "keystroke":
            self.pidL.reset()
            self.pidR.reset()
            self.pid_velL.reset()
            self.pid_velR.reset()    
        self.active = "keystroke"
        self.last_key_time = time.time()

        vl =  msg.data[0]
        vr = msg.data[1]
        self.pid_velL.set_setpoint(vl)
        self.pid_velR.set_setpoint(vr)
        self.pid_velL.update(self.pid_velL.output())  
        self.pid_velR.update(self.pid_velR.output())  
        self.pidL.set_setpoint(self.pid_velL.output())
        self.pidR.set_setpoint(self.pid_velR.output())
        
    def rpm_callback(self, msg):
        if self.estop:
            return    # To pause it during E-Stop
        self.rpm = msg.data
        
    def control_loop(self):      
            if not self.estop: 
                self.pidL.update(self.rpm[0])
                self.pidR.update(self.rpm[1])
                self.thr[0] = self.pidL.output()
                self.thr[1] = self.pidR.output()
                thr_msg = Float32MultiArray()
                thr_msg.data = self.thr
                self.publisher.publish(thr_msg)
                self.get_logger().info(f'Published: {self.thr}')
            


class output:
    def __init__(self, output_min, output_max, kp=1.0, ki=0.0, kd=0.0):
        # PID gains
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Output limits
        self.output_min = output_min
        self.output_max = output_max

        # Internal state
        self._setpoint = 0.0
        self._prev_error = 0.0
        self._integral = 0.0
        self._output = 0.0

    def reset(self):
        self._prev_error = 0.0
        self._integral = 0.0

    def set_setpoint(self, value):
        self._setpoint = value

    def update(self, measurement):
        error = self._setpoint - measurement
        self._integral += error # if dt constant can be ignored
        derivative = (error - self._prev_error) 

        # PID formula
        self._output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Appling limits 
        if self._output > self.output_max:
            self._output = self.output_max
        if self._output < self.output_min:
            self._output = self.output_min

        if self.ki != 0:
            max_integral = self.output_max / self.ki
            min_integral = self.output_min / self.ki
            self._integral = max(min(self._integral, max_integral), min_integral)

        # Save state
        self._prev_error = error
        return self._output
    def output(self):
        return self._output
    


def main(args=None):
    rclpy.init(args=args)
    node = Pid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()