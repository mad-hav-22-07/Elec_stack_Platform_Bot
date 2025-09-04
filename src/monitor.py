import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from matplotlib import pyplot as plt
import numpy as np
#monitor Code

#displays the throttle value, set point value and the encoder value.

class myNode(Node):

    def __init__ (self):
        super().__init__("monitor")
        self.subscriber_object = self.create_subscription(Float32MultiArray,"/monitor",self.callback_function,10)
        plt.ion()
        
        #Setting the starting array to be 0s
        self.throttle_values = np.zeros(10)
        self.encoder_values = np.zeros(10)
        self.setpoint_values = np.zeros(10)
        
        # Creating a matpoltlib fig
        self.figure,self.axes = plt.subplots()
        self.throttle_line, = self.axes.plot(self.throttle_values,color = "red",marker ="o",label = "throttle")
        self.encoder_line, = self.axes.plot(self.encoder_values,color = "blue",marker ="o", label ="encoder")
        self.setpoint_line, = self.axes.plot(self.setpoint_values,color = "green",marker ="o",label ="setpoint")
        self.axes.set_title("Plot")
        self.axes.set_ylim(-100,100)
        self.axes.legend()




    def callback_function(self, msg : Float32MultiArray):
        left_wheel_throttle = msg.data[4]
        left_wheel_encoder_value = msg.data[2]
        left_wheel_setpoint = msg.data[0]

        self.throttle_values = np.roll(self.throttle_values,-1)
        self.throttle_values[-1] = left_wheel_throttle
        self.throttle_line.set_ydata(self.throttle_values)

        self.encoder_values = np.roll(self.encoder_values,-1)
        self.encoder_values[-1]=left_wheel_encoder_value
        self.encoder_line.set_ydata(self.encoder_values)

        self.setpoint_values = np.roll(self.setpoint_values,-1)
        self.setpoint_values[-1] = left_wheel_setpoint
        self.setpoint_line.set_ydata(self.setpoint_values)

        self.axes.set_xlim(0, len(self.encoder_values)-1)
        self.figure.canvas.draw()
        self.figure.canvas.flush_events()
        
        
def main(args = None):
    rclpy.init(args=args)
    node = myNode()
    rclpy.spin(node)
    rclpy.shutdown()