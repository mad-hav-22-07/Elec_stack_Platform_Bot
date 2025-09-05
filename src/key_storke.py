#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys,termios,tty,select

class key_board_node(Node):
    
    def _init_(self):
        super()._init_("keyboard_control")
        self.key_stroke = self.create_publisher(Float32MultiArray,"keystroke",10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.timer = self.create_timer(0.1,self.key_loop)
        self.forward_count = 1
        self.reverse_count = 1
        self.left_velocity = 0
        self.right_velocity = 0
        
    
    def key_loop(self):
        msg = Float32MultiArray()
        msg.data = [0.0, 0.0]

        key = self.getkey()

 
        if key is None:
            return  # no key pressed 

        if key == "a":
            msg.data = [self.left_velocity + 10.0 , self.right_velocity + 10.0]
        elif key == "r":
            msg.data = [10.0, 0.0]
        elif key == "d":
            msg.data = [self.left_velocity - 10.0 , self.right_velocity - 10.0]
        elif key == "l":
            msg.data = [0.0, 10.0]
        elif key == "m":
            msg.data = [10.0,-10.0]
        elif key == "n":
            msg.data = [-10.0,10.0]
        elif key == "s":
            msg.data = [0.0,0.0]
        elif key == "b":
            msg.data = [-10.0,-10.0]
        elif key == "q":
            self.get_logger().info("Quit key pressed. Shutting down...")
            rclpy.shutdown()
            
            return
        else:
            self.get_logger().info(f"Unknown key pressed: {key}")
            return
        
        if(key == "a" or key == "d"):
            self.left_velocity = msg.data[0]
            self.right_velocity = msg.data[1]
        else:
            self.left_velocity = 0
            self.right_velocity = 0
        
        self.key_stroke.publish(msg)
        # only publish if valid key
        self.get_logger().info(f"Sent {msg.data}")
            
    def getkey(self):
        tty.setraw(sys.stdin.fileno())
        rlist,, = select.select([sys.stdin],[],[],0.1)
        key = sys.stdin.read(1) if rlist else None
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,self.settings)
        return key


def main(args=None):
    rclpy.init(args=args)
    node = key_board_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        rclpy.shutdown()

if _name_ =="_main_":
    main()