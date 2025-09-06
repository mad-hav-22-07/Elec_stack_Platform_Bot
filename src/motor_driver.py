
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64MultiArray


class motor_driver(Node):
    def __init__(self, serial_port):
        super().__init__('motor_driver')

        # mode 0 says that it is running in open loop
        self.mode = "0"

        self.serial_port = serial_port

        # opening the serial port
        try:
            self.ser = serial.Serial(
                port = self.serial_port,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )    
            self.ser.isOpen()
            print("Opened port", self.ser, "!")
            self.get_logger().info("Opened Serial Port")

        # if the port is occupied and didnt open, close it and open it again
        except IOError:
            self.ser = serial.Serial(
                port = self.serial_port,
                baudrate = 115200,
                parity = "N",
                stopbits = 1,
                bytesize = 8,
                timeout = 0.1
            )
            print("Waiting for port to close")
            self.ser.close()
            self.ser.open()
            print("Opened port", self.ser, "!")
            self.get_logger().info("Opened Serial Port")
        
        
        self.set_mode(1)
        #self.set_mode(2)
        self.config()
        
        # the throttle values come in as an array of two integers
        self.thr_sub = self.create_subscription(
            Float64MultiArray,
            'Thr',
            self.thr_callback,
            10
        )


    def mcSerialRead(self):
        x = ''
        line = ''

        # this is the command to read bytewise data and stop when it is \r
        while x != b'\r':

            # the bytes are read as b'<char>'
            line += str(x)[2:-1]
            x = self.ser.read()

        return line


    # to set the drivers at open loop mode 0
    def set_mode(self, motor_num):

        #command to set mode of driver
        x = f'^MMOD {motor_num} {str(self.mode)}\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr")

        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr")


    # this command is to save the updates status in the ROM
    def config(self):

        x = f'%EESAV\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr")

        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr")



    def thr_callback(self, msg: Float64MultiArray):
        self.send_setpoint(msg.data[0], 1)
        #self.send_setpoint(msg.data[1], 2)


    def send_setpoint(self, setpoint, motor_num):

        # command to give velocity setpoints
        x = f'!G {motor_num} {str(setpoint)}\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr")
        
        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr")

def main(args=None):
    rclpy.init(args=args)
    node = motor_driver(serial_port = '/dev/ttyACM0')        # The serial port goes here
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()