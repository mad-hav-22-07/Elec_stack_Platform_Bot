
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int8


class motor_driver(Node):
    def __init__(self, serial_port):
        super().__init__('motor_driver')

        # mode 0 says that it is running in open loop
        self.mode = "0"
        self.current_vel = 0

        self.ack_error_count = 0
        self.echo_error_count = 0

        self.ack_error_limit = 5
        self.echo_error_limit = 5

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
            
        # checking if port is actually open
        if self.ser.is_open:
            self.get_logger().info("Opened Serial Port")
        else:
            self.get_logger().info("Unable to Open Port")
            rclpy.shutdown()
        
        
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

        # subscribing to the e-stop topic
        self.e_stop_sub = self.create_subscription(
            Int8,
            'e_stop',
            self.e_stop_callback,
            10
        )

        # we implement e-stop if there are more than 5 acknowledgemnt or echo errors
        self.publisher = self.create_publisher(Int8, 'e_stop', 10)

        self.timer = self.create_timer(0.2, self.timer_callback)


    def mcSerialRead(self):
        x = ''
        line = ''

        # this is the command to read bytewise data and stop when it is \r
        while x != b'\r':

            # the bytes are read as b'<char>'
            line += str(x)[2:-1]
            x = self.ser.read()
        
        self.get_logger().info("Read " + str(line) + " from serial")

        return line


    # to set the drivers at open loop mode 0
    def set_mode(self, motor_num):

        x = f'^MMOD {motor_num} {str(self.mode)}\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr in setting mode")
            self.echo_error_count += 1

        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr in setting mode")
            self.ack_error_count += 1


    # this command is to save the updates status in the ROM
    def config(self):

        x = f'%EESAV\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr in setting configuration")
            self.echo_error_count += 1

        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr in setting configuration")
            self.ack_error_count += 1


    def thr_callback(self, msg: Float64MultiArray):
        self.get_logger().info("Reading from topic Thr")
        self.current_vel = msg.data[0]
        #self.send_setpoint(msg.data[0], 1)
        #self.send_setpoint(msg.data[1], 2)


    def send_setpoint(self, setpoint, motor_num):

        # command to give velocity setpoints
        x = f'!G {motor_num} {str(setpoint)}\r'
        self.ser.write(x.encode('utf-8'))

        if (self.mcSerialRead() != x[:-1]):
            self.get_logger().warn("EchoErr in sending setpoint")
            self.echo_error_count += 1
        
        if (self.mcSerialRead() != "+"):
            self.get_logger().warn("AckErr in etting setpoint")
            self.ack_error_count += 1

    
    # keeps sending the velocity setpoint to the wheel non-stop
    def timer_callback(self):
        self.get_logger().info("Setpoint sent : " + str(self.current_vel))
        self.send_setpoint(self.current_vel, 1)

        # implementing e-stop if ack or echo errors exceed limit
        if (self.ack_error_count > self.ack_error_limit) or (self.echo_error_count > self.echo_error_limit):
            msg = Int8
            msg.data = 1
            self.publisher.publish(msg)

    # implementing e-stop
    def e_stop_callback(self, msg: Int8):
        
        # for e-stop
        if msg.data == 1:
            x = f'!EX\r'
            self.get_logger().warn("E-Stop implemented")
            self.ser.write(x.encode('utf-8'))

        # to restart an e-stopped motor
        elif msg.data == 0:
            x = f"!MG\r"
            self.get_logger().info("E-Stop removed")
            self.ser.write(x.encode('utf-8'))

def main(args=None):
    rclpy.init(args=args)
    node = motor_driver(serial_port = '/dev/ttyACM0')        # The serial port goes here
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()