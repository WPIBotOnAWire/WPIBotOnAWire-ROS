#!/usr/bin/env python3
import serial
import rospy
import struct
from rospy.impl.init import RosStreamHandler
import rosserial_python
import std_msgs.msg

def send_packet(code: int, data: bytes):
    # Packet format:
    # /------------------------------------------------------------\
    # | CODE (sizeof(uint32)) |  LEN (sizeof(uint32)) | DATA (LEN) |
    # \------------------------------------------------------------/

    data = struct.pack("II%ss"%len(data), code, len(data), data)
    arduino.write(data)

def motor_callback(speed: std_msgs.msg.Int32):
    send_packet(1, struct.pack("I", speed.data))

if __name__ == "__main__":
    rospy.init_node('arduino_bridge')
    arduino = serial.Serial("/dev/ttyUSB0", 115200)
    sub = rospy.Subscriber('/arduino_motor', std_msgs.msg.Int32, motor_callback)

    rospy.spin()