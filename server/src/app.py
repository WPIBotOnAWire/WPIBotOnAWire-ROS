#!/usr/bin/env python3
import threading
import rospy
import socket
import time
import struct
import json
from os import path
from std_msgs import msg
from gps_common.msg import GPSFix, GPSStatus

# Packet format:
# /------------------------------------------------------------\
# | CODE (sizeof(uint32)) |  LEN (sizeof(uint32)) | DATA (LEN) |
# \------------------------------------------------------------/

# Codes:
# Heartbeat = 1

heartbeat_timer = 2
id = b''

home_server_host = "localhost"
home_server_port = 5555

class Packet:
    code: int = 0
    data: bytes = []

    def __init__(self, code, data):
        self.code = code
        self.data = data

def load_uuid() -> bytes:
    if not path.exists(path.expanduser("~/uuid")):
        f = open(path.expanduser("~/uuid"), mode="x")
    f = open(path.expanduser("~/uuid"), mode="rb")
    id = f.read(36)
    if len(id) == 0: return b''
    return id
    
def save_uuid():
    f = open(path.expanduser("~/uuid"), "wb")
    f.write(id)
    f.close()

def send_packet(s: socket.socket, p: Packet):
    # Packet format:
    # /---------------------------------------------------\
    # | CODE (uint32) |  LEN (uint32) | params (variable) |
    # \---------------------------------------------------/

    data = struct.pack("II%ss"%len(p.data), p.code, len(p.data), p.data)
    s.send(data)

def heartbeat(s: socket.socket):
    while(True):
        p = Packet(1, id)
        send_packet(s, p)
        time.sleep(heartbeat_timer)

def recv(s: socket.socket) -> Packet:
    pack = Packet

    code = s.recv(4)
    pack.code = int.from_bytes(code, 'little', signed=False)

    length = s.recv(4)
    length = int.from_bytes(length, 'little', signed=False)

    data = s.recv(length)
    pack.data = data

    return pack

def gps_cb(d: GPSFix):
    data = {
        "lat": d.latitude,
        "lon": d.longitude,
        "alt": d.altitude,
        "spd": d.speed
    }

    p = Packet(2, bytes(json.dumps(data), 'UTF-8'))
    send_packet(s, p)

def listener(s: socket.socket):
    global id

    while(True):
        p = recv(s)
        if(p.code == 0):
            print("remote closed connection.")
            return
        
        if(p.code == 1):
            if(len(p.data) > 0 and p.data != id):
                id = p.data
                save_uuid()

        if(p.code == 2):
            print("Setting motor speed to: %d", int.from_bytes(p.data, 'little', signed=False))
            motor_pub.publish(int.from_bytes(p.data, 'little', signed=False))

if __name__ == "__main__":
    rospy.init_node("server_node")
    id = load_uuid()
    print("Starting Server Node...")

    motor_pub = rospy.Publisher("/motor_speed", msg.Int32, queue_size=10)
    gps_sub = rospy.Subscriber("/gps_pub", GPSFix, callback=gps_cb)

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((home_server_host, home_server_port))

    # start heartbeat thread
    hbthread = threading.Thread(target=heartbeat, args=(s,))
    hbthread.start()

    # start listener thread
    recvthread = threading.Thread(target=listener, args=(s,))
    recvthread.start()

    rospy.spin()
