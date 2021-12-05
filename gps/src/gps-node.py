from serial import Serial
from pynmeagps import NMEAReader, NMEAMessage
import rospy
from gps_common.msg import GPSFix, GPSStatus

stream = Serial('/dev/ttyACM0', 9600, timeout=5)
gps = NMEAReader(stream=stream)
node = rospy.init_node('gps_pub')
pub = rospy.Publisher('/gps_pub', GPSFix, queue_size=10)

seq = 0

while True:
    fix = GPSFix()
    status = GPSStatus()

    status.header.seq = seq
    fix.header.seq = seq
    seq += 1

    while True:
        (raw, parsed) = gps.read()

        if(parsed.msgID == "RMC"):
            fix.speed = parsed.spd

        if(parsed.msgID == "GGA"):
            fix.altitude = parsed.alt
    
        if(parsed.msgID == "GLL"):
            fix.latitude = parsed.lat
            fix.longitude = parsed.lon
            break
    
    fix.status = status
    pub.publish(fix)