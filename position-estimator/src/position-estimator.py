#!/usr/bin/env python

import rospy
import math

from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D  ## for lat/lon ## TODO: change to proper GPS

RAD_PER_DMM = math.pi / (180 * 60 * 10000)
KAPPA = 0.1

pub_location = rospy.Publisher('position', Float32)

class PositionEstimator:
    def __init__(self, lat1, lon1, lat2, lon2, a):
        self.sEstimated = 0
        self.sGPS = 0
        self.xGPS = 0

        self.LAT1DMM = lat1
        self.LON1DMM = lon1
        self.LAT2DMM = lat2
        self.LON2DMM = lon2

        self.vertexH = a

        self.LAT0DMM = (lat1 + lat2) / 2
        self.LON0DMM = (lon1 + lon2) / 2

        self.metersPerDMMLat = 0.185185185 ## 10,000 km over 90 degrees
        self.metersPerDMMLon = self.metersPerDMMLat * math.cos(self.LAT0DMM * RAD_PER_DMM)
    
        # find the delta
        self.deltaXi = (self.LON2DMM - self.LON0DMM) * self.metersPerDMMLon
        self.deltaEta = (self.LAT2DMM - self.LAT0DMM) * self.metersPerDMMLat

        # convert to unit vector
        self.xMax = math.sqrt(self.deltaXi * self.deltaXi + self.deltaEta * self.deltaEta)
        self.unitXi = self.deltaXi / self.xMax
        self.unitEta = self.deltaEta / self.xMax

        # sMax
        self.sMax = self.vertexH * self.sinh(self.xMax / self.vertexH)

        # default position to end by just copying over the pole location
        handleGPS(lat2, lon2)
        self.sEstimated = self.sGPS

    def handleGPS(self, latDMM, lonDMM):
        deltaLat = latDMM - self.LAT0DMM
        deltaLon = lonDMM - self.LON0DMM

        distLat = deltaLat * self.metersPerDMMLat;
        distLon = deltaLon * self.metersPerDMMLon;

        # dot the GPS vector with the unit vector of the cable to get x 
        # (ie, project the GPS reading to conform to the cable)
        self.xGPS = distLat * self.unitXi + distLon * self.unitEta
        self.sGPS = self.vertexH * math.sinh(self.xGPS / self.vertexH)

        self.sEstimated = self.sEstimated + KAPPA * (self.sGPS - self.sEstimated);

        return self.sGPS

    def handleEncoder(self, delta):
        self.sEstimated += delta
        pub_location.publish(self.sEstimated)
        rospy.loginfo("S = %f", self.s)
        ## we could also find x


estimator = PositionEstimator()

def EncoderCallback(msg):
    delta = msg.data
    estimator.handleEncoder(delta)
    rospy.loginfo("Delta -> %i meter", delta)

def GPSCallback(msg):
    lat = msg.data
    lon = msg.data
    s = estimator.handleGPS(lat, long)
    rospy.loginfo("Location -> %i meter", s)

def main():
    rospy.init_node('pos_est')
    rospy.Subscriber('/encoder/meters_per_interval', Float32, EncoderCallback)
    rospy.Subscriber('/gps/coords', Pose2D, GPSCallback)

    rospy.spin()

if __name__ == '__main__':
    main()
    
