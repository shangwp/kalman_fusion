#!/usr/bin/python
# -*- coding: utf-8 -*-
import time
import serial
import struct
import time
import re
import rospy, tf
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import math


class Uwb:
    def __init__(self, port_path):
        self.com_ser = serial.Serial(port_path, 115200)
        self.connect_ = struct.pack('>BB', 0x0D, 0x0D)
        self.enable_ = "lep\r"
        self.disable_ = "lep\r"
        self.read_data = []
        self.update_frequency = 20.0
        self.wait_time = 1
        self.wait_loop_number = 0
        self.origin = Odometry()
        self.current = Odometry()
        self.br = tf.TransformBroadcaster()
        self.lastpose = PoseStamped()

    def start(self):
        self.com_ser.write(self.connect_)
        time.sleep(1)
        self.com_ser.write(self.enable_)

    def get_return_value(self):
        odom = Odometry()
        #print(self.com_ser.inWaiting())
        while self.com_ser.inWaiting() >0:
            #print("while")
            self.read_data.append(self.com_ser.read(1)[0])
        #print(len(self.read_data))
        if len(self.read_data) > 0:
            #print("if1",self.read_data)
            read_string = ''.join(self.read_data)
            print("s0",read_string)
            poses = re.findall(r'POS,\d*.\d*,.*\d*.\d*,.*\d*.\d*,\d*', read_string)
            #print ("s1",poses)
            read_string = re.sub(r'POS,\d*.\d*,\d*.\d*,.*\d*.\d*,\d*', "", read_string)
            #print("s2",read_string)
            #read_string = re.findall(r'POS*?$', read_string)
            #print("s3",read_string)
            if len(read_string) > 0:
                #print("if2")
                self.read_data = list(read_string[len(read_string) - 1])
            else:
                self.read_data = []
            if len(poses) > 0:
                #print("if3")
                self.wait_loop_number = 0
                pos = []
                datas = re.findall(r'[. \d]*', poses[len(poses) - 1])
                for data in datas:
                    if data:
                        pos.append(float(data))
                if len(pos) == 4 and pos[3] >= 50:
                    return pos

        return []

        self.wait_loop_number = self.wait_loop_number + 1
        if self.wait_loop_number > (self.wait_time * self.update_frequency):
            self.com_ser.write(self.enable_)
            self.wait_loop_number = 0

    def update(self):
        pub = rospy.Publisher('/uwb_odom', Odometry, queue_size=1000)
        rate = rospy.Rate(self.update_frequency)
        self.tick = 0
        xpos = []
        ypos = []
        zpos = []
        while not rospy.is_shutdown():
            pos = self.get_return_value()
            print "begin,",pos
            if len(pos) > 0:

                odom = Odometry()
                odom.header.stamp = rospy.Time.now()
                # odom.header.frame_id = 'base_footprint'
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                xpos.append(pos[0])
                ypos.append(pos[1])
                if len(xpos) > 3:
                    xpos = xpos[-3:]
                    ypos = ypos[-3:]

                odom.pose.pose.position.x = sum(xpos) / 3.0  # - self.origin.pose.pose.position.x
                odom.pose.pose.position.y = sum(ypos) / 3.0  # - self.origin.pose.pose.position.y
                # odom.pose.pose.position.z = pos[2]
                odom.pose.pose.orientation.x = self.current.pose.pose.orientation.x;
                odom.pose.pose.orientation.y = self.current.pose.pose.orientation.y;
                odom.pose.pose.orientation.z = self.current.pose.pose.orientation.z;
                odom.pose.pose.orientation.w = self.current.pose.pose.orientation.w;

                if (len(xpos) == 3):
                    if(self.lastpose.pose.position.x == 0 and self.lastpose.pose.position.y == 0):
                        self.lastpose.pose.position.x = odom.pose.pose.position.x
                        self.lastpose.pose.position.y = odom.pose.pose.position.y
                    distance = math.sqrt(math.pow((self.lastpose.pose.position.x-odom.pose.pose.position.x),2)+math.pow((self.lastpose.pose.position.y-odom.pose.pose.position.y),2))
                    print distance
                    if(distance < 0.15):
                        pub.publish(odom)
                        self.br.sendTransform((odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z),
                                          (odom.pose.pose.orientation.x,odom.pose.pose.orientation.y,odom.pose.pose.orientation.z,odom.pose.pose.orientation.w),
                                          rospy.Time.now(),"base_footprint","odom")
                        self.lastpose.pose.position.x = odom.pose.pose.position.x
                        self.lastpose.pose.position.y = odom.pose.pose.position.y

                    print pos[0], pos[1]
                    #print odom
                    #print "uwbtime=", odom.header.stamp

            rate.sleep()
    def odom_callback(self,data):
        self.current.pose.pose.orientation.x = data.pose.pose.orientation.x
        self.current.pose.pose.orientation.y = data.pose.pose.orientation.y
        self.current.pose.pose.orientation.z = data.pose.pose.orientation.z
        self.current.pose.pose.orientation.w = data.pose.pose.orientation.w

    def __del__(self):
        self.com_ser.write(self.disable_)


if __name__ == '__main__':
    rospy.init_node('uwb', anonymous=True)
    uwb = Uwb("/dev/ttyACM0")
    sub = rospy.Subscriber("odom",Odometry,uwb.odom_callback)
    uwb.start()
    uwb.update()
    rospy.spin()
