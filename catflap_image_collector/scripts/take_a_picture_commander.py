#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from time import localtime,strftime
from datetime import datetime
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2


class Take_a_picture_commander():
    def __init__(self):
        # define our state variables
        self.door_state = False
        self.timeout =  25
        self.ir_state = False
        # define the listeners and their callbacks
        rospy.Subscriber("door_state",            Bool ,self.callback_door_sensor)
        rospy.Subscriber("inner_ir_sensor_state", Bool ,self.callback_ir_sensor)
        rospy.Subscriber("outer_ir_sensor_state", Bool ,self.callback_ir_sensor)
        self.light_publisher = rospy.Publisher("lightswitch_command", Bool, queue_size=3)
        # start the publisher
        self.take_a_picture_publisher = rospy.Publisher('take_a_picture', Bool, queue_size = 1)
        self.door_lock_publisher = rospy.Publisher('door_lock_command', Bool, queue_size = 1)
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.camera.resolution = (400, 300)
        self.camera.rotation = 180
        self.camera.framerate = 90
        self.rawCapture = PiRGBArray(self.camera, size=(400, 300))

        self.light_publisher.publish(False)
        self.door_lock_publisher.publish(False)
        
        rospy.spin()
        
    def callback_door_sensor(self, door_sensor_data):
        if door_sensor_data.data == False:
            # door open, do nothing, set the timeout
            if self.timeout != 25:
                self.timeout = 25
                self.light_publisher.publish(False)
            self.door_state = True
            
        else:
            # door closed, count down the timeout, if neccessary
            # further condition: only count down, if both sensors have been inactive - ir_state = 0
            if self.timeout > 0 and self.ir_state == 0: self.timeout -= 1
            self.door_state = False

    def callback_ir_sensor(self, data):
        if data.data and self.door_state == False and self.timeout < 1:
            self.timeout = 4
            self.light_publisher.publish(True)
            self.take_a_picture_publisher.publish(True)
            counter = 6
            for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
                self.image = frame.array
                self.rawCapture.truncate(0)
                counter -= 1
                if counter < 1:
                    break
            self.light_publisher.publish(False)
            n = datetime.now()
            timestamp = "{0:04d}_{1:02d}_{2:02d}_{3:02d}_{4:02d}_{5:02d}_{6:01d}".format(n.year,n.month,n.day,n.hour,n.minute,n.second,int(n.microsecond/100000))
            filename = "/home/max/Pictures/training/{0}.jpg".format(timestamp)
            cv2.imwrite(filename,self.image)
            print strftime("{0} {1} {2} {3}:{4}:{5} - picture taken").format(n.year,n.month,n.day,n.hour,n.minute,n.second)
        # keep track of the ir states
        # both sensors send data in regular intervals
        # we want to keep track of situations, where one sensor maybe active
        # e.g. cat sitting in front of it - the callback function will recieve True False True ...
        if data.data:
            # if any sensor is active set state to 2
            self.ir_state = 2
        else:
            # we have a not active sensor, decrement the state until it's 0
            if self.ir_state > 0:  self.ir_state -= 1
            # state is zero after two consecutive False messages
            

if __name__ == '__main__':
    
    rospy.init_node("take_a_picture_commander")
    
    try:
        
        tapc_node = Take_a_picture_commander()

    except rospy.ROSInterruptException:
        # cleanup
        tapc_node.light_publisher.publish(False)
        pass
    else:
        # cleanupt
        tapc_node.light_publisher.publish(False)
        pass

