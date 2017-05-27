#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import camera

class Take_a_picture_commander():
    def __init__(self):
        # define our state variables
        self.door_closed = True
        self.camera_blocked = False
        self.timeout =  5
        self.ir_state = 2
        self.trust = 1.
        self.trust_threshold = 4.
        self.picture_sequence = 0
        # define the listeners and their callbacks
        rospy.Subscriber("door_closed_state",            Bool ,self.callback_door_sensor, queue_size = 1)
        rospy.Subscriber("inner_ir_sensor_state", Bool ,self.callback_ir_sensor, queue_size = 1)
        rospy.Subscriber("outer_ir_sensor_state", Bool ,self.callback_ir_sensor, queue_size = 1)
        self.light_publisher = rospy.Publisher("lightswitch_command", Bool, queue_size=10)
        # start the publisher
        self.take_a_picture_publisher = rospy.Publisher('take_a_picture', Bool, queue_size = 1)
        self.light_publisher.publish(False)
        self.door_lock_publisher = rospy.Publisher('door_lock_command', Bool, queue_size = 1)
        self.door_lock_publisher.publish(True)
        self.camera = camera.camera()
        self.camera.camera_init()
        self.camera.classifier_init()
        rospy.spin()
        
    def callback_door_sensor(self, door_closed_sensor_data):
        #rospy.logdebug("door closed data: {0}".format(door_closed_sensor_data.data))
        if door_closed_sensor_data.data == False:
            if self.door_closed == True:
                rospy.logdebug("door has been opened")
            # door open, do nothing, set the timeout
            if self.timeout != 5:
                self.timeout = 5
            self.door_closed = False
        else:
            # door closed, count down the timeout, if neccessary
            # further condition: only count down, if both sensors have been inactive - ir_state = 0
            if self.door_closed == False:
                rospy.logdebug("door has been closed")
            if self.timeout > 0 and self.ir_state == 0:
                self.timeout -= 1
                if self.timeout == 0:
                    # reset the picture sequence
                    self.picture_sequence = 0
                    self.trust = 1.0
                    rospy.logdebug("door closed a while, timeout reached, picture sequence reset")
            self.door_closed = True

    def callback_ir_sensor(self, data):
        #if data.data == False:
        #    if self.
        #rospy.logdebug("ir active: {0}; door closed: {1}; timeout: {2}".format(data.data,self.door_closed,self.timeout))
        if data.data and self.door_closed == True and self.timeout < 1 and not self.camera_blocked:
            rospy.logdebug("trust is at {}".format(self.trust))
            self.camera_blocked = True
            self.light_publisher.publish(True)
            #rospy.logdebug("picture will be taken")
            (catsnout_detected, pray_detected) = self.camera.action()
            if catsnout_detected:
                if pray_detected:
                    rospy.logdebug("pray detected")
                    self.trust = self.trust * 0.33
                    self.door_lock_publisher.publish(True)
                else:
                    rospy.logdebug("no pray detected")
                    self.trust = self.trust * 2.
                    if self.trust >= self.trust_threshold:
                        rospy.logdebug("door opened, trust is {}".format(self.trust))
                        self.door_lock_publisher.publish(False)
            else:
                self.picture_sequence += 1
                self.trust = self.trust * 0.9
                if self.picture_sequence > 25:
                    rospy.logdebug("door opened after 25 no detections, trust at {}".format(self.trust))
                    self.door_lock_publisher.publish(False)
                else:
                    self.door_lock_publisher.publish(True)
                rospy.logdebug("no catsnout detected")
            self.light_publisher.publish(False)
            self.camera_blocked = False
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
    
    rospy.init_node("take_a_picture_commander",log_level=rospy.DEBUG)
    if True:
    #try:
        
        tapc_node = Take_a_picture_commander()

    #except rospy.ROSInterruptException:
    #    rospy.loginfo("node shutdown by ROSINterruptException")
    #except:
    #    rospy.logerr("unhandled eception")
    #finally:
    #    # cleanup
    #    tapc_node.door_lock_publisher.publish(False)
    #    tapc_node.light_publisher.publish(False)
    #    pass

