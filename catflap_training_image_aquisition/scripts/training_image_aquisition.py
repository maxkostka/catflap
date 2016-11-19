#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from raspicam import ***
from std_msgs.msg import Bool
from sensor_msgs import image


class training_image_aquisition:

    # Pin number where the inner sensors is connected to the Raspi (BCM pin number)
    self.inner_ir_sensor_pin = 16
    self.outer_ir_sensor_pin = 20
    # state of the ir sensor, True means activated, False means no activation (pin state is low active!)
    self.inner_ir_sensor_state = False
    self.pin_to_state = {0:True, 1:False}
    image_publisher = rospy.Publisher('catflap_training_image', image, queue_size = 10)

    def self.__init__(self):
    self.inner_ir_sensor_pin = 16
    self.inner_ir_sensor_state = False
    self.pin_to_state = {0:True, 1:False}
    state_publisher = rospy.Publisher('inner_ir_sensor_state', Bool, queue_size = 1)
    

def inner_ir_sensor_activated(inner_ir_sensor_pin):
    global inner_ir_sensor_state
    global state_publisher
    inner_ir_sensor_state = True
    state_publisher.publish(inner_ir_sensor_state)
    rospy.logdebug('inner ir sensor was activated')

def ir_sensor_inner_start():
    rospy.logdebug("inner ir sensor is started now")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    global inner_ir_sensor_pin
    global inner_ir_sensor_state
    global state_publisher
    GPIO.setup(inner_ir_sensor_pin,GPIO.IN, pull_up_down=GPIO.PUD_UP)

    inner_ir_sensor_state = pin_to_state[ GPIO.input(inner_ir_sensor_pin) ]
    rospy.logdebug('starting inner ir sensor, activated: {0}'.format(inner_ir_sensor_state))
    
    # define the publisher door_state
    state_publisher = rospy.Publisher('inner_ir_sensor_state', Bool, queue_size = 1)
    rospy.init_node('inner_ir_sensor',log_level=rospy.DEBUG)
    rate = rospy.Rate(3)

    # publisher ready - add the edge detection - Falling edge means activation
    GPIO.add_event_detect(inner_ir_sensor_pin,GPIO.FALLING, callback = inner_ir_sensor_activated)

    rospy.logdebug('inner ir sensor is now ready to operate, edge detection on')
    while not rospy.is_shutdown():
        sensorstate = pin_to_state[ GPIO.input(inner_ir_sensor_pin) ]
    # only falling edges are detected with interrups - correct the state for rising edges
        if sensorstate != inner_ir_sensor_state:
            inner_ir_sensor_state = sensorstate
            state_publisher.publish(inner_ir_sensor_state)
            rospy.logdebug('sensor deactivated')
        state_publisher.publish(inner_ir_sensor_state)
        rate.sleep()
        

if __name__ == '__main__':
    from inner_irsensor import inner_ir_sensor_pin
    try:
        ir_sensor_inner_start()
    except rospy.ROSInterruptException:
        rospy.logdebug('inner ir sensor is stopped now')
        # cleanup GPIO pins used
        GPIO.cleanup(inner_ir_sensor_pin)
        rospy.logdebug('inner ir sensor stopped, GPIO cleaned up')
    else:
        rospy.logdebug('inner ir sensor is stopped now')
        # cleanup GPIO pins used
        GPIO.cleanup(inner_ir_sensor_pin)
        rospy.logdebug('inner ir sensor stopped, GPIO cleaned up')
