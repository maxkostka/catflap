#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Pin number where the outer IR sensor is connected to the Raspi (BCM pin number)
outer_ir_sensor_pin = 20
# state of the ir sensor, True means activated, False means no activation (pin state is low active!)
outer_ir_sensor_state = False
pin_to_state = {0:True, 1:False}
state_publisher = rospy.Publisher('outer_ir_sensor_state', Bool, queue_size = 1)

def outer_ir_sensor_activated(outer_ir_sensor_pin):
    global outer_ir_sensor_state
    global state_publisher
    outer_ir_sensor_state = True
    state_publisher.publish(outer_ir_sensor_state)
    rospy.logdebug('outer ir sensor was activated')

def ir_sensor_outer_start():
    rospy.logdebug("outer ir sensor is started now")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    global outer_ir_sensor_pin
    global outer_ir_sensor_state
    global state_publisher
    GPIO.setup(outer_ir_sensor_pin,GPIO.IN, pull_up_down=GPIO.PUD_UP)

    outer_ir_sensor_state = pin_to_state[ GPIO.input(outer_ir_sensor_pin) ]
    rospy.logdebug('starting outer ir sensor, activated: {0}'.format(outer_ir_sensor_state))
    
    # define the publisher door_state
    state_publisher = rospy.Publisher('outer_ir_sensor_state', Bool, queue_size = 1)
    rospy.init_node('outer_irsensor',log_level=rospy.DEBUG)
    rate = rospy.Rate(3)

    # publisher ready - add the edge detection - Falling edge means activation
    GPIO.add_event_detect(outer_ir_sensor_pin,GPIO.FALLING, callback = outer_ir_sensor_activated)

    rospy.logdebug('outer ir sensor is now ready to operate, edge detection on')
    while not rospy.is_shutdown():
        sensorstate = pin_to_state[ GPIO.input(outer_ir_sensor_pin) ]
	# only falling edges are detected with interrups - correct the state for rising edges
        if sensorstate != outer_ir_sensor_state:
            outer_ir_sensor_state = sensorstate
            state_publisher.publish(outer_ir_sensor_state)
            rospy.logdebug('sensor deactivated')
        state_publisher.publish(outer_ir_sensor_state)
        rate.sleep()
        

if __name__ == '__main__':
    from outer_irsensor import outer_ir_sensor_pin
    try:
        ir_sensor_outer_start()
    except rospy.ROSInterruptException:
        rospy.logdebug('outer ir sensor is stopped now')
        # cleanup GPIO pins used
        GPIO.cleanup(outer_ir_sensor_pin)
        rospy.logdebug('outer ir sensor stopped, GPIO cleaned up')
    else:
        rospy.logdebug('outer ir sensor is stopped now')
        # cleanup GPIO pins used
        GPIO.cleanup(outer_ir_sensor_pin)
        rospy.logdebug('outer ir sensor stopped, GPIO cleaned up')
