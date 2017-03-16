#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Pin number where the door sensor is connected to Raspi (BCM pin number)
door_sensor = 19
# Pin number where the door sensor supply is connected to Raspi (BCM pin number)
#door_sensor_supply = 19
# doorstate global variable, True=closed; False=open

doorstate = True

def door_closed(channel):
    global doorstate
    global door_sensor
    doorstate = True
    GPIO.remove_event_detect(door_sensor)
    GPIO.add_event_detect(door_sensor,GPIO.FALLING, callback = door_opened)
    rospy.loginfo("door closed")

def door_opened(channel):
    global doorstate
    global door_sensor
    doorstate = False
    GPIO.remove_event_detect(door_sensor)
    GPIO.add_event_detect(door_sensor,GPIO.RISING, callback = door_closed)
    rospy.loginfo("door opened")


def door_sensor_start():
    rospy.logdebug("door sensor is started now")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    global doorstate
    global door_sensor
 #   global door_sensor_supply

    # sensing pin is pulldown input
    GPIO.setup(door_sensor,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    # supply pin is output high
    #GPIO.setup(door_sensor_supply,GPIO.OUT, initial=1)
    # supply is connected to sensing pin when the door is closed
    if GPIO.input(door_sensor) == 1:
        doorstate = True
        rospy.logdebug('starting with door closed')
        GPIO.add_event_detect(door_sensor,GPIO.FALLING, callback = door_opened)
    else:
        doorstate = False
        rospy.logdebug('starting with door opened')
        GPIO.add_event_detect(door_sensor,GPIO.RISING, callback = door_closed)

    # define the publisher door_state
    door_state_publisher = rospy.Publisher('door_state', Bool, queue_size = 1)
    rospy.init_node('door_sensor',log_level=rospy.DEBUG)
    rate = rospy.Rate(5)
    rospy.logdebug('door sensor is now ready to operate')
    while not rospy.is_shutdown():
        sensorstate = GPIO.input(door_sensor)
        if sensorstate != doorstate:
            doorstate = sensorstate
            if doorstate:
                rospy.loginfo('door state was automatically corrected - now closed')
            else:
                rospy.loginfo('door state was automatically corrected - now opened')

        door_state_publisher.publish(doorstate)
        rate.sleep()

if __name__ == '__main__':
    from doorsensor import door_sensor
    #from doorsensor import door_sensor_supply
    try:
        door_sensor_start()
    except rospy.ROSInterruptException:
        rospy.logdebug('door sensor is stopped now')
        # cleanup GPIO pins used
        #GPIO.output(door_sensor_supply,GPIO.LOW)
        GPIO.cleanup(door_sensor)
        #GPIO.cleanup(door_sensor_supply)
        rospy.logdebug('door sensor stopped, GPIO cleaned up')
    else:
        rospy.logdebug('door sensor is stopped now')
        # cleanup GPIO pins used
        #GPIO.output(door_sensor_supply,GPIO.LOW)
        GPIO.cleanup(door_sensor)
        #GPIO.cleanup(door_sensor_supply)
        rospy.logdebug('door sensor stopped, GPIO cleaned up')

