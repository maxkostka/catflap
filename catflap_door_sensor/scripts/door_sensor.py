#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

class Door_sensor():
    # two pins for convenient detection of rising and falling edges
    door_sensor_pin1 = 13
    door_sensor_pin2 = 6

    def callback_door_closed(self,channel):
        self.door_closed = False
        self.door_closed_publisher.publish(self.door_closed)

    def callback_door_opened(self,channel):
        self.door_closed = True
        self.door_closed_publisher.publish(self.door_closed)


    def __init__(self):
        # state variable
        self.door_closed = True
        rospy.logdebug("door sensor will be started now")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.door_sensor_pin1,GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.door_sensor_pin2,GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(self.door_sensor_pin2,GPIO.FALLING, callback = self.callback_door_opened)
        GPIO.add_event_detect(self.door_sensor_pin1,GPIO.RISING,  callback = self.callback_door_closed)

        # start the publisher door_state_closed
        self.door_closed_publisher = rospy.Publisher('door_closed_state', Bool, queue_size = 1)
        rospy.init_node('door_sensor')
        rate = rospy.Rate(1)
        rospy.logdebug('door_sensor is now ready to operate')
    
        while not rospy.is_shutdown():
            dsp1 = GPIO.input(self.door_sensor_pin1)
            dsp2 = GPIO.input(self.door_sensor_pin2)
            if dsp1 == 1 and dsp2 == 1:
                if self.door_closed != False:
                    rospy.logwarn('door state was automatically corrected - now opened')
                self.door_closed = False
                rospy.logdebug('door open')
            else:
                if self.door_closed != True:
                    rospy.logwarn('door state was automatically corrected - now closed')
                self.door_closed = True
                rospy.logdebug('door possibly closed')
            self.door_closed_publisher.publish(self.door_closed)
            rate.sleep()

if __name__ == '__main__':
    try:
        ds = Door_sensor()
    except rospy.ROSInterruptException:
        rospy.logwarn('door sensor is stopped by ROS')
    except:
        rospy.logerr("unhandled exception")
    finally:
        rospy.logdebug('GPIO cleanup')
        GPIO.cleanup()
