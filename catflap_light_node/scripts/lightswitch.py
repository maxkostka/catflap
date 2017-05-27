#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

# Pin number where the light toggle ist connected to Raspi (BCM pin number)
lightpin = 26


def callback(data):
    global lightpin
    if data.data == True:
        GPIO.output(lightpin,GPIO.HIGH)
    else:
        GPIO.output(lightpin,GPIO.LOW)

def lightswitch():
    rospy.logdebug("lighstwith is started now")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    global lightpin
    GPIO.setup(lightpin,GPIO.OUT, initial=0)
    
    rospy.init_node('lightswitch',log_level=rospy.DEBUG)
    rospy.Subscriber('lightswitch_command', Bool, callback, queue_size=10)
    rospy.logdebug('lightswitch is now ready to operate')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    from lightswitch import lightpin
    try:
        lightswitch()
    except rospy.ROSInterruptException:
        rospy.logdebug('lightswitch is stopped now')
        GPIO.output(lightpin,GPIO.LOW)
        GPIO.cleanup(lightpin)
        rospy.logdebug('lightswitch stopped, GPIO cleaned up')
