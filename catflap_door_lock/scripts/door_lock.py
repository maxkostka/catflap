#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
from time import sleep
# Pin number where the door_lock servo is connected to Raspi (BCM pin number)
servopin = 5

class Door_lock():
    def __init__(self):
        GPIO.setwarnings(False)
        self.servopin = 5
        self.timeout = 25
        self.locked = False
        self.lock()
        rospy.logdebug("door_lock is started now")
        rospy.Subscriber('door_lock_command', Bool, self.callback_door_lock,queue_size=1)
        rospy.logdebug('door_lock is now ready to operate')
    
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def open(self):
        #print "open ..."
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servopin,GPIO.OUT)
        servo_pwm_handle = GPIO.PWM(self.servopin,50)
        servo_pwm_handle.start(12)
        sleep(0.01)
        for dutyCycle in range(11,2,-1):
            servo_pwm_handle.ChangeDutyCycle(dutyCycle)
            #print dutyCycle
            sleep(0.01)
        sleep(0.4)
        servo_pwm_handle.ChangeDutyCycle(0)
        servo_pwm_handle.stop()
        #print "opened"
        GPIO.cleanup(self.servopin)
        self.locked = False

    def lock(self):
        #print "lock ..."
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.servopin,GPIO.OUT)
        servo_pwm_handle = GPIO.PWM(self.servopin,50)
        servo_pwm_handle.start(2)
        sleep(0.01)
        for dutyCycle in range(3,12,1):
            servo_pwm_handle.ChangeDutyCycle(dutyCycle)
            #print dutyCycle
            sleep(0.01)
        sleep(0.4)
        servo_pwm_handle.ChangeDutyCycle(0)
        servo_pwm_handle.stop()
        #print "locked"
        GPIO.cleanup(self.servopin)
        self.locked = True

    def callback_door_lock(self, data):
        # data is bool, lock = True, open = False
        #rospy.logdebug("callback")
        if data.data == True and self.locked == False:
            self.lock()
        elif data.data == False and self.locked == True:
            self.open()
            sleep(5)
            self.lock()
        #rospy.logdebug("callback ended")
        

if __name__ == '__main__':

    rospy.init_node('door_lock',log_level=rospy.DEBUG)
    
    try:
        door_lock_node = Door_lock()
    except rospy.ROSInterruptException:
        rospy.loginfo("rosnode shutdown by ROSInterruptException")
#    except:
#        rospy.logerr("unhandled exception")
    finally:
#        door_lock_node.open()
        GPIO.cleanup()
        print "door_lock_node closed with an exception, GPIO cleaned up"
        rospy.logdebug('door_lock stopped with an exception, GPIO cleaned up')
        
