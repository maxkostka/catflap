#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import telepot
import yaml

class Telegram_node():

    def callback(self,data):
        if ".png" in data.data:
            file_handle = open(data.data)
            self.bot.sendPhoto(self.mid,file_handle)
            file_handle.close()    
        else:
            self.bot.sendMessage(self.mid,data.data)

    def __init__(self):
        conf_file = open("/home/max/projects/catflap/ros_catkin_ws/src/catflap_telegram_node/scripts/conf.yaml", 'r')
        self.cfg = yaml.load(conf_file)
        conf_file.close()
        rospy.logdebug("telegram node is started now")
        
        telepot_token = self.cfg["token"]
        self.bot = telepot.Bot(telepot_token)
        self.mid = self.cfg["id"]

        rospy.init_node('telegram_node',log_level=rospy.DEBUG)
        rospy.Subscriber('telegram_message', String, self.callback, queue_size=150)
        rospy.logdebug('telegram node is now ready to operate')

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()



if __name__ == '__main__':
    try:
        tgn = Telegram_node()
    except rospy.ROSInterruptException:
        rospy.logdebug('telegram node is stopped now')
