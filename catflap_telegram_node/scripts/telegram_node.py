#!/usr/bin/env python

from telepot.loop import MessageLoop
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
import telepot
import yaml
from threading import Lock

class Telegram_node():

    def handle_incoming_msg(self,msg):
        content_type, chat_type, chat_id = telepot.glance(msg)
        # only reply to max
        if chat_id == self.mid and content_type == 'text':
            self.bot.sendMessage(self.mid,"received:{}".format(msg['text']))
            if msg['text'] == "/lock":
                self.door_lock_publisher.publish(True)
            if msg['text'] == "/open":
                self.door_lock_publisher.publish(False)
            if msg['text'] == "/photo":
                self.ir_publisher.publish(True)
        else:
            if content_type != 'text' and chat_id in self.knownIDs:
                self.bot.sendMessage(self.mid,"stop sending me {}s, please. I wont use anything like that.".format(content_type))
            if chat_id not in self.knownIDs:
                self.bot.sendMessage(chat_id,"Hi. Nice that you sent me a message. I will notify my master, so he may contact you, if he likes. you can also mail him: kostka.max@gmail.com")
                self.bot.sendMessage(self.mid,u"id {0}, name {1} sent me a msg. I will forward it to you".format(chat_id,msg['chat']['first_name']))
                self.bot.forwardMessage(self.mid,chat_id,msg['message_id'])



    def callback(self,data):
        rospy.logdebug("callback started. data = {0}".format(data.data))
        # set the lock, will be released at the end
        self.lock.acquire()
        # how to handle pictures? send the one which detect prey
        prey = False
        if ".png" in data.data:
            prey = (not ("_no_prey.png" in data.data)) and ("_prey.png" in data.data)
            if prey:
                image = data.data
                file_handle = open(image)
                self.bot.sendPhoto(self.mid,file_handle)
                file_handle.close()
                file_handle = open(image)
                self.bot.sendPhoto(self.cfg['andre']['id'],file_handle)
                file_handle.close()
                file_handle = open(image)
                self.bot.sendPhoto(self.cfg['andy']['id'],file_handle)
                self.bot.sendMessage(self.cfg['andy']['id'], u"Ohje vielleicht wollte die charly was mitbringen.")
                file_handle.close()                
            else:
                self.bot.sendMessage(self.mid, data.data.split("/")[-1])

        else:
            self.bot.sendMessage(self.mid,data.data)
        self.lock.release()
        rospy.logdebug("callback finished, data = {0}".format(data.data))


    def __init__(self):
        conf_file = open("/home/max/projects/catflap/ros_catkin_ws/src/catflap_telegram_node/scripts/conf.yaml", 'r')
        self.cfg = yaml.load(conf_file)
        conf_file.close()
        self.png_list = []
        self.thread_lock = False
        rospy.logdebug("telegram node is started now")
                
        telepot_token = self.cfg['bot']['token']
        self.bot = telepot.Bot(telepot_token)
        self.mid = self.cfg['max']['id']
        self.knownIDs = [self.cfg['max']['id'],self.cfg['andre']['id']]

        rospy.init_node('telegram_node',log_level=rospy.DEBUG)
        self.lock = Lock()
        rospy.Subscriber('telegram_message', String, self.callback, queue_size=150)
        rospy.logdebug('telegram node is now ready to operate')
        self.door_lock_publisher = rospy.Publisher('door_lock_command', Bool, queue_size = 1)
        self.ir_publisher = rospy.Publisher('outer_ir_sensor_state', Bool, queue_size = 1)
        MessageLoop(self.bot,self.handle_incoming_msg).run_as_thread()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()



if __name__ == '__main__':
    try:
        tgn = Telegram_node()
    except rospy.ROSInterruptException:
        rospy.logdebug('telegram node is stopped now')
