#!/usr/bin/env python
from time import localtime,strftime
from datetime import datetime
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import yaml
import rospy
from std_msgs.msg import String
from threading import Lock
import sys

class camera():

    def __init__(self):
        conf_file = open("/home/max/projects/catflap/ros_catkin_ws/src/catflap_image_collector/scripts/conf.yaml", 'r')
        self.cfg = yaml.load(conf_file)
        conf_file.close()
        self.telegram_publisher = rospy.Publisher('telegram_message', String, queue_size = 150)

            
    def camera_init(self):
        self.camera            = PiCamera()
        self.camera.resolution = self.cfg["camera"]["resolution"]
        self.camera.rotation   = self.cfg["camera"]["rotation"]
        self.camera.framerate  = self.cfg["camera"]["framerate"]
        self.rawCapture        = PiRGBArray(self.camera, size = self.cfg["camera"]["resolution"])
        self.telegram_publisher.publish("camera init")
        self.lock = Lock()

    def classifier_init(self):
        self.cascade = cv2.CascadeClassifier(self.cfg["camera"]["classifier"]["path"])
        bg_temp      = cv2.imread(self.cfg["camera"]["background_image"])
        bg_temp      = bg_temp[:,150:390,:]
        self.bg      = cv2.cvtColor(bg_temp,cv2.COLOR_BGR2GRAY)

    def get_cat_faces(self):
        pass
        
    def detect_prey(self,cnt, h, area_threshold = 200):
        # check if a contour reaches below height h
        # if the contour are below height h is above the threshold, return True
        prey_detected = False
        ind_list = []
        for i, point in enumerate(cnt):
            if cnt[i][0][1] <= h:
                ind_list.append(i)
        prey_cnt = np.delete(cnt,ind_list,0)
        if len(prey_cnt) > 0 and cv2.contourArea(prey_cnt) >= area_threshold:
            prey_detected = True
        else:
            prey_detected = False
    
        return (prey_detected,prey_cnt)
    
    def move_cnts(self,cnt, x,y):
        for i, point in enumerate(cnt):
            cnt[i][0][0] = cnt[i][0][0] + x
            cnt[i][0][1] = cnt[i][0][1] + y
        return cnt
    
    def get_biggest_contour(self,cnts):
        areas = []
        for c in cnts:
            areas.append(cv2.contourArea(c))
        sortedCnts = sorted( zip(areas, cnts), key= lambda x: x[0], reverse = True)
        return sortedCnts[0][1]
    
    def action(self, trust):
        self.lock.acquire()
        # take a picture, detect
        prey_detected = False
        catsnout_detected = False
        counter = 6
        settings = ""
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            self.image = frame.array
            self.rawCapture.truncate(0)
            counter -= 1
            if counter < 1:
                break
        n = datetime.now()
        timestamp_text = "{0:04d}_{1:02d}_{2:02d}_{3:02d}_{4:02d}_{5:02d}_{6:01d}".format(n.year,n.month,n.day,n.hour,n.minute,n.second,int(n.microsecond/100000))
        settings = "{0}: ".format(timestamp_text)
        settings = settings + "{0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}, {8}, {9}, {10}, {11}, {12}, {13}, {14}, {15}, {16}, {17}".format(self.camera.sharpness, self.camera.contrast, self.camera.brightness, self.camera.saturation, self.camera.ISO, self.camera.exposure_compensation, self.camera.exposure_mode, self.camera.meter_mode, self.camera.awb_mode, self.camera.analog_gain, self.camera.awb_gains, self.camera.digital_gain, self.camera.drc_strength, self.camera.exposure_speed, self.camera.iso, self.camera.saturation, self.camera.sensor_mode, self.camera.shutter_speed)
        settings = settings + "\n"
        try:
            file = open("/home/max/camera_settings.txt","a")
            file.write(settings)
            file.close()
        except Exception:
            rospy.logdebug("Unexpected Error: {0}".format(sys.exc_info()[0]))

        filename = "/home/max/Pictures/raw/{0}.png".format(timestamp_text)
        cv2.imwrite(filename,self.image)
        # region of interest, background area
        self.image = self.image[:,150:390,:]
        
        image_text = "no catsnout detected"
        ## detection
        rects = self.cascade.detectMultiScale(self.image,
                scaleFactor = 1.1, minNeighbors = 5,
                minSize = (67, 50), maxSize = (240, 195),
                flags = cv2.CASCADE_SCALE_IMAGE)
        if rects != ():
            catsnout_detected = True
            # reset picture sequence when catsnout was detected
            self.picture_sequence = 0
            for (x,y,w,h) in rects:
                (x1, y1) = (x + int(w*4./5.), y + h + h)
                if y1 > self.image.shape[0]:
                    y1 = self.image.shape[0]
                # paint catsnout detection window in green
                cv2.rectangle(self.image, (x,y), (x1, y1), (0,255,0),2)
            # grayscale image
            img = cv2.cvtColor(self.image,cv2.COLOR_BGR2GRAY)
            # histogram equalization
            img = cv2.equalizeHist(img)
            # extract region of interes
            roi = img[y:y1,x:x1]
            # extract region of interest in the background image
            bg_roi = self.bg[y:y1,x:x1]
            bg_roi = cv2.multiply(bg_roi,.6)
            # subtract the roi from background
            diff_roi = cv2.subtract(roi,bg_roi)
            blur = cv2.GaussianBlur(diff_roi,(5,5),0)
            ret,thresh = cv2.threshold(blur,0,255,cv2.THRESH_OTSU)
            # erode and dilate -> remove small blobs
            #thresh = cv2.erode(thresh, None, iterations=1)
            #thresh = cv2.dilate(thresh, None, iterations=2)
            #thresh = cv2.erode(thresh, None, iterations=1)
            #thresh = cv2.dilate(thresh, None, iterations=2)
            # invert    
            thresh = cv2.bitwise_not(thresh)
            # find contours
            image_c, cnts, hierarchy = cv2.findContours(thresh, 1, 2)
            # look for biggest contour
            biggest_cnt = self.get_biggest_contour(cnts)
            # shift contour coordinates from roi to orig image
            biggest_cnt_moved = self.move_cnts(biggest_cnt,x,y)
            # draw contour 
            cv2.drawContours(self.image, [biggest_cnt_moved], -1, (0, 60, 100), thickness = 1)
            # detect prey
            prey_detected, prey_cnt = self.detect_prey(biggest_cnt_moved, y + int(h/5*4))
            if prey_detected:
                # draw detection in red
                cv2.drawContours(self.image, [prey_cnt], -1, (0, 0, 255), thickness = 2)
                image_text = "prey detected"
                trust = trust * 0.33
                filename_mod = "/home/max/Pictures/classified/cat_prey/{0}_prey.png".format(timestamp_text)
            else:
                image_text = "no prey detected"
                trust = trust * 2.0
                filename_mod = "/home/max/Pictures/classified/cat_no_prey/{0}_no_prey.png".format(timestamp_text)
        else:
            trust = trust * 0.9
            filename_mod     = "/home/max/Pictures/classified/no_catsnout/{0}_no_catsnout.png".format(timestamp_text)

        cv2.putText(self.image,timestamp_text,(2,14),cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        cv2.putText(self.image,image_text,(2,32),cv2.FONT_HERSHEY_SIMPLEX, 0.66, (0, 0, 255), 2)
        cv2.putText(self.image,"{0:.2f}/3.00".format(trust),(2,50),cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 2)
        
        cv2.imwrite(filename_mod,self.image)
        self.telegram_publisher.publish(filename_mod)
        self.lock.release()
        return (catsnout_detected, prey_detected)
