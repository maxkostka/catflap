#!/usr/bin/env python
from time import localtime,strftime
from datetime import datetime
from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import numpy as np
import yaml


class camera():

    def __init__(self):
        conf_file = open("/home/max/projects/catflap/ros_catkin_ws/src/catflap_image_collector/scripts/conf.yaml", 'r')
        self.cfg = yaml.load(conf_file)
        conf_file.close()
            
    def camera_init(self):
        self.camera            = PiCamera()
        self.camera.resolution = self.cfg["camera"]["resolution"]
        self.camera.rotation   = self.cfg["camera"]["rotation"]
        self.camera.framerate  = self.cfg["camera"]["framerate"]
        self.rawCapture        = PiRGBArray(self.camera, size = self.cfg["camera"]["resolution"])


    def classifier_init(self):
        self.cascade = cv2.CascadeClassifier(self.cfg["camera"]["classifier"]["path"])
        bg_temp      = cv2.imread(self.cfg["camera"]["background_image"])
        self.bg      = cv2.cvtColor(bg_temp,cv2.COLOR_BGR2GRAY)
        
    def get_cat_faces(self):
        pass
        
    def detect_pray(self,cnt, h, area_threshold = 200):
        # check if a contour reaches below height h
        # if the contour are below height h is above the threshold, return True
        pray_detected = False
        ind_list = []
        for i, point in enumerate(cnt):
            if cnt[i][0][1] <= h:
                ind_list.append(i)
        pray_cnt = np.delete(cnt,ind_list,0)
        if len(pray_cnt) > 0 and cv2.contourArea(pray_cnt) >= area_threshold:
            pray_detected = True
        else:
            pray_detected = False
    
        return (pray_detected,pray_cnt)
    
    def move_cnts(self,cnt, x,y):
        #print "move cnt"
        #print len(cnt)
        for i, point in enumerate(cnt):
            cnt[i][0][0] = cnt[i][0][0] + x
            cnt[i][0][1] = cnt[i][0][1] + y
        return cnt
    
    def get_biggest_contour(self,cnts):
        #print "get biggest cnt"
        #print len(cnts)
        #for c in cnts:
        #    print len(c)
        areas = []
        for c in cnts:
            areas.append(cv2.contourArea(c))
        sortedCnts = sorted( zip(areas, cnts), key= lambda x: x[0], reverse = True)
        return sortedCnts[0][1]
    
    def action(self):
        # take a picture, detect
        pray_detected = False
        catsnout_detected = False
        counter = 6
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            self.image = frame.array
            self.rawCapture.truncate(0)
            counter -= 1
            if counter < 1:
                break
        n = datetime.now()
        timestamp = "{0:04d}_{1:02d}_{2:02d}_{3:02d}_{4:02d}_{5:02d}_{6:01d}".format(n.year,n.month,n.day,n.hour,n.minute,n.second,int(n.microsecond/100000))
        filename = "/home/max/Pictures/raw/{0}.png".format(timestamp)
        cv2.imwrite(filename,self.image)
        ## detection
        rects = self.cascade.detectMultiScale(self.image,
                scaleFactor = 1.1, minNeighbors = 5,
                minSize = (100, 75), maxSize = (240, 195),
                flags = cv2.CASCADE_SCALE_IMAGE)
        if rects != ():
            catsnout_detected = True
            # reset picture sequence when catsnout was detected
            self.picture_sequence = 0
            for (x,y,w,h) in rects:
                (x1, y1) = (x + w, y + h + h)
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
            # detect pray
            pray_detected, pray_cnt = self.detect_pray(biggest_cnt_moved, y + int(h/5*4))
            if pray_detected:
                # draw detection in red
                cv2.drawContours(self.image, [pray_cnt], -1, (0, 0, 255), thickness = 2)
                filename_mod = "/home/max/Pictures/classified/cat_pray/{0}_pray.png".format(timestamp)
            else:
                filename_mod = "/home/max/Pictures/classified/cat_no_pray/{0}_no_pray.png".format(timestamp)
        else:
            filename_mod     = "/home/max/Pictures/classified/no_catsnout/{0}_no_catsnout.png".format(timestamp)

        cv2.imwrite(filename_mod,self.image)
        return (catsnout_detected, pray_detected)
