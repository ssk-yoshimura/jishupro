#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from jsk_recognition_msgs.msg import PeoplePoseArray
import actionlib
from jsk_2021_10_semi.msg import *

# ActionServer
# ActionClientからgoalが送られると、画像とpose estimatorのtopicを受け取り始める
# resultを返すときに、画像とpose estimatorの受け取りをやめる
# yoshimura-ac.lがActionClient


class image_converter:

    def __init__(self):

        self.joint_names = ["left shoulder", "right shoulder", "left elbow", "right elbow"]
        self.joint_x = [0]*4
        self.joint_y = [0]*4
        self.fileRect = [0]*4 # ファイルのx,y,w,h

        self.goalVal = 0
        self.filew = []
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.bridge = CvBridge()
        self.server = actionlib.SimpleActionServer('yoshimura', YoshimuraAction, self.execute, False)
        self.server.start()
        
    # ActionServerの関数
    def execute(self, goal):
        print ("goal is" + str(goal.yoshimura_goal))
        rate = rospy.Rate(10)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback)
        self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses)
        while self.goalVal < 50:
            rate.sleep()
            if rospy.is_shutdown():
                break
        self.image_sub.unregister()
        self.pose_sub.unregister()
        cv2.destroyAllWindows()
        result = self.server.get_default_result()
        result.yoshimura_result = int(np.mean(self.filew)) #ファイルの横幅をresultとする
        print ("result is " + str(result.yoshimura_result))
        self.goalVal = 0
        self.filew = []
        self.server.set_succeeded(result)
        
    # 画像処理のコールバック関数
    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape

        # 緑色を閾値にして2値化
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = np.zeros((rows, cols))
        gray[(hsv[:,:,0] > 80) & (hsv[:,:,0] < 100) & (hsv[:,:,1] > 40)] = 255

        # 輪郭検出
        gray2 = np.zeros((rows, cols))
        gray = gray.astype(np.uint8)
        gray, contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # print(len(contours))

        # 最大領域の輪郭を探す
        maxareaidx = 0
        maxarea = 0
        for i, cnt in enumerate(contours):
            area = cv2.contourArea(cnt)
            if area > maxarea:
               maxareaidx = i
               maxarea = area

        go_flag = False
               
        # 最大領域の外接矩形
        if len(contours) > 0:
            gray2 = cv2.drawContours(gray2, [contours[maxareaidx]], 0, 100, 1)
            self.fileRect = cv2.boundingRect(contours[maxareaidx])
            x, y, w, h = self.fileRect
            cv2.rectangle(gray2, (x,y), (x+w, y+h), 255, 3)
            

            # 発進してよいかの判別
            f0 = max(self.joint_y[0], self.joint_y[1]) < y
            f1 = y < min(self.joint_y[2], self.joint_y[3])
            f2 = self.joint_x[3] < x
            f3 = x+w < self.joint_x[2]
            f4 = min(self.joint_y[2], self.joint_y[3]) < y+h
            f5 = abs(self.joint_x[0]+self.joint_x[1]-x-x-w) < 50
            go_flag = f0 & f1 & f2 & f3 & f4 & f5
            # print (abs(self.joint_x[0]+self.joint_x[1]-x-x-w))

            goc = [255, 255, 255] if go_flag else [0, 0, 255]
            cv2.rectangle(img, (x,y), (x+w, y+h), goc, 3)
            
        if go_flag:
            print("go_flag is " + str(go_flag) + ' ' + str(self.goalVal))
            self.goalVal += 1
            self.filew.append(self.fileRect[2])
        else:
            print("no")
            self.goalVal = 0
            self.filew = []
            
        # 人間の関節描画
        for i in range(4):
            cv2.circle(gray2, (int(self.joint_x[i]), int(self.joint_y[i])), 10, 255, thickness=3)
            cv2.circle(img, (int(self.joint_x[i]), int(self.joint_y[i])), 10, (255, 255, 255), thickness=3)
            
        cv2.imshow("img", img)
        cv2.imshow("gray2", gray2)
        cv2.waitKey(3)

        # cv2.imwrite('a.jpg', img)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

    # pose_estimatorのコールバック関数
    def callback_poses(self, data):
        if len(data.poses) == 0:
            return
        for i, v in enumerate(self.joint_names):
            if v in data.poses[0].limb_names:
                idx = data.poses[0].limb_names.index(v)
                self.joint_x[i] = data.poses[0].poses[idx].position.x
                self.joint_y[i] = data.poses[0].poses[idx].position.y
                # print(v + ' ' + str(self.joint_x[i]) + ' ' + str(self.joint_y[i]))

if __name__ == '__main__':    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    rospy.spin()
    cv2.destroyAllWindows()
