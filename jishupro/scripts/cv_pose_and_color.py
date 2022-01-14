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

# human estimatorと画像の色情報から、書類が適切な位置にあるかを識別する

class image_converter:

    def __init__(self):

        self.joint_names = ["left shoulder", "right shoulder", "left elbow", "right elbow", "right wrist", "left wrist"]
        self.joint_size = len(self.joint_names)
        self.joint_x = np.zeros(self.joint_size)
        self.joint_y = np.zeros(self.joint_size)
        self.fileRect = [0]*4 # ファイルの隅
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) # PC
        # self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback) # usb camera
        self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses) # PV, usb

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
            """
            f0 = max(self.joint_y[0], self.joint_y[1]) < y
            f1 = y < min(self.joint_y[2], self.joint_y[3])
            f2 = self.joint_x[3] < x
            f3 = x+w < self.joint_x[2]
            f4 = min(self.joint_y[2], self.joint_y[3]) < y+h
            f5 = abs(self.joint_x[0]+self.joint_x[1]-x-x-w) < 50
            go_flag = f0 & f1 & f2 & f3 & f4 & f5
            # print (abs(self.joint_x[0]+self.joint_x[1]-x-x-w))
            """
            go_flag = False

            goc = [255, 255, 255] if go_flag else [0, 0, 255]
            cv2.rectangle(img, (x,y), (x+w, y+h), goc, 3)
            
        if go_flag:
            print("go_flag is " + str(go_flag))
        else:
            print("no")
            
        # 人間の関節描画
        for i in range(self.joint_size):
            cv2.circle(gray2, (int(self.joint_x[i]), int(self.joint_y[i])), 10, 255, thickness=3)
            cv2.circle(img, (int(self.joint_x[i]), int(self.joint_y[i])), 10, (255, 255, 255), thickness=3)
            
        cv2.imshow("img", img)
        # cv2.imshow("gray2", gray2)
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

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
