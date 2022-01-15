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
from jishupro.msg import *

# human estimatorと画像の色情報から、書類が適切な位置にあるかを識別する

jidx = {"ls":0, "rs":1, "le":2, "re":3, "lw":4, "rw":5}

class image_converter:

    def __init__(self):

        self.joint_names = ["left shoulder", "right shoulder", "left elbow", "right elbow", "left wrist", "right wrist"]
        self.joint_size = len(self.joint_names)
        self.joint_x = np.zeros(self.joint_size)
        self.joint_y = np.zeros(self.joint_size)
        self.joint_exist = np.zeros(self.joint_size)
        self.rw_state_estimator = 0
        self.lw_state_estimator = 0
        self.goal = 0
        self.rval = 0
        self.lval = 0
        self.nval = 0
        self.goalVal = 0
        
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=1)

        self.bridge = CvBridge()

        self.server = actionlib.SimpleActionServer('yoshimura', YoshimuraAction, self.execute, False)
        self.server.start()
        
        
    def execute(self, goal):
        print("goal is " + str(goal.yoshimura_goal)) # 1:hand 2:nade
        self.goal = goal.yoshimura_goal
        rate = rospy.Rate(10)
        # self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.callback) # PC
        self.image_sub = rospy.Subscriber("/image_raw",Image,self.callback) # usb camera
        self.pose_sub = rospy.Subscriber("/edgetpu_human_pose_estimator/output/poses", PeoplePoseArray, self.callback_poses) # PV, usb
        while self.goalVal == 0:
            rate.sleep()
            if rospy.is_shutdown():
                break
        self.image_sub.unregister()
        self.pose_sub.unregister()
        cv2.destroyAllWindows()
        result = self.server.get_default_result()
        if self.rval >= 40 & self.goal == 1:
            result.yoshimura_result = 1 # 1は右手
        elif self.lval >= 40 & self.goal == 1:
            result.yoshimura_result = 2 # 2は左手
        else:
            result.yoshimura_result = 3 # 3はなでる
        print ("result is " + str(result.yoshimura_result))
        self.rval = 0
        self.lval = 0
        self.nval = 0
        self.goalVal = 0
        self.server.set_succeeded(result)

    # 画像処理のコールバック関数
    def callback(self,data):
        try:
            img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = img.shape

        # xが横、yが縦

        # 右手
        # 肩と肘のx座標が近い
        v0 = abs(self.joint_x[jidx["re"]] - self.joint_x[jidx["rs"]])
        f0 = v0 < 50
        # 肩と肘のy座標が十分離れている
        v1 = self.joint_y[jidx["re"]] - self.joint_y[jidx["rs"]]
        f1 =  v1 > 120
        # 肘と手首のy座標が近い
        v2 = abs(self.joint_y[jidx["re"]] - self.joint_y[jidx["rw"]])
        f2 = v2 < 80

        # 右手首が映らないときの状態を推定する
        if self.joint_exist[jidx["rw"]] == 1:
            self.rw_state_estimator = min(3, self.rw_state_estimator+1)
        else:
            self.rw_state_estimator = max(-3, self.rw_state_estimator-1)

        fw = self.rw_state_estimator > 0

        # print(str(f0) + " " + str(f1) + " " + str(f2) + " " + str(fw))
        # print(str(v0) + " " + str(v1) + " " + str(v2))
        r_flag = f0 & f1 & f2 & fw

        # 左手
        # 肩と肘のx座標が近い
        w0 = abs(self.joint_x[jidx["le"]] - self.joint_x[jidx["ls"]])
        g0 = w0 < 50
        # 肩と肘のy座標が十分離れている
        w1 = self.joint_y[jidx["le"]] - self.joint_y[jidx["ls"]]
        g1 =  w1 > 120
        # 肘と手首のy座標が近い
        w2 = abs(self.joint_y[jidx["le"]] - self.joint_y[jidx["lw"]])
        g2 = w2 < 80

        # 左手首が映らないときの状態を推定する
        if self.joint_exist[jidx["lw"]] == 1:
            self.lw_state_estimator = min(3, self.lw_state_estimator+1)
        else:
            self.lw_state_estimator = max(-3, self.lw_state_estimator-1)

        gw = self.lw_state_estimator > 0

        l_flag = g0 & g1 & g2 & gw

        if r_flag:
            self.rval += 1
        else:
            self.rval = max(0, self.rval-1)

        if l_flag:
            self.lval += 1
        else:
            self.lval = max(0, self.lval-1)

        if (self.rval >= 40 or self.lval >= 40) and self.goal == 1:
            self.goalVal = 1
            
        # なでる
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        gray = np.zeros((rows, cols))
        gray[(hsv[:,:,0] < 40) & (hsv[:,:,1] > 40)] = 255
        gray[240:] = 0
        nadenade = np.count_nonzero(gray == 255)
        if nadenade > 100000:
            self.nval += 1
        else:
            self.nval = max(0, self.nval-1)

        if self.nval >= 20 and self.goal == 2:
            self.goalVal = 1
            
        # 人間の関節描画
        for i in range(self.joint_size):
            joint_color = (255 if self.joint_exist[i]==1 else 0)
            cv2.circle(img, (int(self.joint_x[i]), int(self.joint_y[i])), 10, (255, 0, joint_color), thickness=3)
            
        cv2.imshow("img", img)
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
                self.joint_exist[i] = 1
            else:
                self.joint_exist[i] = 0

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
