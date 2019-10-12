#!/usr/bin/env python
import cv2
import threading
import Queue as que
import time
import numpy as np

import roslib
import sys
import rospy

import importlib
import cPickle
import genpy.message
from rospy import ROSException
import sensor_msgs.msg
import actionlib
import rostopic
import rosservice
from rosservice import ROSServiceException

from slidewindow import SlideWindow
from warper import Warper
from pidcal import PidCal

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

warper = Warper()
slidewindow  = SlideWindow()
pidcal = PidCal()

q1 = que.Queue()
bridge = CvBridge()

cv_image = None
ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
car_run_speed = 0.5
left = None
right = None

def k_move(linear, angle):
    ack_msg = AckermannDriveStamped()
    ack_msg.header.stamp = rospy.Time.now()
    ack_msg.header.frame_id = ''
    ack_msg.drive.steering_angle = angle
    ack_msg.drive.speed = linear
    #ack_msg.drive.steering_angle = 0
    #ack_msg.drive.speed = 0
    ack_publisher.publish(ack_msg)

def lds_callback(data):
    global ranges_list
    global right
    global left
    ranges_list = data.ranges
    right = min(ranges_list[90:119])
    left = min(ranges_list[59:89])

def img_callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

ranges_list = []
mode = 0
count = 0
race_cnt = 0

def auto_drive(pid):
    global car_run_speed
    global mode
    global count
    global race_cnt
    global right
    global left
    straight = 0.065
    curve = 0.01
    w = 0
    
    if -0.065 < pid and pid < 0.065:
        w = 1.5
    # elif -0.065 < pid and pid < 0.065:
    #     w = 1
    else:
        w = 0.8
        curve = 0.015

    #right = min(ranges_list[89:119])
    #left = min(ranges_list[69:90])

    if abs(pid) < straight:
        if count > 9:
            mode += 1
        count = 0
    count += abs(pid) > 0.095

    print("mode : ", mode)
    print("race_cnt : ", race_cnt)

    if mode == 2:
        car_run_speed = 0.6
    if car_run_speed < 1.0 * w:
        car_run_speed += 0.005 * 10
    else:
        car_run_speed -= curve * 10
    
    #if right <= 0.2 and mode == 2:
    #    print("right")
    #    pid = 0.2
    #if left <= 0.2 and mode == 2:
    #    print("left")
    #    pid = -0.2

    return car_run_speed, pid

def detect_stop_line_black_yellow(frame):
    img = cv2.cvtColor(warper.warp(frame.copy()), cv2.COLOR_BGR2HSV)
    lab_black = [np.array([90, 0, 0]), 
                 np.array([111, 90, 180])]

    lab_yellow = [np.array([20, 50, 100]),
                  np.array([40, 255, 200])]

    mask_lab_black = cv2.inRange(img, lab_black[0], lab_black[1])
    mask_lab_yellow = cv2.inRange(img, lab_yellow[0], lab_yellow[1])

    mask = mask_lab_black
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = mask[350:, 50:640-50]

    mask2 = mask_lab_yellow
    kernel = np.ones((3, 3), np.uint8)
    mask2 = cv2.dilate(mask2, kernel, iterations=3)
    mask2 = cv2.erode(mask2, kernel, iterations=1)
    mask2 = mask2[350:, 50:640-50]

    frame = frame[350:, 50:640-50]

    #return frame, mask, mask2

    d_w = cv2.countNonZero(mask) * 1000 / (mask.shape[0] * mask.shape[1])
    d_y = cv2.countNonZero(mask2) * 1000 / (mask2.shape[0] * mask2.shape[1])
	
    return d_w, d_y
 

def main():
    global cv_image
    global ack_publisher
    global race_cnt
    global mode
    global ranges_list
    global right
    global left
    rospy.sleep(3)
    bridge = CvBridge()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    lds_sub = rospy.Subscriber("/scan", LaserScan, lds_callback)
    rospy.init_node('auto_xycar', anonymous=True)

    #ack_publisher = rospy.Publisher('vesc/low_level/ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    # ack_publisher = rospy.Publisher('ackermann_cmd_mux/input/teleop', AckermannDriveStamped, queue_size=1)
    # record the origin
    #out = cv2.VideoWriter('/home/nvidia/Desktop/outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    # record the processed
    #out2 = cv2.VideoWriter('/home/nvidia/Desktop/oripy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (640,480))

    while cv_image != None:
      img1, x_location = process_image(cv_image)
      cv2.imshow('result', img1)
      #bin1, bin2 = detect_stop_line_black_yellow(cv_image)
      #cv2.imshow("origin", org)
      #cv2.imshow("yellowbin", bin2)
      if x_location != None:
          x_location -= 20
          #print("left : ", left, " right : ", right)
          #if right <= 0.2 and mode == 2:
          #  print("right")
          #  x_location -= 100
          #if left <= 0.2 and mode == 2:
          #  print("left")
          #  x_location += 100
          pid = round(pidcal.pid_control(int(x_location)), 6)
          d_w, d_y = detect_stop_line_black_yellow(cv_image)
        #   print("dw, dy : ", d_w, d_y)
          #print("pid : ", pid)
          linear, angle = auto_drive(pid)

          if d_w > 18 and d_y > 2 and mode >= 3:
              race_cnt += 1

          if d_w > 18 and d_y > 2:
              mode = 0
          
          if right <= 0.4 and mode == 2:
              rospy.sleep(rospy.Duration(0.1))
              k_move(0.6, 0.3)
              rospy.sleep(rospy.Duration(1))
              k_move(0.6, -0.34)
              rospy.sleep(rospy.Duration(1))
   
          if left <= 0.4 and mode == 2:
              rospy.sleep(rospy.Duration(0.1))
              k_move(0.6, -0.3)
              rospy.sleep(rospy.Duration(1))
              k_move(0.6, 0.34)
              rospy.sleep(rospy.Duration(1))


          if d_w > 18 and d_y > 2 and race_cnt == 3:
              #print("\n\nstop---------------------\n\n", "dw:", d_w, "dy", d_y)
              rospy.sleep(0.3)
              k_move(0 ,0)
              rospy.sleep(60)
        
          else:
              k_move(linear, angle)        
      if cv2.waitKey(1) & 0xFF == ord('q'):
          break
      cv2.imshow("origin", cv_image)
      #out.write(img1)
      #out2.write(cv_image)

    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows() 

def process_image(frame):
    
    # grayscle
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)
    # canny edge
    low_threshold = 60#60
    high_threshold = 70# 70
    edges_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # warper
    img = warper.warp(edges_img)
    img1, x_location = slidewindow.slidewindow(img)
    
    return img1, x_location

if __name__ == '__main__':
    main()
