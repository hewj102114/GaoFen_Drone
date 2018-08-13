#!/home/ubuntu/anaconda2/bin/python2.7
#coding=utf-8
import cv2
import numpy as np
import rospy
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3Stamped
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt
from skimage import img_as_float,img_as_ubyte
brightness_list=[]

def callback(msg):
    global image
    bridge = CvBridge()
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)  
    image=cv2.medianBlur(image,5)
    image_ycrcb=cv2.cvtColor(image,cv2.COLOR_BGR2YCrCb)
    
    image_float=img_as_float(image_ycrcb[:,:,0])
    imgmean=np.mean(image_float)
    brightness_list.append(imgmean)
    if len(brightness_list)>5:
        brightness_list.pop(0)
    median_brightness=np.median(brightness_list)
    # print(median_brightness,brightness_list)

    image_float = image_float+median_brightness-imgmean
    image_ycrcb[:,:,0]=img_as_ubyte(image_float)
    image1=cv2.cvtColor(image_ycrcb,cv2.COLOR_YCR_CB2BGR)
    lower_hsv = np.array([130, 130, 130])
    upper_hsv = np.array([195, 195, 195])
    brightness_mask = cv2.inRange(image1, lower_hsv, upper_hsv)
    brightness_mask=cv2.medianBlur(brightness_mask,5)
    imgmax=np.amax(image1,axis=2)
    imgmin=np.amin(image1,axis=2)
    ret, delta_thresh = cv2.threshold(
                imgmax-imgmin, 8, 255, cv2.THRESH_BINARY_INV)
    mask=delta_thresh & brightness_mask
    mask=cv2.blur(mask,(5,5))
    cv2.imshow("mask",delta_thresh)
    cv2.imshow("mask22",brightness_mask)
    cv2.imshow("mask1",mask)

    drawing=image1.copy()
    imgg ,contours,hierarchy = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    img_contour=np.zeros((480,640,1),np.uint8)
    new_contours=[]
    for contour in contours:
        if len(contour)>300:
            print '####',len(contour)
            new_contours.append(contour)
    img_contour = cv2.drawContours(img_contour,new_contours,-1,(255,255,255),-1)
    edges = cv2.Canny(img_contour, 50, 150)
    cv2.imshow("b",img_contour)
    # lines = cv2.HoughLinesP(img_contour, 1, np.pi / 180, 50,minLineLength=80, maxLineGap=5)
    
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD,0.5)
    lines = lsd.detect(img_contour)[0]
    left_lines=[]
    right_lines=[]
    # print '#',len(lines)
    for line in lines:
        x1, y1, x2, y2 = line[0]
        length=math.sqrt(pow(x1-x2,2)+pow(y1-y2,2))
        cv2.line(drawing, (x1, y1), (x2, y2), (0, 0, 0), 1, lineType=cv2.LINE_AA)
        if abs(y1-y2)>5 and abs(x1-x2)>5 and length>50:
            # print(abs((y1-y2)*1.0/(x1-x2)-(240-y2)*1.0/(320-x2)))
            
            if y1<300 and y2<300 and min(x1,x2)<300 and max(x1,x2)<400:
                left_lines.append(line)
                cv2.line(drawing, (x1, y1), (x2, y2), (0, 255, 0), 1, lineType=cv2.LINE_AA)
            if y1<300 and y2<300 and min(x1,x2)>640-400 and max(x1,x2)>640-300:
                right_lines.append(line)
                cv2.line(drawing, (x1, y1), (x2, y2), (255, 0, 0), 1, lineType=cv2.LINE_AA)
    avg_k_l=0
    for left_line in left_lines:
       x1, y1, x2, y2 = left_line[0]
       avg_k_l+=(y1-y2)*1.0/(x1-x2)
    #    print x1,x2,y1,y2,avg_k_l
    if len(left_lines):
        avg_k_l=avg_k_l/len(left_lines)
    
    avg_k_r=0
    for right_line in right_lines:
       x1, y1, x2, y2 = right_line[0]
       avg_k_r+=(y1-y2)*1.0/(x1-x2)
    if len(right_lines):
        avg_k_r=avg_k_r/len(right_lines)

    print avg_k_l,avg_k_r

    cv2.imshow("dd",drawing)
    cv2.waitKey(1)
    # plt.show()



rospy.init_node('lsd')
subrgb = rospy.Subscriber('/airsim/image/front/rgb', Image, callback)
# pub = rospy.Publisher('airsim/depth/circle', Vector3Stamped, queue_size=1)
# pub_err = rospy.Publisher('airsim/depth/error', Int16, queue_size=1)
rospy.spin()

