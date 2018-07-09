#!/home/ubuntu/anaconda2/bin/python2.7
# coding=utf-8
# !/usr/bin/python2.7
# !/home/ubuntu/anaconda2/bin/python2.7
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import rospy
import roslib
import time
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

import cv2
import sys
import os
import glob
import tf
import tf2_ros
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import Odometry
from gf_perception.msg import ObjectList
from gf_perception.msg import Object

import tensorflow as tf
from config import *
from train import _draw_box
from nets import *

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# from sklearn.cluster import KMeans


frame_rate_list = np.zeros(10)
count = 0
frame_rate_idx = 0
frame_rate = 0.0
flag = True
qn_odom = [0, 0, 0, 0]
team_x = team_y = team_relative_x = team_relative_y = 0
odom_pos_x = odom_pos_y = 0
odom_yaw = odom_pos_x = odom_pos_y = odom_vel_x = odom_vel_y = 0
last_enemy_position = ObjectList()
connection_status = team_hp = global_team_x = global_team_y = 0

lose_frame_count = 0

video = cv2.VideoWriter('/home/ubuntu/GaoFen_Drone/src/gf_perception/scripts/visual/demo.avi',
                        cv2.VideoWriter_fourcc(*"MJPG"),
                        20,
                        (640, 480))

def DetectInit():
    global sess, model, mc

    detect_net = 'squeezeDet'
    checkpoint = '/home/ubuntu/GaoFen_Drone/src/gf_perception/scripts/weights/model.ckpt-69000'

    assert detect_net == 'squeezeDet' or detect_net == 'squeezeDet+', 'Selected nueral net architecture not supported'

    tf.Graph().as_default()
    # Load model
    if detect_net == 'squeezeDet':
        mc = kitti_squeezeDet_config()
        mc.BATCH_SIZE = 1
        # model parameters will be restored from checkpoint
        mc.LOAD_PRETRAINED_MODEL = False
        model = SqueezeDet(mc, '0')
    elif detect_net == 'squeezeDet+':
        mc = kitti_squeezeDetPlus_config()
        mc.BATCH_SIZE = 1
        mc.LOAD_PRETRAINED_MODEL = False
        model = SqueezeDetPlus(mc, '0')

    saver = tf.train.Saver(model.model_params)
    # Use jit xla
    gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.5)
    config = tf.ConfigProto(allow_soft_placement=True, gpu_options=gpu_options)
    config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
    # with tf.Session(config=config) as sess:
    sess = tf.Session(config=config)
    saver.restore(sess, checkpoint)
    

def callback_rgb(rgb):
    global rgb_image, count
    global sess, model, mc, video, frame_rate_list, frame_rate_idx, frame_rate, odom_yaw, odom_pos_x, odom_pos_y, odom_vel_x, odom_vel_y
    global last_enemy_position, lose_frame_count

    # 1. Get rgb images
    bridge = CvBridge()
    try:
        rgb_image = bridge.imgmsg_to_cv2(rgb, desired_encoding="8UC3")
    except CvBridgeError as error:
        print(error)
    
    count = count + 1
    times = {}
    t_start = time.time()

    # 2. Preprocessing 
    im = rgb_image
    im = im.astype(np.float32, copy=False)
    im = cv2.resize(im, (mc.IMAGE_WIDTH, mc.IMAGE_HEIGHT))
    input_image = im - mc.BGR_MEANS

    t_reshape = time.time()
    times['reshape'] = t_reshape - t_start

    # 3. Use SqueezeDet to detect number
    det_boxes, det_probs, det_class = sess.run([model.det_boxes, model.det_probs, model.det_class], feed_dict={model.image_input: [input_image]})
    
    t_detect = time.time()
    times['detect'] = t_detect - t_reshape

    # 4. NMS and delete bounding boxes by threshold
    final_boxes, final_probs, final_class = model.filter_prediction(det_boxes[0],
                                                                    det_probs[0],
                                                                    det_class[0])
    keep_idx = np.squeeze(np.argwhere(np.array(final_probs) > mc.PLOT_PROB_THRESH))

    final_boxes = np.array(final_boxes)[keep_idx, :]
    final_probs = np.array(final_probs)[keep_idx]
    final_class = np.array(final_class)[keep_idx]

    # 如果没有检测到任何物体, 对存储检测结果的list进行变换
    if keep_idx.shape == ():
        final_boxes = final_boxes[np.newaxis, :]
        final_probs = np.array([final_probs])
        final_class = np.array([final_class])

    print("final_boxes: {}, final_probs: {}, final_class: {}".format(final_boxes, final_probs, final_class))

    detection_pub_list = ObjectList()
    detection_pub_list.header.stamp = rospy.Time.now()
    detection_pub_list.header.frame_id = 'detection_number'
    detection_pub = Object()
    # 5. if detected -> enter if, else -> enter empty 
    if len(final_boxes) > 0:
        # detected number
        count = final_boxes.shape[0]
        for idx in range(count):
            box = final_boxes[idx, :]
            cx, cy, w, h = box[0], box[1], box[2], box[3]
            detection_pub.number = final_class[idx]
            detection_pub.center.x = cx
            detection_pub.center.y = cy
            detection_pub.size.x = w
            detection_pub.size.y = h
            detection_pub_list.count = count
            detection_pub_list.object.append(detection_pub)
    else:
        detection_pub_list.count = 0
        detection_pub_list.object = []
    pub.publish(detection_pub_list)

    # 6. Print time
    t_filter = time.time()
    times['filter'] = t_filter - t_detect
    times['total'] = time.time() - t_start
    time_str = 'Total time: {:.4f}, detection time: {:.4f}, filter time: {:.4f}'.format(times['total'], times['detect'], times['filter'])
    print(time_str)

    # 7. Visual results
    cls2clr = {'1': (0, 0, 255), 
               '2': (0, 255, 0), 
               '3': (255, 0, 0),
               '4': (255, 255, 0),
               '5': (0, 255, 255),
               '6': (255, 0, 255),
               '7': (255, 255, 255),
               '8': (200, 0, 0),
               '9': (0, 200, 0),
               '10': (0, 0, 200)}
    if mc.VISUAL:
        im = _draw_box(im, final_boxes, [mc.CLASS_NAMES[idx] + ':%.2f' % prob for idx, prob in zip(final_class, final_probs)], cdict=cls2clr)

        if mc.DRAW_Video:
            im = im.astype('uint8')
            video.write(im)
            
        if mc.SHOW:
            im = im.astype('uint8')
            cv2.imshow('demo', im)
            cv2.waitKey(3)

rospy.init_node('gaofen_detection')
DetectInit()

subrgb = rospy.Subscriber('/airsim/image/front/rgb', Image, callback_rgb)
pub = rospy.Publisher('gaofen_detection/number', ObjectList, queue_size=1)

rospy.spin()
video.release()
cv2.destroyAllWindows()
