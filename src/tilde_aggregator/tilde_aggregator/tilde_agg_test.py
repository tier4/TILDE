#!/usr/bin/env /usr/bin/python3
# -*- coding: utf-8 -*-

import os, sys, time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from .tilde_agg_func import *
from .tilde_agg_com import *

from tilde_msg.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from autoware_auto_vehicle_msgs.msg import VelocityReport

g_test_topic = []
def test_init(node, mode):
    global g_test_topic
    if mode == 'TESTA':
        g_test_topic = [ \
            { \
                'topic': "/localization/pose_twist_fusion_filter/biased_pose_with_covariance/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '1_MTT    ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/for_tilde_interpolater_pose_with_covariance/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '2_ORG_MTT    ', \
            }, \
            { \
                'topic': "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': 'ORG:EKF-pose_with_covariance          ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/for_tilde_interpolater_pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': 'ORG:NDT-for_tilde_interpolater_pose_with_covariance       ', \
            }, \
        ]

    if mode == 'TEST0':
        g_test_topic = [ \
            { \
                'topic': "/localization/util/downsample/pointcloud/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '1_MTT    ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/pose_with_covariance/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '2_MTT    ', \
            }, \
            { \
                'topic': "/localization/util/downsample/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': 'ORG:SensorPoints:downsample/pcl          ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/initial_pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': 'ORG:EKF-InitialPoseWithCovarianceStamped ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': 'ORG:NDT-PoseWithCovarianceStamped       ', \
            }, \
            #{ \
            #    'topic': "/localization/pose_twist_fusion_filter/estimated_yaw_bias", \
            #    'mtype': type(PoseWithCovarianceStamped()), \
            #    'prev': 0.0, \
            #    'dra': 0.0, \
            #    'num': 0, \
            #    'name': 'ORG:EKF-estimated_yaw_bias ', \
            #}, \
        ]
    ##############################
    if mode == 'TEST1':
        g_test_topic = [ \
            { \
                'topic': "/localization/util/downsample/pointcloud/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '1_MTT    ', \
            }, \
            { \
                'topic': "/localization/util/downsample/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 1_ORG   ', \
            }, \
            { \
                'topic': "/localization/util/voxel_grid_downsample/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 1_SUB_1 ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/pose_with_covariance/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '2_MTT    ', \
            }, \
            { \
                'topic': "/localization/pose_estimator/pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_ORG   ', \
            }, \
            { \
                'topic': "/sensing/gnss/pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_1 ', \
            }, \
            { \
                'topic': "/map/pointcloud_map", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_2 ', \
            }, \
            { \
                'topic': "/localization/util/downsample/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_3 ', \
            }, \
            { \
                'topic': "/localization/pose_twist_fusion_filter/biased_pose_with_covariance", \
                'mtype': type(PoseWithCovarianceStamped()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_4 ', \
            }, \
        ]
    ##############################
    if mode == 'TEST2':
        g_test_topic = [ \
            { \
                'topic': "/sensing/lidar/top/outlier_filtered/pointcloud/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '1_MTT    ', \
            }, \
            { \
                'topic': "/sensing/lidar/top/outlier_filtered/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 1_ORG   ', \
            }, \
            { \
                'topic': "/sensing/lidar/top/rectified/pointcloud_ex", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 1_SUB_1 ', \
            }, \
            { \
                'topic': "/sensing/lidar/concatenated/pointcloud/message_tracking_tag", \
                'mtype': type(MessageTrackingTag()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': '2_MTT    ', \
            }, \
            { \
                'topic': "/sensing/lidar/concatenated/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_ORG   ', \
            }, \
            { \
                'topic': "/sensing/lidar/left/outlier_filtered/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_1 ', \
            }, \
            { \
                'topic': "/sensing/lidar/right/outlier_filtered/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_2 ', \
            }, \
            { \
                'topic': "/sensing/lidar/top/outlier_filtered/pointcloud", \
                'mtype': type(PointCloud2()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_3 ', \
            }, \
            { \
                'topic': "/vehicle/status/velocity_status", \
                'mtype': type(VelocityReport()), \
                'prev': 0.0, \
                'dra': 0.0, \
                'num': 0, \
                'name': ' 2_SUB_4 ', \
            }, \
        ]
    test_sub(node)
    #PP(f"{g_test_topic=}")


def test_sub(node):
    global g_test_topic
    for w in g_test_topic:
        w['prev'] = time.perf_counter()
        if 'MTT' in w['name']:
            continue
        tpname = w['topic']
        mtype = w['mtype']
        profile = QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=5)
        #print(f"{location()}: create_sub {tpname}")
        sub = node.create_subscription(
                                        mtype, 
                                        tpname, 
                                        node.mtt_topic_callback_wrapper(tpname, None, None, w), 
                                        profile
                                        )

def test_top_topic_cb(self, topic_name, msg):
    global g_test_topic
    w = g_test_topic[0]
    cur = time.perf_counter()
    duration = cur - w['prev']
    w['prev'] = cur
    w['dra'] = duration
    pub_time = stamp_to_sec(msg.output_info.header_stamp)
    print(f"|{(pub_time):.9f}|{duration:2.4f}|{w['num']:4d} |{w['name']:10s}|{topic_name}", flush=True)
    w['num'] += 1

def test_topic_cb(self, topic_name, msg, test_info):
    global g_test_topic
    for w in g_test_topic:
        if w == test_info:
            break
        if MTT_TOPIC_POSTFIX in topic_name:
            if w['topic'] == topic_name:
                test_info = w
                break
    else:
        return True
    w = test_info
    if w['topic'] == topic_name:
        cur = time.perf_counter()
        duration = cur - w['prev']
        w['prev'] = cur
        w['dra'] = duration
        if 'SUB' in w['name']:
            pub_time = stamp_to_sec(msg.header.stamp)
        elif 'ORG' in w['name']:
            pub_time = stamp_to_sec(msg.header.stamp)
        elif 'MTT' in w['name']:
            pub_time = stamp_to_sec(msg.output_info.header_stamp)
        print(f"|{(pub_time):.9f}|{duration:2.4f}|{w['num']:4d} |{w['name']:10s}|{topic_name}", flush=True)
        w['num'] += 1
        if 'MTT' not in w['name']:
            return False
    return True

##########
recv_topics = {}
target = []
def default_callback_wrapper(topic_name):
    DP(f"{location()}: [SET] {topic_name=}")
    def callback(msg):
        tm = stamp_to_sec(msg.output_info.pub_time)
        recv_topics.update({topic_name: tm})
        #self.get_logger().info(f"[{topic_name}: {msg}")
    return callback

def default_timer_callback():
    diff = set(target) - set(recv_topics)
    if len(diff) == 0:
        PP(f"\n\n!!! completed {recv_topics}\n^\n")
    else:
        PP(f"\n### Unreceived {diff}\n")

def check1_recv_topic(node, profile):
    for pinfo in node.static_path_list:
        target.append(pinfo.top_topic.name)
        for w in pinfo.topic_dict.keys():
            target.append(w)

    while(True):
        exclude = []
        cur_topics = [w[0] for w in node.get_topic_names_and_types()]
        if set(target) == set(cur_topics):
            PP(f"!!! topics ready {target}")
            break
        else:
            for w in target:
                if not w in cur_topics:
                    exclude.append(w)
            PP(f"[EXCLUDE] {exclude}")

        time.sleep(5)
    for w in target:
        PP(f"---sub: {w}")
        sub = node.create_subscription(
                                        MessageTrackingTag, 
                                        w, 
                                        default_callback_wrapper(w),
                                        profile
                                    )
    node.create_timer(5.0, default_timer_callback)

def check2_recv_topic(node, profile):
    for pinfo in node.static_path_list:
        target.append(pinfo.top_topic.name)
        for w in pinfo.topic_dict.keys():
            target.append(w)

    for tn in target:
        PP(f"--- SUB {tn}")
        sub = node.create_subscription(
                                        MessageTrackingTag, 
                                        tn, 
                                        default_callback_wrapper(tn),
                                        profile
                                    )
    node.create_timer(5.0, default_timer_callback)
