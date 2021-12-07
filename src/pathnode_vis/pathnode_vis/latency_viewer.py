#!env python3
# Copyright 2021 Research Institute of Systems Planning, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pickle
from statistics import mean

import rclpy
from rclpy.node import Node

from pathnode_vis.pubinfo_traverse import TopicGraph, InputSensorStampSolver
from path_info_msg.msg import PubInfo, PubTopicTimeInfo, SubTopicTimeInfo
from pathnode_vis.pub_info import PubInfos as PubInfosObj, PubInfo as PubInfoObj, time2str
import pathnode_vis.pub_info

PUB_INFO_TOPICS = [
    # "/diagnostics/info/pub",
    "/initialpose3d/info/pub",
    "/localization/debug/ellipse_marker/info/pub",
    "/localization/pose_estimator/exe_time_ms/info/pub",
    "/localization/pose_estimator/initial_pose_with_covariance/info/pub",
    "/localization/pose_estimator/initial_to_result_distance/info/pub",
    "/localization/pose_estimator/initial_to_result_distance_new/info/pub",
    "/localization/pose_estimator/initial_to_result_distance_old/info/pub",
    "/localization/pose_estimator/iteration_num/info/pub",
    "/localization/pose_estimator/monte_carlo_initial_pose_marker/info/pub",
    "/localization/pose_estimator/ndt_marker/info/pub",
    "/localization/pose_estimator/points_aligned/info/pub",
    "/localization/pose_estimator/pose/info/pub",
    "/localization/pose_estimator/pose_with_covariance/info/pub",
    "/localization/pose_estimator/transform_probability/info/pub",
    "/localization/pose_twist_fusion_filter/debug/info/pub",
    "/localization/pose_twist_fusion_filter/debug/measured_pose/info/pub",
    "/localization/pose_twist_fusion_filter/estimated_yaw_bias/info/pub",
    "/localization/pose_twist_fusion_filter/pose/info/pub",
    "/localization/pose_twist_fusion_filter/pose_with_covariance/info/pub",
    "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias/info/pub",
    "/localization/pose_twist_fusion_filter/pose_without_yawbias/info/pub",
    "/localization/pose_twist_fusion_filter/twist/info/pub",
    "/localization/pose_twist_fusion_filter/twist_with_covariance/info/pub",
    "/localization/twist_estimator/twist/info/pub",
    "/localization/twist_estimator/twist_with_covariance/info/pub",
    "/localization/util/crop_box_filter_measurement_range/crop_box_polygon/info/pub",
    "/localization/util/downsample/pointcloud/info/pub",
    "/localization/util/measurement_range/pointcloud/info/pub",
    "/localization/util/voxel_grid_downsample/pointcloud/info/pub",
    "/sensing/gnss/fixed/info/pub",
    "/sensing/gnss/pose/info/pub",
    "/sensing/gnss/pose_with_covariance/info/pub",
    "/sensing/imu/imu_data/info/pub",
    "/sensing/lidar/concatenate_data/concat_num/info/pub",
    "/sensing/lidar/concatenate_data/not_subscribed_topic_name/info/pub",
    "/sensing/lidar/concatenated/pointcloud/info/pub",
    "/sensing/lidar/crop_box_filter/crop_box_polygon/info/pub",
    "/sensing/lidar/front_left/mirror_crop_box_filter/crop_box_polygon/info/pub",
    "/sensing/lidar/front_left/mirror_cropped/pointcloud/info/pub",
    "/sensing/lidar/front_left/self_crop_box_filter/crop_box_polygon/info/pub",
    "/sensing/lidar/front_left/self_cropped/pointcloud/info/pub",
    "/sensing/lidar/front_right/mirror_crop_box_filter/crop_box_polygon/info/pub",
    "/sensing/lidar/front_right/mirror_cropped/pointcloud/info/pub",
    "/sensing/lidar/front_right/self_crop_box_filter/crop_box_polygon/info/pub",
    "/sensing/lidar/front_right/self_cropped/pointcloud/info/pub",
    "/sensing/lidar/left/crop_box_filter_mirror/crop_box_polygon/info/pub",
    "/sensing/lidar/left/crop_box_filter_self/crop_box_polygon/info/pub",
    "/sensing/lidar/left/mirror_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/left/outlier_filtered/pointcloud/info/pub",
    "/sensing/lidar/left/self_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/measurement_range_cropped/pointcloud/info/pub",
    "/sensing/lidar/no_ground/pointcloud/info/pub",
    "/sensing/lidar/rear/crop_box_filter_mirror/crop_box_polygon/info/pub",
    "/sensing/lidar/rear/crop_box_filter_self/crop_box_polygon/info/pub",
    "/sensing/lidar/rear/mirror_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/rear/outlier_filtered/pointcloud/info/pub",
    "/sensing/lidar/rear/self_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/right/crop_box_filter_mirror/crop_box_polygon/info/pub",
    "/sensing/lidar/right/crop_box_filter_self/crop_box_polygon/info/pub",
    "/sensing/lidar/right/mirror_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/right/outlier_filtered/pointcloud/info/pub",
    "/sensing/lidar/right/self_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/top/crop_box_filter_mirror/crop_box_polygon/info/pub",
    "/sensing/lidar/top/crop_box_filter_self/crop_box_polygon/info/pub",
    "/sensing/lidar/top/mirror_cropped/pointcloud_ex/info/pub",
    "/sensing/lidar/top/outlier_filtered/pointcloud/info/pub",
    "/sensing/lidar/top/self_cropped/pointcloud_ex/info/pub",
    ]
LEAVES = [
    "/initialpose",
    "/map/pointcloud_map",
    "/sensing/lidar/top/rectified/pointcloud",
    "/sensing/imu/imu_data",
    "/vehicle/status/twist",
    ]
PUB_INFO = "topic_infos.pkl"
GRAPH_PKL = "graph.pkl"
TIMER_SEC = 1.0
TARGET_TOPIC = "/sensing/lidar/concatenated/pointcloud"

class LatencyStat(object):
    def __init__(self):
        self.dur_ms_list = []
        self.is_leaf_list = []

    def add(self, dur_ms, is_leaf):
        self.dur_ms_list.append(dur_ms)
        self.is_leaf_list.append(is_leaf)

    def report(self):
        dur_ms_list = self.dur_ms_list
        is_leaf_list = self.is_leaf_list

        dur_min = float(min(dur_ms_list))
        dur_mean = float(mean(dur_ms_list))
        dur_max = float(max(dur_ms_list))

        is_all_leaf = all(is_leaf_list)

        return {
            "dur_min": dur_min,
            "dur_mean": dur_mean,
            "dur_max": dur_max,
            "is_all_leaf": is_all_leaf,
            }

class PerTopicLatencyStat(object):
    def __init__(self):
        self.data = {}

    def add(self, topic, dur_ms, is_leaf):
        self.data.setdefault(topic, LatencyStat()).add(dur_ms, is_leaf)

    def report(self):
        ret = {}
        for (topic, stat) in self.data.items():
            ret[topic] = stat.report()
        return ret

    def print_report(self):
        reports = self.report()
        for (topic, report) in reports.items():
            print(f"{topic:80} {report['dur_min']:>6.1f} {report['dur_mean']:>6.1f} {report['dur_min']:>6.1f} {report['is_all_leaf']}")

class LatencyViewerNode(Node):
    def __init__(self):
        super().__init__('latency_viewer_node')
        self.declare_parameter("topics", PUB_INFO_TOPICS)
        self.declare_parameter("leaves", LEAVES)
        self.declare_parameter("graph_pkl", GRAPH_PKL)
        self.declare_parameter("timer_sec", TIMER_SEC)
        self.declare_parameter("target_topic", TARGET_TOPIC)

        self.subs = {}
        topics = self.get_parameter("topics").get_parameter_value().string_array_value
        for topic in topics:
            sub = self.create_subscription(
                PubInfo,
                topic,
                self.listener_callback,
                1)
            self.subs[topic] = sub

        graph_pkl = self.get_parameter("graph_pkl").get_parameter_value().string_value
        graph = pickle.load(open(graph_pkl, "rb"))
        self.solver = InputSensorStampSolver(graph)

        self.pub_infos = PubInfosObj()

        timer_sec = self.get_parameter("timer_sec").get_parameter_value().double_value
        self.timer = self.create_timer(1.0,
                                       self.timer_callback)

        self.target_topic = self.get_parameter("target_topic").get_parameter_value().string_value

    def listener_callback(self, pub_info_msg):
        # print(f"{pub_info_msg.output_info.topic_name}")
        pub_info = PubInfoObj(pub_info_msg.output_info.topic_name, pub_info_msg.output_info.header_stamp)
        for input_info in pub_info_msg.input_infos:
            pub_info.add_input_info(input_info.topic_name,
                                    input_info.has_header_stamp,
                                    input_info.header_stamp)
        self.pub_infos.add(pub_info)

    def timer_callback(self):
        # print(f"timer_callback")
        pubinfos = self.pub_infos
        solver = self.solver
        target_topic = self.target_topic

        stamps = sorted(pubinfos.stamps(target_topic))
        # [-3] has no specific meaning.
        # But [-1] may not have full info. So we look somewhat old info.
        idx = -3
        if len(stamps) == 0:
            return
        elif len(stamps) < 3:
            idx = 0

        stats = PerTopicLatencyStat()
        for target_stamp in stamps[:idx]:
            results = solver.solve(pubinfos, target_topic, target_stamp)

            for r in results.data:
                stats.add(r.topic, r.dur_ms, r.is_leaf)
                # print(f"{r.topic:80} {r.stamp:>20} {r.dur_ms:4} ms {r.is_leaf} {r.parent}")

        stats.print_report()

def main(args=None):
    rclpy.init(args=args)
    node = LatencyViewerNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
