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
from builtin_interfaces.msg import Time as TimeMsg

from path_info_msg.msg import PubInfo
from pathnode_vis.pubinfo_traverse import TopicGraph, InputSensorStampSolver
from pathnode_vis.pub_info import (
    PubInfo as PubInfoObj,
    PubInfos as PubInfosObj
    )

EXCLUDES_TOPICS = [
    "/diagnostics/info/pub",
    ]
LEAVES = [
    "/initialpose",
    "/map/pointcloud_map",
    "/sensing/lidar/top/rectified/pointcloud",
    "/sensing/imu/imu_data",
    "/vehicle/status/twist",
    ]
PUB_INFO = "topic_infos.pkl"
TIMER_SEC = 1.0
TARGET_TOPIC = "/sensing/lidar/concatenated/pointcloud"
STOPS = [
    "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias",
    ]


class LatencyStat(object):
    def __init__(self):
        self.dur_ms_list = []
        self.dur_pub_ms_list = []
        self.dur_pub_ms_steady_list = []
        self.is_leaf_list = []

    def add(self, r):
        """
        Parameters
        ----------
        r: pubinfo_traverse.SolverResults
        """
        self.dur_ms_list.append(r.dur_ms)
        self.dur_pub_ms_list.append(r.dur_pub_ms)
        self.dur_pub_ms_steady_list.append(r.dur_pub_ms_steady)
        self.is_leaf_list.append(r.is_leaf)

    def report(self):
        dur_ms_list = self.dur_ms_list
        is_leaf_list = self.is_leaf_list
        dur_pub_ms_list = self.dur_pub_ms_list
        dur_pub_ms_steady_list = self.dur_pub_ms_steady_list

        dur_min = float(min(dur_ms_list))
        dur_mean = float(mean(dur_ms_list))
        dur_max = float(max(dur_ms_list))

        dur_pub_min = float(min(dur_pub_ms_list))
        dur_pub_mean = float(mean(dur_pub_ms_list))
        dur_pub_max = float(max(dur_pub_ms_list))

        dur_pub_steady_min = float(min(dur_pub_ms_steady_list))
        dur_pub_steady_mean = float(mean(dur_pub_ms_steady_list))
        dur_pub_steady_max = float(max(dur_pub_ms_list))

        is_all_leaf = all(is_leaf_list)

        return {
            "dur_min": dur_min,
            "dur_mean": dur_mean,
            "dur_max": dur_max,
            "dur_pub_min": dur_pub_min,
            "dur_pub_mean": dur_pub_mean,
            "dur_pub_max": dur_pub_max,
            "dur_pub_steady_min": dur_pub_steady_min,
            "dur_pub_steady_mean": dur_pub_steady_mean,
            "dur_pub_steady_max": dur_pub_steady_max,
            "is_all_leaf": is_all_leaf,
            }


class PerTopicLatencyStat(object):
    def __init__(self):
        self.data = {}

    def add(self, r):
        """
        Parameters
        ----------
        r: pubinfo_traverse.SolverResults
        """
        self.data.setdefault(r.topic, LatencyStat()).add(r)

    def report(self):
        ret = {}
        for (topic, stat) in self.data.items():
            ret[topic] = stat.report()
        return ret

    def print_report(self):
        reports = self.report()
        s = "{:80} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}".format(
            "topic", "dur", "dur", "dur", "e2e", "e2e", "e2e", "e2e_s", "e2e_s", "e2e_s"
        )
        print(s)

        def p(v):
            if v > 1000:
                return "   inf"
            else:
                return "{:>6.1f}".format(v)

        for (topic, report) in reports.items():
            s = f"{topic:80} "
            s += f"{p(report['dur_min'])} "
            s += f"{p(report['dur_mean'])} "
            s += f"{p(report['dur_max'])} "
            s += f"{p(report['dur_pub_min'])} "
            s += f"{p(report['dur_pub_mean'])} "
            s += f"{p(report['dur_pub_max'])} "
            s += f"{p(report['dur_pub_steady_min'])} "
            s += f"{p(report['dur_pub_steady_mean'])} "
            s += f"{p(report['dur_pub_steady_max'])} "
            s += f"{report['is_all_leaf']}"
            print(s)


class LatencyViewerNode(Node):
    def __init__(self):
        super().__init__('latency_viewer_node')
        self.declare_parameter("excludes_topics", EXCLUDES_TOPICS)
        self.declare_parameter("leaves", LEAVES)
        self.declare_parameter("graph_pkl", "")
        self.declare_parameter("timer_sec", TIMER_SEC)
        self.declare_parameter("target_topic", TARGET_TOPIC)
        self.declare_parameter("keep_info_sec", 3)
        self.declare_parameter("wait_sec_to_init_graph", 10)
        self.declare_parameter("mode", "stat")
        self.declare_parameter("stops", STOPS)

        self.subs = {}
        excludes_topic = (
            self.get_parameter("excludes_topics")
            .get_parameter_value().string_array_value)
        topics = self.get_pub_info_topics(excludes=excludes_topic)
        for topic in topics:
            print(topic)
            sub = self.create_subscription(
                PubInfo,
                topic,
                self.listener_callback,
                1)
            self.subs[topic] = sub

        self.solver = None
        self.stops = (
            self.get_parameter("stops")
            .get_parameter_value().string_array_value)
        graph_pkl = (
            self.get_parameter("graph_pkl")
            .get_parameter_value().string_value)
        if graph_pkl:
            graph = pickle.load(open(graph_pkl, "rb"))
            self.solver = InputSensorStampSolver(graph)

        self.pub_infos = PubInfosObj()

        timer_sec = (
            self.get_parameter("timer_sec")
            .get_parameter_value().double_value)
        self.timer = self.create_timer(timer_sec,
                                       self.timer_callback)

        self.target_topic = (
            self.get_parameter("target_topic")
            .get_parameter_value().string_value)
        self.keep_info_sec = (
            self.get_parameter("keep_info_sec")
            .get_parameter_value().integer_value)
        self.wait_sec_to_init_graph = (
            self.get_parameter("wait_sec_to_init_graph").
            get_parameter_value().integer_value)
        self.wait_init = 0

        self.init_skips()

    def init_skips(self):
        """
        Definition of skips.
        See TopicGraph.__init__ comment.
        """
        skips = {}
        RECT_OUT_EX = "/sensing/lidar/{}/rectified/pointcloud_ex"
        RECT_OUT = "/sensing/lidar/{}/rectified/pointcloud"
        RECT_IN = "/sensing/lidar/{}/mirror_cropped/pointcloud_ex"

        # top
        for pos in ["top", "left", "right"]:
            skips[RECT_OUT_EX.format(pos)] = RECT_IN.format(pos)
            skips[RECT_OUT.format(pos)] = RECT_IN.format(pos)

        self.skips = skips

    def listener_callback(self, pub_info_msg):
        # print(f"{pub_info_msg.output_info.topic_name}")
        output_info = pub_info_msg.output_info
        pub_info = PubInfoObj(output_info.topic_name,
                              output_info.pub_time,
                              output_info.pub_time_steady,
                              output_info.header_stamp)
        for input_info in pub_info_msg.input_infos:
            pub_info.add_input_info(input_info.topic_name,
                                    input_info.sub_time,
                                    input_info.sub_time_steady,
                                    input_info.has_header_stamp,
                                    input_info.header_stamp)
        self.pub_infos.add(pub_info)

    def handle_stat(self, stamps):
        """Report statistics

        Paramters
        ---------
        stamps: stamps sorted by old-to-new order

        Returns
        -------
        """
        pubinfos = self.pub_infos
        target_topic = self.target_topic
        solver = self.solver
        stops = self.stops

        # [-3] has no specific meaning.
        # But [-1] may not have full info. So we look somewhat old info.
        idx = -3
        if len(stamps) == 0:
            return
        elif len(stamps) < 3:
            idx = 0

        stats = PerTopicLatencyStat()
        for target_stamp in stamps[:idx]:
            results = solver.solve(pubinfos, target_topic, target_stamp,
                                   stops=stops)
            for r in results.data:
                stats.add(r)

        print("stats")
        stats.print_report()
        print("")

    def handle_one_hot(self, stamps):
        """Report only one message

        Paramters
        ---------
        stamps: stamps sorted by old-to-new order

        Returns
        -------
        """
        pubinfos = self.pub_infos
        target_topic = self.target_topic
        solver = self.solver
        stops = self.stops

        # [-3] has no specific meaning.
        # But [-1] may not have full info. So we look somewhat old info.
        idx = -3
        if len(stamps) == 0:
            return
        elif len(stamps) < 3:
            idx = 0

        target_stamp = stamps[idx]
        results = solver.solve(pubinfos, target_topic, target_stamp,
                               stops=stops)

        def p(v):
            if v > 1000:
                return "   inf"
            else:
                return "{:>6.1f}".format(v)

        def pbool(v):
            return "True" if v else "False"

        print("one_hot")
        s = "{:80} {:>20} {:>6} {:>6} {:>6} {:>5}".format(
            "topic", "stamp", "dur", "e2e", "e2e_s", "is_leaf", "parent"
        )
        print(s)
        for result in results.data:
            s = f"{result.topic:80} "
            s += f"{result.stamp:>20} "
            s += f"{p(result.dur_ms)} "
            s += f"{p(result.dur_pub_ms)} "
            s += f"{p(result.dur_pub_ms_steady)} "
            s += f"{pbool(result.is_leaf):>5} "
            s += f"{result.parent}"
            print(s)
        print("")

    def timer_callback(self):
        # print(f"timer_callback")
        pubinfos = self.pub_infos
        target_topic = self.target_topic

        stamps = sorted(pubinfos.stamps(target_topic))
        print(f"stamps: {len(stamps)}, {stamps[0] if len(stamps) > 0 else ''}")
        if len(stamps) == 0:
            return

        if not self.solver:
            if self.wait_init < self.wait_sec_to_init_graph:
                self.wait_init += 1
                return
            print("init solver")
            graph = TopicGraph(pubinfos, skips=self.skips)
            self.solver = InputSensorStampSolver(graph)

        mode = self.get_parameter("mode").get_parameter_value().string_value
        if mode == "stat":
            self.handle_stat(stamps)
        elif mode == "onehot":
            self.handle_one_hot(stamps)
        else:
            print("unknown mode")

        # cleanup PubInfos
        (latest_sec, latest_ns) = map(lambda x: int(x), stamps[-1].split("."))
        until_stamp = TimeMsg(sec=latest_sec - self.keep_info_sec,
                              nanosec=latest_ns)
        pubinfos.erase_until(until_stamp)

    def get_pub_info_topics(self, excludes=[]):
        """Get all topic infos

        Paramters
        ---------

        Returns
        -------
        map
          a list of pubinfo topics
        """
        msg_type = "path_info_msg/msg/PubInfo"
        topic_and_types = self.get_topic_names_and_types()
        filtered_topic_and_types = \
            filter(lambda x: msg_type in x[1], topic_and_types)
        topics = map(lambda x: x[0], filtered_topic_and_types)

        return topics


def main(args=None):
    rclpy.init(args=args)
    node = LatencyViewerNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
