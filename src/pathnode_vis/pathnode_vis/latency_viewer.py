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

import argparse
import curses
import pickle
import sys
from statistics import mean

import rclpy
from rclpy.node import Node
from rclpy.time import Time

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


class Printer(object):
    def print(self, lines):
        for s in lines:
            print(s)


class NcursesPrinter(object):
    def __init__(self, stdscr):
        stdscr.scrollok(True)
        self.stdscr = stdscr

    def print(self, lines):
        stdscr = self.stdscr
        stdscr.clear()

        for s in lines:
            if s[-1] != "\n":
                s += "\n"
            stdscr.addstr(s)
        stdscr.refresh()


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

    def print_report(self, printer):
        logs = []
        reports = self.report()
        logs.append("{:80} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}".format(
            "topic", "dur", "dur", "dur", "e2e", "e2e", "e2e", "e2e_s", "e2e_s", "e2e_s"
        ))

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
            logs.append(s)

        printer.print(logs)


def calc_one_hot(results):
    """Calcurate one hot result.

    Paramters
    ---------
    results: see InputSensorStampSolver.solve2

    Return
    ------
    list of (depth, topic name, dur, dur_steady)
    depth: depth of watched topic
    topic_name: watched topic
    dur: final topic pubtime - pubtime
    dur_steady
    """
    last_pub_time = Time.from_msg(results.data[0].out_info.pubsub_stamp)
    last_pub_time_steady = Time.from_msg(
        results.data[0].out_info.pubsub_stamp_steady)

    def calc(node, depth):
        name = node.name

        if len(node.data) == 0 or node.data[0] is None:
            return (depth, name, None, None)

        pub_time = Time.from_msg(node.data[0].out_info.pubsub_stamp)
        dur = last_pub_time - pub_time
        dur_ms = dur.nanoseconds // (10 ** 6)

        pub_time_steady = Time.from_msg(
            node.data[0].out_info.pubsub_stamp_steady)
        dur_steady = last_pub_time_steady - pub_time_steady
        dur_ms_steady = dur_steady.nanoseconds // (10 ** 6)

        return (depth, name, dur_ms, dur_ms_steady)

    return results.apply_with_depth(calc)


class LatencyViewerNode(Node):
    def __init__(self, stdscr=None):
        """Constructor

        Parameters
        ----------
        stdscr: if not None, use ncurses
        """
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

        print(stdscr)
        if stdscr is not None:
            self.printer = NcursesPrinter(stdscr)
        else:
            self.printer = Printer()

        self.subs = {}
        excludes_topic = (
            self.get_parameter("excludes_topics")
            .get_parameter_value().string_array_value)
        topics = self.get_pub_info_topics(excludes=excludes_topic)
        logs = []
        for topic in topics:
            logs.append(topic)
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

        self.printer.print(logs)

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

        skips["/sensing/lidar/no_ground/pointcloud"] = \
            "/sensing/lidar/concatenated/pointcloud"

        self.skips = skips

    def listener_callback(self, pub_info_msg):
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

        stats.print_report(self.printer)

    def handle_one_hot(self, stamps):
        """Report only one message

        Paramters
        ---------
        stamps: stamps sorted by old-to-new order

        Returns
        -------
        array of strings for log
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
        results = solver.solve2(pubinfos, target_topic, target_stamp,
                                stops=stops)

        onehot_durs = calc_one_hot(results)
        logs = []
        for (depth, name, dur_ms, dur_ms_steady) in onehot_durs:
            spacer = " " * depth
            name = spacer + name
            if dur_ms is None:
                dur_ms = "NA"
            if dur_ms_steady is None:
                dur_ms_steady = "NA"

            s = f"{name:80} {dur_ms:>6} {dur_ms_steady:>6}"
            logs.append(s)

        return logs

    def timer_callback(self):
        pubinfos = self.pub_infos
        target_topic = self.target_topic

        stamps = sorted(pubinfos.stamps(target_topic))
        logs = []
        logs.append(f"stamps: {len(stamps)}, {stamps[0] if len(stamps) > 0 else ''}")
        if len(stamps) == 0:
            self.printer.print(logs)
            return

        if not self.solver:
            if self.wait_init < self.wait_sec_to_init_graph:
                self.printer.print(logs)
                self.wait_init += 1
                return
            logs.append("init solver")
            graph = TopicGraph(pubinfos, skips=self.skips)
            self.solver = InputSensorStampSolver(graph)

        mode = self.get_parameter("mode").get_parameter_value().string_value
        if mode == "stat":
            logs.extend(self.handle_stat(stamps))
        elif mode == "onehot":
            logs.extend(self.handle_one_hot(stamps))
        else:
            logs.append("unknown mode")

        self.printer.print(logs)

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


def main_curses(stdscr, args=None):
    rclpy.init(args=args)

    node = LatencyViewerNode(stdscr=stdscr)
    rclpy.spin(node)
    rclpy.shutdown()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--batch", action="store_true",
        help="Run as batch mode, just like top command `-b` option")
    parsed = parser.parse_known_args()[0]

    print(parsed)

    if not parsed.batch:
        print("not batch")
        curses.wrapper(main_curses, args=sys.argv)
    else:
        print("batch")
        main_curses(None, args=sys.argv)


if __name__ == '__main__':
    main(sys.argv)
