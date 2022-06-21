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

"""Latency viewer node and its main function."""

import argparse
import curses
import os
import pickle
from statistics import mean
import sys
import time

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy
    )
from rclpy.time import Time

from tilde_msg.msg import (
    MessageTrackingTag,
    )
from tilde_vis.message_tracking_tag import (
    MessageTrackingTag as MessageTrackingTagObj,
    MessageTrackingTags as MessageTrackingTagsObj,
    time2str
    )
from tilde_vis.message_tracking_tag_traverse import (
    InputSensorStampSolver,
    TopicGraph
    )
from tilde_vis.printer import (
    NcursesPrinter,
    Printer
    )


EXCLUDES_TOPICS = [
    '/diagnostics/message_tracking_tag',
    '/control/trajectory_follower/mpc_follower/debug/markers/message_tracking_tag',  # noqa: #501
    '/control/trajectory_follower/mpc_follower/debug/steering_cmd/message_tracking_tag',  # noqa: #501
    '/localization/debug/ellipse_marker/message_tracking_tag',
    '/localization/pose_twist_fusion_filter/debug/message_tracking_tag',
    '/localization/pose_twist_fusion_filter/debug/measured_pose/message_tracking_tag',  # noqa: #501
    '/localization/pose_twist_fusion_filter/debug/stop_flag/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/drivable_area/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/debug/markers/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/area_with_objects/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/clearance_map/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/marker/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/object_clearance_map/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/debug/smoothed_points/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/backward_filtered_trajectory/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/forward_filtered_trajectory/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/merged_filtered_trajectory/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_external_velocity_limited/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_lateral_acc_filtered/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_raw/message_tracking_tag',  # noqa: #501
    '/planning/scenario_planning/motion_velocity_smoother/debug/trajectory_time_resampled/message_tracking_tag',  # noqa: #501
    ]
LEAVES = [
    '/initialpose',
    '/map/pointcloud_map',
    '/sensing/lidar/top/rectified/pointcloud',
    '/sensing/imu/imu_data',
    '/vehicle/status/twist',
    ]
MESSAGE_TRACKING_TAG = 'topic_infos.pkl'
TIMER_SEC = 1.0
TARGET_TOPIC = '/sensing/lidar/concatenated/pointcloud'
STOPS = [
    '/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias',  # noqa: #501
    '/perception/occupancy_grid_map/map',
    '/perception/object_recognition/detection/detection_by_tracker/objects',
    '/perception/object_recognition/tracking/objects',
    ]
DUMP_DIR = 'dump.d'


def truncate(s, left_length=20, n=80):
    """
    Truncate string.

    Parameters
    ----------
    s: string
    left_length: first part length
    n: total length

    Return
    ------
    truncated string such that "abc...edf"

    """
    assert left_length + 5 < n

    if len(s) < n:
        return s

    pre = s[:left_length]
    post = s[len(s) - n + left_length + 5:]

    return pre + '...' + post


class LatencyStat(object):
    """Latency statistics."""

    def __init__(self):
        """Constructor."""
        self.dur_ms_list = []
        self.dur_pub_ms_list = []
        self.dur_pub_ms_steady_list = []
        self.is_leaf_list = []

    def add(self, r):
        """
        Add single result.

        Parameters
        ----------
        r: message_tracking_tag_traverse.SolverResults

        """
        self.dur_ms_list.append(r.dur_ms)
        self.dur_pub_ms_list.append(r.dur_pub_ms)
        self.dur_pub_ms_steady_list.append(r.dur_pub_ms_steady)
        self.is_leaf_list.append(r.is_leaf)

    def report(self):
        """Report statistics."""
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
            'dur_min': dur_min,
            'dur_mean': dur_mean,
            'dur_max': dur_max,
            'dur_pub_min': dur_pub_min,
            'dur_pub_mean': dur_pub_mean,
            'dur_pub_max': dur_pub_max,
            'dur_pub_steady_min': dur_pub_steady_min,
            'dur_pub_steady_mean': dur_pub_steady_mean,
            'dur_pub_steady_max': dur_pub_steady_max,
            'is_all_leaf': is_all_leaf,
            }


class PerTopicLatencyStat(object):
    """Per topic latency statistics."""

    def __init__(self):
        """Constructor."""
        self.data = {}

    def add(self, r):
        """
        Add single result.

        Parameters
        ----------
        r: message_tracking_tag_traverse.SolverResults

        """
        self.data.setdefault(r.topic, LatencyStat()).add(r)

    def report(self):
        """Get report as dictionary."""
        ret = {}
        for (topic, stat) in self.data.items():
            ret[topic] = stat.report()
        return ret

    def print_report(self, printer):
        """
        Print report.

        Parameters
        ----------
        printer: see printer.py

        """
        logs = []
        reports = self.report()
        logs.append('{:80} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}'.format(
            'topic', 'dur', 'dur', 'dur', 'e2e', 'e2e', 'e2e', 'e2e_s', 'e2e_s', 'e2e_s'
        ))

        def p(v):
            if v > 1000:
                return '   inf'
            else:
                return '{:>6.1f}'.format(v)

        for (topic, report) in reports.items():
            s = f'{topic:80} '
            s += f'{p(report["dur_min"])} '
            s += f'{p(report["dur_mean"])} '
            s += f'{p(report["dur_max"])} '
            s += f'{p(report["dur_pub_min"])} '
            s += f'{p(report["dur_pub_mean"])} '
            s += f'{p(report["dur_pub_max"])} '
            s += f'{p(report["dur_pub_steady_min"])} '
            s += f'{p(report["dur_pub_steady_mean"])} '
            s += f'{p(report["dur_pub_steady_max"])} '
            s += f'{report["is_all_leaf"]}'
            logs.append(s)

        printer.print_(logs)


def calc_one_hot(results):
    """
    Calculate one hot result.

    Parameters
    ----------
    results: see InputSensorStampSolver.solve2

    Return
    ------
    list of (depth, topic name, dur, dur_steady, is_leaf)
    depth: depth of watched topic
    topic_name: watched topic
    dur: final topic pub_time - sensor pub_time
    dur_steady: steady version of dur
    is_leaf: leaf of not [bool]
    stamp: target topic output stamp

    """
    last_pub_time = Time.from_msg(results.data[0].out_info.pubsub_stamp)
    last_pub_time_steady = Time.from_msg(
        results.data[0].out_info.pubsub_stamp_steady)

    def calc(node, depth):
        name = node.name
        is_leaf = node.num_children() == 0

        if len(node.data) == 0 or node.data[0] is None:
            return (depth, name, None, None, is_leaf, None)

        stamp = node.data[0].out_info.stamp
        pub_time = Time.from_msg(node.data[0].out_info.pubsub_stamp)
        dur = last_pub_time - pub_time
        dur_ms = dur.nanoseconds // (10 ** 6)

        pub_time_steady = Time.from_msg(
            node.data[0].out_info.pubsub_stamp_steady)
        dur_steady = last_pub_time_steady - pub_time_steady
        dur_ms_steady = dur_steady.nanoseconds // (10 ** 6)

        return (depth, name, dur_ms, dur_ms_steady, is_leaf, stamp)

    return results.apply_with_depth(calc)


def handle_stat(stamps, message_tracking_tags, target_topic, solver, stops, dumps=False):
    """
    Calculate latency statistics.

    Parameters
    ----------
    stamps:
    message_tracking_tags:
    target_topic:
    solver:
    stops:
    dumps:

    Result
    ------
    TreeNode.
    .data: see calc_stat

    """
    idx = -3
    if len(stamps) == 0:
        return
    elif len(stamps) < 3:
        idx = 1

    merged = None
    print(f'idx: {idx}')
    for target_stamp in stamps[:idx]:
        results = solver.solve2(
            message_tracking_tags, target_topic, target_stamp,
            stops=stops)

        if dumps:
            pickle.dump(results,
                        open(f'{DUMP_DIR}/stat_results_{target_stamp}.pkl',
                             'wb'),
                        protocol=pickle.HIGHEST_PROTOCOL)

        results = update_stat(results)
        if merged is None:
            merged = results
        else:
            merged.merge(results)

    stats = calc_stat(merged)
    return stats


def update_stat(results):
    """
    Update results to have durations.

    Parameters
    ----------
    results: see InputSensorStampSolver.solve2

    Return
    ------
    results with
    results.data: [(dur_ms, dur_ms_steady)]
    results.data_orig = results.data

    """
    last_pub_time = Time.from_msg(
        results.data[0].out_info.pubsub_stamp)
    last_pub_time_steady = Time.from_msg(
        results.data[0].out_info.pubsub_stamp_steady)

    def _update_stat(node):
        if len(node.data) == 0 or node.data[0] is None:
            node.data_orig = node.data
            node.data = [(None, None)]
            return

        pub_time = Time.from_msg(
            node.data[0].out_info.pubsub_stamp)
        dur = last_pub_time - pub_time
        dur_ms = dur.nanoseconds // (10 ** 6)

        pub_time_steady = Time.from_msg(
            node.data[0].out_info.pubsub_stamp_steady)
        dur_steady = last_pub_time_steady - pub_time_steady
        dur_ms_steady = dur_steady.nanoseconds // (10 ** 6)

        node.data_orig = node.data
        node.data = [(dur_ms, dur_ms_steady)]

    results.apply(_update_stat)

    return results


def calc_stat(results):
    """
    Calculate statistics.

    Parameters
    ----------
    results: TreeNode which is updated by update_stat

    Return
    ------
    list of dictionary whose keys are:
      "depth"
      "name"
      "dur_min"
      "dur_mean"
      "dur_max"
      "dur_min_steady"
      "dur_mean_steady"
      "dur_max_steady"
      "is_leaf"

    """
    def _calc_stat(node, depth):
        duration = []
        duration_steady = []
        is_leaf = True if node.num_children() == 0 else False
        for d in node.data:
            if d[0] is not None:
                duration.append(d[0])
            if d[1] is not None:
                duration_steady.append(d[1])

        def _calc(arr, fn):
            if len(arr) == 0:
                return None
            return fn(arr)

        return {
            'depth': depth,
            'name': node.name,
            'dur_min': _calc(duration, min),
            'dur_mean': _calc(duration, mean),
            'dur_max': _calc(duration, max),
            'dur_min_steady': _calc(duration_steady, min),
            'dur_mean_steady': _calc(duration_steady, mean),
            'dur_max_steady': _calc(duration_steady, max),
            'is_leaf': is_leaf
        }

    return results.apply_with_depth(_calc_stat)


class LatencyViewerNode(Node):
    """Latency viewer node."""

    def __init__(self, stdscr=None):
        """
        Constructor.

        Parameters
        ----------
        stdscr: if not None, use ncurses

        """
        super().__init__('latency_viewer_node')
        self.declare_parameter('excludes_topics', EXCLUDES_TOPICS)
        self.declare_parameter('leaves', LEAVES)
        self.declare_parameter('graph_pkl', '')
        self.declare_parameter('timer_sec', TIMER_SEC)
        self.declare_parameter('target_topic', TARGET_TOPIC)
        self.declare_parameter('keep_info_sec', 3)
        self.declare_parameter('wait_sec_to_init_graph', 10)
        self.declare_parameter('mode', 'stat')
        self.declare_parameter('stops', STOPS)
        # whether to dump solver.solve() result
        self.declare_parameter('dumps', False)

        print(stdscr)
        if stdscr is not None:
            self.printer = NcursesPrinter(stdscr)
        else:
            self.printer = Printer()

        self.subs = {}
        # topic vs the latest sequence numbers
        self.topic_seq = {}

        excludes_topic = (
            self.get_parameter('excludes_topics')
            .get_parameter_value().string_array_value)
        topics = self.get_message_tracking_tag_topics(excludes=excludes_topic)
        logs = []

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
        for topic in topics:
            logs.append(topic)
            sub = self.create_subscription(
                MessageTrackingTag,
                topic,
                self.listener_callback,
                qos)
            self.subs[topic] = sub

        self.solver = None
        self.stops = (
            self.get_parameter('stops')
            .get_parameter_value().string_array_value)
        graph_pkl = (
            self.get_parameter('graph_pkl')
            .get_parameter_value().string_value)
        if graph_pkl:
            graph = pickle.load(open(graph_pkl, 'rb'))
            self.solver = InputSensorStampSolver(graph)

        self.message_tracking_tags = MessageTrackingTagsObj()

        timer_sec = (
            self.get_parameter('timer_sec')
            .get_parameter_value().double_value)
        self.timer = self.create_timer(timer_sec,
                                       self.timer_callback)

        self.target_topic = (
            self.get_parameter('target_topic')
            .get_parameter_value().string_value)
        self.keep_info_sec = (
            self.get_parameter('keep_info_sec')
            .get_parameter_value().integer_value)
        self.wait_sec_to_init_graph = (
            self.get_parameter('wait_sec_to_init_graph').
            get_parameter_value().integer_value)
        self.wait_init = 0

        self.init_skips()

        self.printer.print_(logs)

        self.dumps = (
            self.get_parameter('dumps')
            .get_parameter_value().bool_value)
        if self.dumps:
            os.makedirs(DUMP_DIR, exist_ok=True)

    def init_skips(self):
        """
        Define skips.

        See TopicGraph.__init__ comment.
        """
        skips = {}
        RECT_OUT_EX = '/sensing/lidar/{}/rectified/pointcloud_ex'
        RECT_OUT = '/sensing/lidar/{}/rectified/pointcloud'
        RECT_IN = '/sensing/lidar/{}/mirror_cropped/pointcloud_ex'

        # top
        for pos in ['top', 'left', 'right']:
            skips[RECT_OUT_EX.format(pos)] = RECT_IN.format(pos)
            skips[RECT_OUT.format(pos)] = RECT_IN.format(pos)

        skips['/sensing/lidar/no_ground/pointcloud'] = \
            '/sensing/lidar/concatenated/pointcloud'

        self.skips = skips

    def listener_callback(self, message_tracking_tag_msg):
        """Handle MessageTrackingTag message."""
        st = time.time()
        output_info = message_tracking_tag_msg.output_info

        topic = output_info.topic_name
        stamp = time2str(output_info.header_stamp)

        this_seq = output_info.seq
        if topic not in self.topic_seq.keys():
            self.topic_seq[topic] = this_seq
        seq = self.topic_seq[topic]

        if this_seq < seq:  # skew
            s = f'skew topic={topic} ' + \
                f'msg_seq={this_seq}({time2str(output_info.header_stamp)})' + \
                f' saved_seq={seq}'
            stamps = self.message_tracking_tags.stamps(topic)
            if stamps or len(stamps) > 0:
                last_stamp = sorted(stamps)[-1]
                s += f'({last_stamp})'
            self.get_logger().info(s)
        elif seq + 1 < this_seq:  # message drop happens
            s = f'may drop topic={topic} ' + \
                f'msg_seq={this_seq}({time2str(output_info.header_stamp)})' + \
                f' saved_seq={seq}'
            stamps = self.message_tracking_tags.stamps(topic)
            if stamps or len(stamps) > 0:
                last_stamp = sorted(stamps)[-1]
                s += f'({last_stamp})'
            self.get_logger().info(s)
            self.topic_seq[topic] = this_seq
        else:
            self.topic_seq[topic] = this_seq

        # add message_tracking_tag
        message_tracking_tag = MessageTrackingTagObj(
            output_info.topic_name,
            output_info.pub_time,
            output_info.pub_time_steady,
            output_info.has_header_stamp,
            output_info.header_stamp)
        for input_info in message_tracking_tag_msg.input_infos:
            message_tracking_tag.add_input_info(
                input_info.topic_name,
                input_info.sub_time,
                input_info.sub_time_steady,
                input_info.has_header_stamp,
                input_info.header_stamp)
        self.message_tracking_tags.add(message_tracking_tag)

        et = time.time()

        elapsed_ms = (et - st) * 1000
        if elapsed_ms > 1:
            self.get_logger().info(
                f'sub {topic} at {stamp}@ {elapsed_ms} [ms]')

    def handle_stat(self, stamps):
        """
        Report statistics.

        Parameters
        ----------
        stamps: stamps sorted by old-to-new order

        Returns
        -------
        list of string

        """
        message_tracking_tags = self.message_tracking_tags
        target_topic = self.target_topic
        solver = self.solver
        stops = self.stops
        dumps = self.dumps

        stats = handle_stat(stamps,
                            message_tracking_tags,
                            target_topic,
                            solver,
                            stops,
                            dumps=dumps)

        logs = []

        fmt = '{:80} ' + \
            '{:>6} {:>6} {:>6} ' + \
            '{:>6} {:>6} {:>6}'
        logs.append(fmt.format(
            'topic',
            'e2e', 'e2e', 'e2e',
            'e2e_s', 'e2e_s', 'e2e_s'
        ))

        for stat in stats:
            name = (' ' * stat['depth'] +
                    stat['name'] +
                    ('*' if stat['is_leaf'] else ''))
            name = truncate(name)

            def p(v):
                if v is None:
                    s = 'NA'
                    return f'{s:>6}'
                if v > 1000:
                    s = 'inf'
                    return f'{s:>6}'
                else:
                    return f'{v:>6.1f}'

            s = f'{name:80} '
            s += f'{p(stat["dur_min"]):>6} '
            s += f'{p(stat["dur_mean"]):>6} '
            s += f'{p(stat["dur_max"]):>6} '
            s += f'{p(stat["dur_min_steady"]):>6} '
            s += f'{p(stat["dur_mean_steady"]):>6} '
            s += f'{p(stat["dur_max_steady"]):>6} '

            logs.append(s)

        return logs

    def handle_one_hot(self, stamps):
        """
        Report only one message.

        Parameters
        ----------
        stamps: stamps sorted by old-to-new order

        Returns
        -------
        array of strings for log

        """
        message_tracking_tags = self.message_tracking_tags
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
        results = solver.solve2(message_tracking_tags, target_topic, target_stamp,
                                stops=stops)

        if self.dumps:
            pickle.dump(results,
                        open(f'{DUMP_DIR}/one_hot_results_{target_stamp}.pkl',
                             'wb'),
                        protocol=pickle.HIGHEST_PROTOCOL)

        one_hot_durations = calc_one_hot(results)
        logs = []
        for e in one_hot_durations:
            (depth, name, dur_ms, dur_ms_steady, is_leaf, stamp) = e
            name = truncate(' ' * depth + name + ('*' if is_leaf else ''))
            if dur_ms is None:
                dur_ms = 'NA'
            if dur_ms_steady is None:
                dur_ms_steady = 'NA'
            stamp_s = 'NA'
            if stamp:
                stamp_s = time2str(stamp)

            s = f'{name:80} {stamp_s:>20} {dur_ms:>6} {dur_ms_steady:>6}'
            logs.append(s)

        return logs

    def timer_callback(self):
        """Clear old data."""
        st = time.time()
        message_tracking_tags = self.message_tracking_tags
        target_topic = self.target_topic

        stamps = sorted(message_tracking_tags.stamps(target_topic))
        logs = []
        str_stamp = stamps[0] if len(stamps) > 0 else ''
        logs.append(f'stamps: {len(stamps)}, {str_stamp}')
        if len(stamps) == 0:
            self.printer.print_(logs)
            return

        # check header.stamp field existence
        if not message_tracking_tags.get(target_topic, stamps[0]).out_info.has_stamp:
            logs.append('**WARNING** target topic has no stamp field')
            self.printer.print_(logs)
            return

        if not self.solver:
            if self.wait_init < self.wait_sec_to_init_graph:
                self.printer.print_(logs)
                self.wait_init += 1
                return
            logs.append('init solver')
            graph = TopicGraph(message_tracking_tags, skips=self.skips)
            self.solver = InputSensorStampSolver(graph)

        mode = self.get_parameter('mode').get_parameter_value().string_value
        if mode == 'stat':
            logs.extend(self.handle_stat(stamps))
        elif mode == 'one_hot':
            logs.extend(self.handle_one_hot(stamps))
        else:
            logs.append('unknown mode')
        et1 = time.time()
        handle_ms = (et1 - st) * 1000

        # cleanup MessageTrackingTags
        (latest_sec, latest_ns) = map(lambda x: int(x), stamps[-1].split('.'))
        until_stamp = TimeMsg(sec=latest_sec - self.keep_info_sec,
                              nanosec=latest_ns)
        message_tracking_tags.erase_until(until_stamp)
        et2 = time.time()
        cleanup_ms = (et2 - et1) * 1000

        self.printer.print_(logs)

        if handle_ms + cleanup_ms > 30:
            s = f'timer handle_ms={handle_ms}' + \
                f' cleanup_ms={cleanup_ms}'
            self.get_logger().info(s)

    def get_message_tracking_tag_topics(self, excludes=[]):
        """
        Get all topic infos.

        Parameters
        ----------
        excludes: topic names

        Returns
        -------
        map
          a list of message_tracking_tag topics

        """
        # short sleep between node creation and get_topic_names_and_types
        # https://github.com/ros2/ros2/issues/1057
        # We need this sleep with autoware,
        # but don't need in dev environment(i.e. MessageTrackingTag in rosbag)
        time.sleep(0.5)

        msg_type = 'tilde_msg/msg/MessageTrackingTag'
        topic_and_types = self.get_topic_names_and_types()
        filtered_topic_and_types = \
            filter(lambda x: msg_type in x[1], topic_and_types)
        topics = map(lambda x: x[0], filtered_topic_and_types)
        topics = filter(lambda x: x not in excludes, topics)

        return topics


def main_curses(stdscr, args=None):
    """Wrap main function for ncurses."""
    rclpy.init(args=args)

    node = LatencyViewerNode(stdscr=stdscr)
    rclpy.spin(node)
    rclpy.shutdown()


def main(args=None):
    """Main."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--batch', action='store_true',
        help='Run as batch mode, just like top command `-b` option')
    parsed = parser.parse_known_args()[0]

    print(parsed)

    if not parsed.batch:
        print('not batch')
        curses.wrapper(main_curses, args=sys.argv)
    else:
        print('batch')
        main_curses(None, args=sys.argv)


if __name__ == '__main__':
    """Main."""
    main(sys.argv)
