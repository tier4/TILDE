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

"""LatencyViewer PoC version. Deprecated."""

from collections import defaultdict
from logging import basicConfig, getLogger
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tilde_msg.msg import TopicInfo

basicConfig(level=os.getenv('LOG_LEVEL', 'INFO'))
logger = getLogger(__name__)

# for subscriber demo
LIDAR_PREPROCESS = [
    '/sensing/lidar/top/self_cropped/pointcloud_ex',
    # subscriber is /sensing/lidar/top/velodyne_interpolate_node
    # which is not autoware node
    # '/sensing/lidar/top/mirror_cropped/pointcloud_ex',
    '/sensing/lidar/top/rectified/pointcloud_ex',
    '/sensing/lidar/top/outlier_filtered/pointcloud',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/measurement_range_cropped/pointcloud',
]

# for publisher demo
LIDAR_PREPROCESS_PUB = [
    '/sensing/lidar/top/self_cropped/pointcloud_ex',
    '/sensing/lidar/top/mirror_cropped/pointcloud_ex',
    # publisher is /sensing/lidar/top/velodyne_interpolate_node
    # which is not autoware node
    # '/sensing/lidar/top/rectified/pointcloud_ex',
    '/sensing/lidar/top/outlier_filtered/pointcloud',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/measurement_range_cropped/pointcloud',
]


class TopicInfoStatistics(object):
    """TopicInfo statistics."""

    def __init__(self, topics, max_rows=10):
        """Constructor."""
        self.topics = topics
        self.t2i = {topic: i for i, topic in enumerate(topics)}
        self.seq2time = defaultdict(
            lambda: - np.ones(len(self.t2i), dtype=np.float64))
        # (n_messages, n_sub_callbacks), nanoseconds
        self.data = - np.ones((max_rows, len(topics)), dtype=np.float)
        self.data_idx = 0
        self.max_rows = max_rows
        self.num_dump = -1

        s = ''
        for t in topics:
            s += f'{t}  '
        s = s.rstrip()
        s = s.replace('  ', ' -> ')
        print(s)

    def register(self, seq, topic, callback_start_time):
        """Register result."""
        vec = self.seq2time[seq]
        i = self.t2i[topic]
        vec[i] = callback_start_time
        logger.debug('seq {}: i: {} non zero: {}'.format(
            seq, i, np.count_nonzero(vec > 0)))
        if (np.count_nonzero(vec > 0) == len(self.t2i) and
                self.data_idx < self.max_rows):
            self.data[self.data_idx, ...] = vec[...]
            del self.seq2time[seq]
            self.data_idx += 1

            # TODO: delete too old seq key (now leaks)

    def is_filled(self):
        """Check data."""
        return self.data_idx == self.max_rows

    def dump_and_clear(self, dumps=False):
        """Dump and clear results."""
        if dumps:
            self.num_dump += 1
            fname = '{}.npy'.format(self.num_dump)
            np.save(fname, self.data)

        # TODO: ignore nan(-1) field
        data = self.data[self.data.min(axis=1) > 0]

        nano2msec = 1000 * 1000
        data /= nano2msec

        diff = data[:, 1:] - data[:, :-1]
        e2e = data[:, -1] - data[:, 0]

        max_time = diff.max(axis=0)
        avg_time = diff.mean(axis=0)
        min_time = diff.min(axis=0)
        max_e2e = e2e.max()
        avg_e2e = e2e.mean()
        min_e2e = e2e.min()

        def fmt(vec):
            s = ''
            for v in vec:
                s += f'{v:5.1f}  '
            return s.rstrip()

        print('max: ' + fmt(max_time))
        print('avg: ' + fmt(avg_time))
        print('min: ' + fmt(min_time))
        print('e2e: ' + fmt([max_e2e, avg_e2e, min_e2e]))
        print('')

        self.data[...] = -1.0
        self.data_idx = 0


class PathVisNode(Node):
    """Path visualization node."""

    def __init__(self):
        """Constructor."""
        super().__init__('path_vis_node')
        self.declare_parameter('topics', LIDAR_PREPROCESS)
        self.declare_parameter('window', 10)
        self.declare_parameter('dump', False)
        self.declare_parameter('watches_pub', False)

        topics = self.get_parameter('topics') \
                     .get_parameter_value() \
                     .string_array_value
        window = self.get_parameter('window') \
                     .get_parameter_value() \
                     .integer_value
        watches_pub = self.get_parameter('watches_pub') \
                          .get_parameter_value() \
                          .bool_value
        info_name = '/info/sub'
        if watches_pub:
            info_name = '/info/pub'
            topics = LIDAR_PREPROCESS_PUB

        self.statistics = TopicInfoStatistics(topics, window)
        self.subs = []

        logger.debug('info_name: {}'.format(info_name))

        for topic in topics:
            sub = self.create_subscription(
                TopicInfo,
                topic + info_name,
                self.listener_callback,
                1)

            self.subs.append(sub)

    def listener_callback(self, topic_info):
        """Store data and print results."""
        seq = topic_info.seq
        topic = topic_info.topic_name
        start_time = Time.from_msg(topic_info.callback_start).nanoseconds
        dump = self.get_parameter('dump').get_parameter_value().bool_value

        logger.debug('{} {}'.format(seq, topic))
        self.statistics.register(seq, topic, start_time)
        if self.statistics.is_filled():
            self.statistics.dump_and_clear(dump)


def main(args=None):
    """Main."""
    rclpy.init(args=args)
    node = PathVisNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    """Main."""
    main()
