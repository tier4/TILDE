#!env python3

import os
import collections
import numpy as np
from logging import basicConfig, getLogger, DEBUG

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from path_info_msg.msg import TopicInfo

basicConfig(level=os.getenv('LOG_LEVEL', 'INFO'))
logger = getLogger(__name__)

# for subscriber demo
LIDAR_PREPROCESS = [
    '/sensing/lidar/top/self_cropped/pointcloud_ex',
    # '/sensing/lidar/top/mirror_cropped/pointcloud_ex',  # subscriber is /sensing/lidar/top/velodyne_interpolate_node which is not autoware node
    '/sensing/lidar/top/rectified/pointcloud_ex',
    '/sensing/lidar/top/outlier_filtered/pointcloud',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/measurement_range_cropped/pointcloud',
]

# for publisher demo
LIDAR_PREPROCESS_PUB = [
    '/sensing/lidar/top/self_cropped/pointcloud_ex',
    '/sensing/lidar/top/mirror_cropped/pointcloud_ex',
    # '/sensing/lidar/top/rectified/pointcloud_ex',  # publisher is /sensing/lidar/top/velodyne_interpolate_node which is not autoware node
    '/sensing/lidar/top/outlier_filtered/pointcloud',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/measurement_range_cropped/pointcloud',
]

class TopicInfoStatistics(object):
    def __init__(self, topics, max_rows=10):
        self.topics = topics
        self.t2i = {topic:i for i, topic in enumerate(topics)}
        self.seq2time = collections.defaultdict(lambda: - np.ones(len(self.t2i), dtype=np.float64))
        # (n_messages, n_subcallbacks), nanoseconds
        self.data = - np.ones((max_rows, len(topics)), dtype=np.float)
        self.data_idx = 0
        self.max_rows = max_rows
        self.num_dump = -1

        s = ""
        for t in topics:
            s += f"{t}  "
        s = s.rstrip()
        s = s.replace("  ", " -> ")
        print(s)

    def set(self, seq, topic, callback_start_time):
        vec = self.seq2time[seq]
        i = self.t2i[topic]
        vec[i] = callback_start_time
        logger.debug("seq {}: i: {} non zero: {}".format(seq, i, np.count_nonzero(vec > 0)))
        if np.count_nonzero(vec > 0) == len(self.t2i) and self.data_idx < self.max_rows:
            self.data[self.data_idx, ...] = vec[...]
            del self.seq2time[seq]
            self.data_idx += 1

            # TODO: delete too old seq key (now leaks)

    def is_filled(self):
        return self.data_idx == self.max_rows

    def dump_and_clear(self, dumps=False):
        if dumps:
            self.num_dump += 1
            fname = "{}.npy".format(self.num_dump)
            np.save(fname, self.data)

        # TODO: ignore nan(-1) field
        data = self.data[self.data.min(axis=1) > 0]

        nano2msec = 1000 * 1000
        data /= nano2msec

        diff = data[:, 1:] - data[:, :-1]
        e2e = data[:, -1] - data[:, 0]

        maxtime = diff.max(axis=0)
        avgtime = diff.mean(axis=0)
        mintime = diff.min(axis=0)
        maxe2e = e2e.max()
        avge2e = e2e.mean()
        mine2e = e2e.min()

        def fmt(vec):
            s = ""
            for v in vec:
                s += f"{v:5.1f}  "
            return s.rstrip()

        print("max: " + fmt(maxtime))
        print("avg: " + fmt(avgtime))
        print("min: " + fmt(mintime))
        print("e2e: " + fmt([maxe2e, avge2e, mine2e]))
        print("")

        self.data[...] = -1.0
        self.data_idx = 0

class PathVisNode(Node):
    def __init__(self):
        super().__init__('path_vis_node')
        self.declare_parameter("topics", LIDAR_PREPROCESS)
        self.declare_parameter("window", 10)
        self.declare_parameter("dump", False)
        self.declare_parameter("watches_pub", False)

        topics = self.get_parameter("topics").get_parameter_value().string_array_value
        window = self.get_parameter("window").get_parameter_value().integer_value
        watches_pub = self.get_parameter("watches_pub").get_parameter_value().bool_value
        info_name = "/info/sub"
        if watches_pub:
            info_name = "/info/pub"
            topics = LIDAR_PREPROCESS_PUB

        self.statistics = TopicInfoStatistics(topics, window)
        self.subs = []

        logger.debug("info_name: {}".format(info_name))

        for topic in topics:
            sub = self.create_subscription(
                TopicInfo,
                topic + info_name,
                self.listener_callback,
                1)

            self.subs.append(sub)

    def listener_callback(self, topic_info):
        seq = topic_info.seq
        topic = topic_info.topic_name
        stime = Time.from_msg(topic_info.callback_start).nanoseconds
        dump = self.get_parameter("dump").get_parameter_value().bool_value

        logger.debug("{} {}".format(seq, topic))
        self.statistics.set(seq, topic, stime)
        if self.statistics.is_filled():
            self.statistics.dump_and_clear(dump)

def main(args=None):
    rclpy.init(args=args)
    node = PathVisNode()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
