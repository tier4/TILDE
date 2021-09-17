#!env python3

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from path_info_msg.msg import TopicInfo

LIDAR_PREPROCESS = [
    '/sensing/lidar/top/self_cropped/pointcloud_ex',
    # '/sensing/lidar/top/mirror_cropped/pointcloud_ex',  # not autoware node
    '/sensing/lidar/top/rectified/pointcloud_ex',
    '/sensing/lidar/top/outlier_filtered/pointcloud',
    '/sensing/lidar/concatenated/pointcloud',
    '/sensing/lidar/measurement_range_cropped/pointcloud',
]

class TopicInfoStatistics(object):
    def __init__(self, topics, max_rows=10):
        self.topics = topics
        self.t2i = {topic:i for i, topic in enumerate(topics)}
        # (n_messages, n_subcallbacks), nanoseconds
        self.data = - np.ones((max_rows, len(topics)), dtype=np.float)
        self.max_rows = max_rows
        self.num_dump = -1

        s = ""
        for t in topics:
            s += f"{t}  "
        s = s.rstrip()
        s = s.replace("  ", " -> ")
        print(s)

    def set(self, seq, topic, callback_start_time):
        n = seq % self.max_rows
        i = self.t2i[topic]
        self.data[n, i] = callback_start_time

    def is_filled(self):
        return self.data[-1, -1] >= 0

    def dump_and_clear(self, dumps=False):
        if dumps:
            self.num_dump += 1
            fname = "{}.npy".format(self.num_dump)
            np.save(fname, self.data)

        # TODO: ignore nan(-1) field
        self.data[self.data < 0] = 0.0

        nano2msec = 1000 * 1000
        self.data /= nano2msec

        diff = self.data[:, 1:] - self.data[:, :-1]
        e2e = self.data[:, -1] - self.data[:, 0]

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

class PathVisNode(Node):
    def __init__(self):
        super().__init__('path_vis_node')
        self.declare_parameter("topics", LIDAR_PREPROCESS)
        self.declare_parameter("window", 10)
        self.declare_parameter("dump", False)

        topics = self.get_parameter("topics").get_parameter_value().string_array_value
        window = self.get_parameter("window").get_parameter_value().integer_value

        self.statistics = TopicInfoStatistics(topics, window)
        self.subs = []
        for topic in topics:
            sub = self.create_subscription(
                TopicInfo,
                topic + "_info",
                self.listener_callback,
                1)

            self.subs.append(sub)

    def listener_callback(self, topic_info):
        seq = topic_info.seq
        topic = topic_info.topic_name
        stime = Time.from_msg(topic_info.callback_start).nanoseconds
        dump = self.get_parameter("dump").get_parameter_value().bool_value

        # print("{} {}".format(seq, topic))
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
