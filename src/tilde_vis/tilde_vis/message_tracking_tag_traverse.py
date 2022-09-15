#!/usr/bin/python3
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

"""MessageTrackingTag Traverser."""

import argparse
from collections import defaultdict, deque
import json
import pickle
import time

from rclpy.time import Time

from tilde_vis.data_as_tree import TreeNode
from tilde_vis.message_tracking_tag import time2str


def str_stamp2time(str_stamp):
    """Convert string time to Time."""
    sec, nanosec = str_stamp.split('.')
    return Time(seconds=int(sec), nanoseconds=int(nanosec))


class SolverResult(object):
    """SolverResult."""

    def __init__(self, topic, stamp, dur_ms,
                 dur_pub_ms, dur_pub_ms_steady,
                 is_leaf, parent):
        """
        Initialize data.

        Parameters
        ----------
        topic: topic [string]
        stamp: header.stamp [rclpy.Time]
        dur_ms: duration of header.stamp in ms [double]
        dur_pub_ms: duration of output.pub_time in ms [double]
        dur_pub_ms_steady: same with above but in steady clock ms [double]
        is_leaf: bool
        parent: parent topic [string]

        """
        self.topic = topic
        self.stamp = stamp
        self.dur_ms = dur_ms
        self.dur_pub_ms = dur_pub_ms
        self.dur_pub_ms_steady = dur_pub_ms_steady
        self.is_leaf = is_leaf
        self.parent = parent


class SolverResultsPrinter(object):
    """SolverResultsPrinter."""

    @classmethod
    def as_tree(cls, results):
        """
        Construct array of string to print.

        This method returns tree command like expression
        from output topic to source topics.
        Multiple input topics are shown by indented listing.

        Example
        ---------------
          topic               dur   e2e
          out_topic:
            in_topic1         0.1   0.1
               in_topic1_1    0.2   0.2
            in_topic2         0.3   0.3

        Parameters
        ----------
        results: SolverResults

        Return
        ------
        array of string

        """
        pass


class SolverResults(object):
    """SolverResults."""

    def __init__(self):
        """Initialize data."""
        self.data = []  # list of Result

    def add(self, *args):
        """
        Register Result.

        Parameters
        ----------
        args: completely forward. See Result.__init__.

        """
        self.data.append(SolverResult(*args))


class InputSensorStampSolver(object):
    """InputSensorStampSolver."""

    def __init__(self, graph):
        """Initialize solver."""
        # {topic: {stamp: {sensor_topic: [stamps]}}}
        self.topic_stamp_to_sensor_stamp = {}
        self.graph = graph
        self.skips = graph.skips
        self.empty_results = {}

    def solve(self, message_tracking_tags, tgt_topic, tgt_stamp,
              stops=[]):
        """
        Traverse from (tgt_topic, tgt_stamp)-message.

        Parameters
        ----------
        topic: target topic
        stamp: target stamp(str)

        stops: list of stop topics to prevent loop

        Return
        ------
        SolverResults

        Calculate topic_stamp_to_sensor_stamp internally.

        """
        graph = self.graph
        path_bfs = graph.bfs_rev(tgt_topic)
        is_leaf = {t: b for (t, b) in path_bfs}
        skips = self.skips

        stamp = tgt_stamp

        # dists[topic][stamp]
        dists = defaultdict(lambda: defaultdict(lambda: -1))
        queue = deque()
        parentQ = deque()

        start = str_stamp2time(tgt_stamp)
        start_message_tracking_tag = message_tracking_tags.get(tgt_topic, tgt_stamp)
        start_pub_time = start_message_tracking_tag.out_info.pubsub_stamp
        start_pub_time_steady = start_message_tracking_tag.out_info.pubsub_stamp_steady

        dists[tgt_topic][tgt_stamp] = 1
        queue.append((tgt_topic, tgt_stamp,
                      start_pub_time, start_pub_time_steady))
        parentQ.append('')

        ret = SolverResults()
        while len(queue) != 0:
            topic, stamp, sub_time, sub_time_steady = queue.popleft()
            parent = parentQ.popleft()

            dur = start - str_stamp2time(stamp)
            dur_ms = dur.nanoseconds // 10**6

            dur_pub = Time.from_msg(start_pub_time) - Time.from_msg(sub_time)
            dur_pub_ms = dur_pub.nanoseconds // 10**6

            dur_pub_steady = \
                Time.from_msg(start_pub_time_steady) - \
                Time.from_msg(sub_time_steady)
            dur_pub_ms_steady = dur_pub_steady.nanoseconds // 10**6

            ret.add(topic, stamp, dur_ms,
                    dur_pub_ms, dur_pub_ms_steady,
                    is_leaf[topic], parent)

            # NDT-EKF has loop, so stop
            if topic in stops:
                continue

            # get next edges
            next_message_tracking_tag = message_tracking_tags.get(topic, stamp)
            if not next_message_tracking_tag:
                continue

            for in_infos in next_message_tracking_tag.in_infos.values():
                for in_info in in_infos:
                    nx_topic = in_info.topic
                    if nx_topic in skips:
                        nx_topic = skips[nx_topic]
                    nx_stamp = time2str(in_info.stamp)
                    if dists[nx_topic][nx_stamp] > 0:
                        continue
                    dists[nx_topic][nx_stamp] = dists[topic][stamp] + 1
                    queue.append((nx_topic, nx_stamp,
                                  in_info.pubsub_stamp,
                                  in_info.pubsub_stamp_steady))
                    parentQ.append(topic)

        return ret

    def solve2(self, message_tracking_tags, tgt_topic, tgt_stamp,
               stops=[]):
        """
        Traverse DAG from output to input.

        Parameters
        ----------
        message_tracking_tags: message_tracking_tags [MessageTrackingTags]
        tgt_topic: output topic [string]
        tgt_stamp: output header stamp [string]
        stops: stop list

        Return
        ------
        TreeNode
        - Tree structure represents TopicGraph.
          This means that even if some MessageTrackingTags loss in some timing,
          returned TreeNode preserve entire graph.
        - .name means topic
        - .data is MessageTrackingTag of the topic. [] whn MessageTrackingTag loss

        """
        skips = self.skips
        stamp = tgt_stamp
        key = tgt_topic + '.'.join(stops)
        if key not in self.empty_results:
            self.empty_results[key] = self._solve_empty(tgt_topic, stops=stops)
        empty_results = self.empty_results[key]

        queue = deque()

        root_results = TreeNode(tgt_topic)
        root_results.merge(empty_results)  # setup entire graph

        queue.append((tgt_topic, tgt_stamp, root_results))
        while len(queue) != 0:
            topic, stamp, current_result = queue.popleft()

            next_message_tracking_tag = message_tracking_tags.get(topic, stamp)
            if next_message_tracking_tag is not None:
                current_result.add_data(next_message_tracking_tag)
            else:
                # print(f"message_tracking_tag of {topic} {stamp} not found")
                continue

            # NDT-EKF has loop, so stop
            if topic in stops:
                continue

            for in_infos in next_message_tracking_tag.in_infos.values():
                for in_info in in_infos:
                    nx_topic = in_info.topic
                    if nx_topic in skips:
                        nx_topic = skips[nx_topic]
                    nx_stamp = time2str(in_info.stamp)
                    next_result = current_result.get_child(nx_topic)

                    queue.append((nx_topic, nx_stamp,
                                  next_result))

        return root_results

    def append(self, topic, stamp, sensor_topic, sensor_stamp):
        """Append results to topic_stamp_to_sensor_stamp."""
        dic = self.topic_stamp_to_sensor_stamp
        if topic not in dic.keys():
            dic[topic] = {}
        if stamp not in dic[topic].keys():
            dic[topic][stamp] = {}
        if sensor_topic not in dic[topic][stamp].keys():
            dic[topic][stamp][sensor_topic] = []

        dic[topic][stamp][sensor_topic].append(sensor_stamp)

    def _solve_empty(self, tgt_topic,
                     stops=[]):
        """
        Get empty results to know graph.

        Parameters
        ----------
        tgt_topic: output topic [string]
        stops: stop list

        Returns
        -------
        TreeNode

        """
        skips = self.skips
        graph = self.graph

        queue = deque()
        root_results = TreeNode(tgt_topic)

        queue.append((tgt_topic, root_results))
        while len(queue) != 0:
            topic, current_result = queue.popleft()

            if topic in stops:
                continue

            rev_topics = graph.rev_topics(topic)
            for nx_topic in rev_topics:
                if nx_topic in skips:
                    nx_topic = skips[nx_topic]

                next_result = current_result.get_child(nx_topic)
                queue.append((nx_topic, next_result))

        return root_results


class TopicGraph(object):
    """Construct topic graph by ignoring stamps."""

    def __init__(self, message_tracking_tags, skips={}):
        """
        Initialize graph.

        Parameters
        ----------
        message_tracking_tags: MessageTrackingTags
        skips: skip topics. {downstream: upstream} by input-to-output order
               ex) {"/sensing/lidar/top/rectified/pointcloud_ex":
                    "/sensing/lidar/top/mirror_cropped/pointcloud_ex"}

        """
        self.topics = sorted(message_tracking_tags.all_topics())
        self.t2i = {t: i for i, t in enumerate(self.topics)}
        self.skips = skips
        n = len(self.topics)

        # edges from publisher to subscription
        self.topic_edges = [set() for _ in range(n)]
        # edges from subscription to publisher
        self.rev_edges = [set() for _ in range(n)]
        for out_topic in self.topics:
            in_topics = message_tracking_tags.in_topics(out_topic)

            out_id = self.t2i[out_topic]
            for in_topic in in_topics:
                if in_topic in skips.keys():
                    in_topic = skips[in_topic]
                in_id = self.t2i[in_topic]
                self.topic_edges[in_id].add(out_id)
                self.rev_edges[out_id].add(in_id)

    def dump(self, fname):
        """Dump."""
        out = {}
        out['topics'] = self.topics
        out['topic2id'] = self.t2i
        out['topic_edges'] = [list(edge) for edge in self.topic_edges]
        out['rev_edges'] = [list(edge) for edge in self.rev_edges]

        json.dump(out, open(fname, 'wt'))

    def rev_topics(self, topic):
        """
        Get input topics of the target topic.

        Parameters
        ----------
        topic: topic name

        Return
        ------
        List[Topic]

        """
        out_topic_idx = self.t2i[topic]
        return [self.topics[i] for i in self.rev_edges[out_topic_idx]]

    def dfs_rev(self, start_topic):
        """
        Do depth first search from the topic.

        Parameters
        ----------
        start_topic: topic name

        Return
        ------
        topic names in appearance order

        """
        edges = self.rev_edges
        n = len(edges)
        seen = [False for _ in range(n)]
        sid = self.t2i[start_topic]

        ret = []

        def dfs(v):
            ret.append(self.topics[v])
            seen[v] = True
            for nxt in edges[v]:
                if seen[nxt]:
                    continue
                dfs(nxt)
        dfs(sid)

        return ret

    def bfs_rev(self, start_topic):
        """
        Do breadth first search from the topic.

        Parameters
        ----------
        start_topic: topic name

        Return
        ------
        topic names in appearance order

        """
        edges = self.rev_edges
        n = len(edges)
        dist = [-1 for _ in range(n)]
        queue = deque()

        sid = self.t2i[start_topic]
        dist[sid] = 0
        queue.append(sid)
        paths = []

        while len(queue) != 0:
            v = queue.popleft()
            paths.append((self.topics[v], len(edges[v]) == 0))
            for nv in edges[v]:
                if dist[nv] >= 0:
                    continue
                dist[nv] = dist[v] + 1
                queue.append(nv)

        return paths


def run(args):
    """Run."""
    pickle_file = args.pickle_file
    message_tracking_tags = pickle.load(open(pickle_file, 'rb'))

    tgt_topic = args.topic
    tgt_stamp = sorted(message_tracking_tags.stamps(tgt_topic))[args.stamp_index]

    graph = TopicGraph(message_tracking_tags)

    print('dump')
    graph.dump('graph.json')
    pickle.dump(graph, open('graph.pkl', 'wb'),
                protocol=pickle.HIGHEST_PROTOCOL)

    # bfs_path = graph.bfs_rev(tgt_topic)
    # print(f"BFS from {tgt_topic}")
    # for p, is_leaf in bfs_path:
    #     print(f"  {p} {is_leaf}")
    # print("")

    st = time.time()
    solver = InputSensorStampSolver(graph)
    solver.solve(message_tracking_tags, tgt_topic, tgt_stamp)
    et = time.time()

    print(f'solve {(et-st) * 1000} [ms]')


def main():
    """Run main."""
    parser = argparse.ArgumentParser()
    parser.add_argument('pickle_file')
    parser.add_argument('stamp_index', type=int, default=0,
                        help='header stamp index')
    parser.add_argument('topic',
                        default='/sensing/lidar/no_ground/pointcloud',
                        nargs='?')

    args = parser.parse_args()

    run(args)


if __name__ == '__main__':
    main()
