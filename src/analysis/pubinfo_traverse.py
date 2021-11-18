#!/usr/bin/python3

import pickle
import argparse
from collections import deque, defaultdict

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from pub_info import time2str, PubInfo, PubInfos

def strstamp2time(strstamp):
    sec, nanosec = strstamp.split(".")
    return Time(seconds=int(sec), nanoseconds=int(nanosec))

class InputSensorStampSolver(object):
    def __init__(self):
        # {topic: {stamp: {sensor_topic: [stamps]}}}
        self.topic_stamp_to_sensor_stamp = {}

    def solve(self, pubinfos, tgt_topic, tgt_stamp):
        """
        topic: target topic
        stamp: target stamp(str)

        return list of {sensor: oldest stamp}

        Calcurate topic_stamp_to_sensor_stamp internally.
        """
        graph = TopicGraph(pubinfos)
        path_bfs = graph.bfs_rev(tgt_topic)
        is_leaf = {t: b for (t, b) in path_bfs}

        wants = []  # topic, stamp
        stamp = tgt_stamp

        # dists[topic][stamp]
        dists = defaultdict(lambda: defaultdict(lambda: -1))
        queue = deque()

        dists[tgt_topic][tgt_stamp] = 1
        queue.append((tgt_topic, tgt_stamp))

        start = None
        while len(queue) != 0:
            topic, stamp = queue.popleft()
            is_leaf_s = "looks_sensor" if is_leaf[topic] else ""

            if start is None:
                start = strstamp2time(stamp)
            dur = start - strstamp2time(stamp)
            dur_ms = dur.nanoseconds // 10**6

            print(f"{topic:80} {stamp:>20} {dur_ms:4} ms {is_leaf_s}")

            # NDT-EKF has loop, so skip
            if topic == "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias":
                continue

            # get next edges
            next_pubinfo = pubinfos.get(topic, stamp)
            if not next_pubinfo:
                continue

            for in_info in next_pubinfo.in_infos.values():
                nx_topic = in_info.topic
                nx_stamp = time2str(in_info.stamp)
                if dists[nx_topic][nx_stamp] > 0:
                    continue
                dists[nx_topic][nx_stamp] = dists[topic][stamp] + 1
                queue.append((nx_topic, nx_stamp))


    def append(self, topic, stamp, sensor_topic, sensor_stamp):
        dic = self.topic_stamp_to_sensor_stamp
        if topic not in dic.keys():
            dic[topic] = {}
        if stamp not in dic[topic].keys():
            dic[topic][stamp] = {}
        if sensor_topic not in dic[topic][stamp].keys():
            dic[topic][stamp][sensor_topic] = []

        dic[topic][stamp][sensor_topic].append(sensor_stamp)


class TopicGraph(object):
    "Construct topic graph by ignoring stamps"

    def __init__(self, pubinfos):
        self.topics = sorted(pubinfos.all_topics())
        self.t2i = {t: i for i, t in enumerate(self.topics)}
        n = len(self.topics)

        # from sub -> pub
        self.topic_edges = [set() for _ in range(n)]
        # from out -> in
        self.rev_edges = [set() for _ in range(n)]
        for out_topic in self.topics:
            in_topics = pubinfos.in_topics(out_topic)

            out_id = self.t2i[out_topic]
            for in_topic in in_topics:
                in_id = self.t2i[in_topic]
                self.topic_edges[in_id].add(out_id)
                self.rev_edges[out_id].add(in_id)

    def rev_topics(self, topic):
        """
        get input topics
        return List[Topic]
        """
        ins = self.t2i[topic]
        return [self.topics[i] for i in iself.rev_edges[i]]

    def dfs_rev(self, start_topic):
        '''
        traverse topic graph reversely from start_topic

        return topic names in appearance order
        '''
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
        return list of (topic, is_leaf)
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

def main(args):
    pklfile = args.pickle_file
    pubinfos = pickle.load(open(pklfile, "rb"))

    tgt_topic = args.topic
    tgt_stamp = sorted(pubinfos.stamps(tgt_topic))[args.stamp_index]

    graph = TopicGraph(pubinfos)
    bfs_path = graph.bfs_rev(tgt_topic)

    # print(f"BFS from {tgt_topic}")
    # for p, is_leaf in bfs_path:
    #     print(f"  {p} {is_leaf}")
    # print("")

    solver = InputSensorStampSolver()
    solver.solve(pubinfos, tgt_topic, tgt_stamp)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("pickle_file")
    parser.add_argument("stamp_index", type=int, default=0,
                        help="header stamp index")
    parser.add_argument("topic", default="/sensing/lidar/no_ground/pointcloud", nargs="?")

    args = parser.parse_args()

    main(args)
