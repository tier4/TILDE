#!/usr/bin/python3

import pickle
import argparse
from collections import deque, defaultdict
import time
import json

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.time import Time

from pathnode_vis.pub_info import time2str, PubInfo, PubInfos

def strstamp2time(strstamp):
    sec, nanosec = strstamp.split(".")
    return Time(seconds=int(sec), nanoseconds=int(nanosec))

class SolverResult(object):
    def __init__(self, topic, stamp, dur_ms, dur_pub_ms, is_leaf, parent):
        """
        topic: topic [string]
        stamp: header.stamp [rclpy.Time]
        dur_ms: duration of header.stamp in ms [double]
        dur_pub_ms: duration of output.pub_time in ms [double]
        is_leaf: bool
        parent: parent topic [string]
        """
        self.topic = topic
        self.stamp = stamp
        self.dur_ms = dur_ms
        self.dur_pub_ms = dur_pub_ms
        self.is_leaf = is_leaf
        self.parent = parent

class SolverResults(object):
    def __init__(self):
        self.data = []

    def add(self, *args):
        self.data.append(SolverResult(*args))

class InputSensorStampSolver(object):
    def __init__(self, graph):
        # {topic: {stamp: {sensor_topic: [stamps]}}}
        self.topic_stamp_to_sensor_stamp = {}
        self.graph = graph

    def solve(self, pubinfos, tgt_topic, tgt_stamp):
        """
        topic: target topic
        stamp: target stamp(str)

        return SolverResults

        Calcurate topic_stamp_to_sensor_stamp internally.
        """
        graph = self.graph
        path_bfs = graph.bfs_rev(tgt_topic)
        is_leaf = {t: b for (t, b) in path_bfs}

        wants = []  # topic, stamp
        stamp = tgt_stamp

        # dists[topic][stamp]
        dists = defaultdict(lambda: defaultdict(lambda: -1))
        queue = deque()
        parentQ = deque()

        start = strstamp2time(tgt_stamp)
        start_pub_info = pubinfos.get(tgt_topic, tgt_stamp)
        start_pub_time = start_pub_info.out_info.pubsub_stamp

        dists[tgt_topic][tgt_stamp] = 1
        queue.append((tgt_topic, tgt_stamp, start_pub_time))
        parentQ.append("")

        st = time.time()

        ret = SolverResults()
        while len(queue) != 0:
            topic, stamp, sub_time = queue.popleft()
            parent = parentQ.popleft()
            is_leaf_s = "looks_sensor" if is_leaf[topic] else ""

            dur = start - strstamp2time(stamp)
            dur_ms = dur.nanoseconds // 10**6

            dur_pub = Time.from_msg(start_pub_time) - Time.from_msg(sub_time)
            dur_pub_ms = dur_pub.nanoseconds // 10**6

            # print(f"{topic:80} {stamp:>20} {dur_ms:4} ms {is_leaf_s} {parent}")
            ret.add(topic, stamp, dur_ms, dur_pub_ms, is_leaf[topic], parent)

            # NDT-EKF has loop, so skip
            if topic == "/localization/pose_twist_fusion_filter/pose_with_covariance_without_yawbias":
                continue

            # get next edges
            next_pubinfo = pubinfos.get(topic, stamp)
            if not next_pubinfo:
                continue

            for in_infos in next_pubinfo.in_infos.values():
                for in_info in in_infos:
                    nx_topic = in_info.topic
                    nx_stamp = time2str(in_info.stamp)
                    if dists[nx_topic][nx_stamp] > 0:
                        continue
                    dists[nx_topic][nx_stamp] = dists[topic][stamp] + 1
                    queue.append((nx_topic, nx_stamp, in_info.pubsub_stamp))
                    parentQ.append(topic)

        et = time.time()
        # print(f"solve internal: {(et-st)*1000} [ms]")

        return ret

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

    def dump(self, fname):
        out = {}
        out["topics"] = self.topics
        out["topic2id"] = self.t2i
        out["topic_edges"] = [list(l) for l in self.topic_edges]
        out["rev_edges"] = [list(l) for l in self.rev_edges]

        json.dump(out, open(fname, "wt"))

    def rev_topics(self, topic):
        """
        get input topics
        return List[Topic]
        """
        ins = self.t2i[topic]
        return [self.topics[i] for i in self.rev_edges[i]]

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

def run(args):
    pklfile = args.pickle_file
    pubinfos = pickle.load(open(pklfile, "rb"))

    tgt_topic = args.topic
    tgt_stamp = sorted(pubinfos.stamps(tgt_topic))[args.stamp_index]

    graph = TopicGraph(pubinfos)
    bfs_path = graph.bfs_rev(tgt_topic)
    print("dump")
    graph.dump("graph.json")
    pickle.dump(graph, open("graph.pkl", "wb"), protocol=pickle.HIGHEST_PROTOCOL)

    # print(f"BFS from {tgt_topic}")
    # for p, is_leaf in bfs_path:
    #     print(f"  {p} {is_leaf}")
    # print("")

    st = time.time()
    solver = InputSensorStampSolver(graph)
    solver.solve(pubinfos, tgt_topic, tgt_stamp)
    et = time.time()

    print(f"solve {(et-st) * 1000} [ms]")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pickle_file")
    parser.add_argument("stamp_index", type=int, default=0,
                        help="header stamp index")
    parser.add_argument("topic", default="/sensing/lidar/no_ground/pointcloud", nargs="?")

    args = parser.parse_args()

    run(args)

if __name__ == "__main__":
    main()
