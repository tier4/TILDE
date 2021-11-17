#!/usr/bin/env python3
# traversal topic
#   given path: topic1 -> node1 -> topic2 -> node2 -> topic3 -> node3 -> topic4
#   traverse topic2 to topic4, then "topic2 -> node2 -> topic3 -> node3 -> topic4" printed

import argparse

import rclpy
from rosgraph2_impl import Edge, Graph

EXCLUDE_TOPICS = [
    "/parameter_events",
    "/rosout",
    "/clock",
    ]

def main(args):
    rclpy.init()
    node = rclpy.create_node("topic_traversal_node")
    graph = Graph(node)
    graph.update()

    cnt = 0
    while cnt < 10 and len(graph.nt_edges.edges_by_end) <= len(EXCLUDE_TOPICS):
        cnt += 1
        graph = Graph(node)
        graph.update()

    if len(graph.nt_edges.edges_by_end) <= len(EXCLUDE_TOPICS):
        print("not enough edges")
        return

    print("found {} keys".format(len(graph.nt_edges.edges_by_end.keys())))
    for k in graph.nt_edges.edges_by_end.keys():
        print("  '{}'".format(k))

    def find_key(name):
        for key in graph.nt_edges.edges_by_end:
            if name in key and 0 <= len(key) - len(name) <= 2:
                # print("find_key: {} -> {}".format(name, key))
                return key
        return None

    name_from = find_key(args.frm)
    name_to = find_key(args.to)

    print("")
    print("args:  {} -> {}".format(args.frm, args.to))
    print("graph: {} -> {}".format(name_from, name_to))
    print("")

    seen = set()
    path = []

    def traverse(now):
        # print("path: {}, traverse {}".format(path, now))
        if now == None:
            return False
        if now in seen:  # basically, assume now is not in seen
            return False

        seen.add(now)
        path.append(now)
        if now == name_from:
            return True

        for nxt in graph.nt_edges.edges_by_end[now]:
            if isinstance(nxt, Edge):
                # print("nxt.start: {}".format(nxt.start))
                nxt = nxt.start
            nxt = nxt.strip()
            if nxt in EXCLUDE_TOPICS:
                continue
            # print("nxt: '{}'".format(nxt))
            nxt = find_key(nxt)
            if nxt in seen:
                continue
            # print("nxt: {}".format(nxt))
            if traverse(nxt):
                return True
        path.pop()

        return False

    if traverse(name_to):
        print("Found")
    path = path[::-1]
    # path = path[::2]

    for i in path:
        print("  '{}'".format(i))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("frm")
    parser.add_argument("to")
    args = parser.parse_args()
    main(args)
