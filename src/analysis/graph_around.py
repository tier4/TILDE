#!/usr/bin/env python3
# list around the node

import argparse
from collections import deque

import rclpy
from rosgraph2_impl import Edge

from graph_common import find_key, get_graph

EXCLUDE_TOPICS = [
    "/parameter_events",
    "/rosout",
    "/clock",
    ]


def main(args):
    frm = args.frm
    max_depth = args.depth
    least_edges = args.least_edges

    rclpy.init()
    graph = get_graph(least_edges)

    key = find_key(graph.nt_edges.edges_by_end, frm)
    if not key:
        print("{} not found".format(frm))
        return
    print("key: '{}'".format(key))

    seen = set()
    current_item = deque([key])
    next_item = deque([])
    depth = 0

    edges = graph.nt_edges.edges_by_start
    print("depth: {}".format(depth))
    while len(current_item) != 0 and depth <= max_depth:
        now = current_item.popleft()
        print("now: '{}'".format(now))

        for nxt in edges[now]:
            if isinstance(nxt, Edge):
                # print("nxt.start: {}".format(nxt.start))
                nxt = nxt.end
            nxt = nxt.strip()
            if nxt in EXCLUDE_TOPICS:
                continue
            nxt = find_key(edges, nxt)
            if nxt is None:
                continue
            if nxt in seen:
                continue
            seen.add(nxt)
            next_item.append(nxt)

            # for n in edges[nxt]:
            #     if isinstance(n, Edge):
            #         # print("nxt.start: {}".format(nxt.start))
            #         n = n.end
            #     n = n.strip()
            #     n = find_key(edges, n)
            #     if n in EXCLUDE_TOPICS:
            #         continue
            #     if n is None:
            #         continue
            #     if n in seen:
            #         continue
            #     next_item.append(n)

        if len(current_item) == 0:
            depth += 1
            current_item = next_item
            next_item = deque([])
            print("")
            if depth <= max_depth:
                print("depth: {}".format(depth))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("frm")
    parser.add_argument("--depth", type=int, default=2)
    parser.add_argument("--least-edges", type=int, default=200,
                        help="the least number of edges to wait for graph.update")
    args = parser.parse_args()
    main(args)
