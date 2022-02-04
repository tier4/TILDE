import rclpy
from rosgraph2_impl import Edge, Graph

def get_graph(least_num, prints_edge=False):
    node = rclpy.create_node("topic_traversal_node")
    graph = Graph(node)
    graph.update()

    cnt = 0
    while cnt < 10 and len(graph.nt_edges.edges_by_end) <= least_num:
        cnt += 1
        graph = Graph(node)
        graph.update()

    if len(graph.nt_edges.edges_by_end) <= least_num:
        raise Exception("not enough edges")

    print("found {} keys".format(len(graph.nt_edges.edges_by_end.keys())))

    if prints_edge:
        for k in graph.nt_edges.edges_by_end.keys():
            print("  '{}'".format(k))

    return graph

def find_key(edges, name):
    for key in edges:
        if name in key and 0 <= len(key) - len(name) <= 2:
            # print("find_key: {} -> {}".format(name, key))
            return key
    return None

