#!/usr/bin/python3

import unittest

import rclpy
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

from pathnode_vis.data_as_tree import TreeNode
from pathnode_vis.pub_info import (
    PubInfo
    )
from pathnode_vis.latency_viewer import (
    calc_stat,
    update_stat,
    LatencyViewerNode,
    )
from path_info_msg.msg import (
    PubInfo as PubInfoMsg,
    )



class TestCalcStat(unittest.TestCase):
    def test_none(self):
        root = TreeNode("root")
        root.data = [(None, None), (None, None), (None, None)]
        ret = calc_stat(root)

        self.assertEqual(len(ret), 1)
        self.assertEqual(ret[0]["dur_min"], None)
        self.assertEqual(ret[0]["dur_mean"], None)

    def test_simple(self):
        root = TreeNode("root")
        root.data = [(1, 10), (2, 20), (3, 30)]
        ret = calc_stat(root)

        self.assertEqual(len(ret), 1)
        self.assertEqual(ret[0]["dur_min"], 1)
        self.assertEqual(ret[0]["dur_mean"], 2)
        self.assertEqual(ret[0]["dur_max"], 3)
        self.assertEqual(ret[0]["dur_min_steady"], 10)
        self.assertEqual(ret[0]["dur_mean_steady"], 20)
        self.assertEqual(ret[0]["dur_max_steady"], 30)


def time_msg(sec, ms):
    """Get builtin.msg.Time"""
    return Time(seconds=sec, nanoseconds=ms * 10**6).to_msg()


def get_solver_result_simple(
        t1_pub,
        t1_sub, t2_pub,
        t2_sub, t3_pub):
    """Generate solver result
    Graph: topic1 --> topic2 --> topic3

    You can specify each pub/sub time.
    Assume:
    - header_stamp is preserved at topic1 pub time.
    - stamp and stamp_steady are same
    """
    tree_node3 = TreeNode("topic3")
    tree_node2 = tree_node3.get_child("topic2")
    tree_node1 = tree_node2.get_child("topic1")

    stamp1 = t1_pub
    pubinfo3 = PubInfo(
        "topic3",
        t3_pub, t3_pub, stamp1)
    pubinfo3.add_input_info(
        "topic2", t2_sub, t2_sub,
        True, stamp1)
    tree_node3.add_data(pubinfo3)

    pubinfo2 = PubInfo(
        "topic2",
        t2_pub, t2_pub, stamp1)
    pubinfo2.add_input_info(
        "topic1", t1_sub, t1_sub,
        True, stamp1)
    tree_node2.add_data(pubinfo2)

    pubinfo1 = PubInfo(
        "topic1",
        t1_pub, t1_pub, stamp1)
    tree_node1.add_data(pubinfo1)

    return tree_node3


class TestUpdateStat(unittest.TestCase):
    def test_simple(self):
        """
        pub/sub time
        (case1)    10   11     12  13    14
        (case2)    20   24     28  30    35

        t1 means:
           topic1 pub at t=10 and subscribed at t=11
           topic2 pub at t=12 and subscribed at t=13
           topic3 pub at t=14
        """
        results_case1 = get_solver_result_simple(
            time_msg(0, 10),
            time_msg(0, 11), time_msg(0, 12),
            time_msg(0, 13), time_msg(0, 14))

        update_stat_case1 = update_stat(results_case1)
        topic3_result_data = update_stat_case1.data
        self.assertEqual(topic3_result_data[0], (0, 0))

        topic2_result_data = update_stat_case1.name2child["topic2"].data
        self.assertEqual(topic2_result_data[0], (2, 2))

        topic1_result_data = \
            update_stat_case1.name2child["topic2"].name2child["topic1"].data
        self.assertEqual(topic1_result_data[0], (4, 4))


class TestListenerCallback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_seq(self):
        topic_name = "topic"
        node = LatencyViewerNode()
        msg = PubInfoMsg()
        msg.output_info.topic_name = topic_name

        msg.output_info.seq = 0
        msg.output_info.header_stamp = TimeMsg(sec=10, nanosec=0)
        node.listener_callback(msg)
        self.assertTrue(topic_name in node.topic_seq)
        self.assertEqual(len(node.pub_infos.stamps(topic_name)), 1)
        self.assertEqual(node.topic_seq[topic_name], 0)

        msg.output_info.seq = 1
        msg.output_info.header_stamp = TimeMsg(sec=11, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.pub_infos.stamps(topic_name)), 2)
        self.assertEqual(node.topic_seq[topic_name], 1)

        msg.output_info.seq = 3
        msg.output_info.header_stamp = TimeMsg(sec=13, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.pub_infos.stamps(topic_name)), 3)
        self.assertEqual(node.topic_seq[topic_name], 3)

        msg.output_info.seq = 2
        msg.output_info.header_stamp = TimeMsg(sec=12, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.pub_infos.stamps(topic_name)), 4)
        self.assertEqual(node.topic_seq[topic_name], 3)

        node.destroy_node()

    def test_seq_non_zero_start(self):
        topic_name = "topic"
        node = LatencyViewerNode()
        msg = PubInfoMsg()
        msg.output_info.topic_name = topic_name

        msg.output_info.seq = 10
        msg.output_info.header_stamp = TimeMsg(sec=10, nanosec=0)
        node.listener_callback(msg)
        self.assertTrue(topic_name in node.topic_seq)
        self.assertEqual(node.topic_seq[topic_name], 10)

        msg.output_info.seq = 20
        msg.output_info.header_stamp = TimeMsg(sec=20, nanosec=0)
        node.listener_callback(msg)
        self.assertTrue(topic_name in node.topic_seq)
        self.assertEqual(node.topic_seq[topic_name], 20)

        msg.output_info.seq = 15
        msg.output_info.header_stamp = TimeMsg(sec=15, nanosec=0)
        node.listener_callback(msg)
        self.assertTrue(topic_name in node.topic_seq)
        self.assertEqual(node.topic_seq[topic_name], 20)

        node.destroy_node()


if __name__ == '__main__':
    unittest.main()
