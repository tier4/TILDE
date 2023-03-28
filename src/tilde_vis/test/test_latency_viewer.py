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

import unittest

from builtin_interfaces.msg import Time as TimeMsg
import rclpy
from rclpy.time import Time

from tilde_msg.msg import (
    MessageTrackingTag as MessageTrackingTagMsg,
    )
from tilde_vis.data_as_tree import TreeNode
from tilde_vis.latency_viewer import (
    calc_stat,
    LatencyViewerNode,
    update_stat,
    )
from tilde_vis.message_tracking_tag import (
    MessageTrackingTag
    )


class TestCalcStat(unittest.TestCase):

    def test_none(self):
        root = TreeNode('root')
        root.data = [(None, None), (None, None), (None, None)]
        ret = calc_stat(root)

        self.assertEqual(len(ret), 1)
        self.assertEqual(ret[0]['dur_min'], None)
        self.assertEqual(ret[0]['dur_mean'], None)

    def test_simple(self):
        root = TreeNode('root')
        root.data = [(1, 10), (2, 20), (3, 30)]
        ret = calc_stat(root)

        self.assertEqual(len(ret), 1)
        self.assertEqual(ret[0]['dur_min'], 1)
        self.assertEqual(ret[0]['dur_mean'], 2)
        self.assertEqual(ret[0]['dur_max'], 3)
        self.assertEqual(ret[0]['dur_min_steady'], 10)
        self.assertEqual(ret[0]['dur_mean_steady'], 20)
        self.assertEqual(ret[0]['dur_max_steady'], 30)


def time_msg(sec, ms):
    """Get builtin.msg.Time."""
    return Time(seconds=sec, nanoseconds=ms * 10**6).to_msg()


def get_solver_result_simple(
        t1_pub,
        t1_sub, t2_pub,
        t2_sub, t3_pub):
    """
    Generate solver result.

    Graph: topic1 --> topic2 --> topic3

    You can specify each pub/sub time.
    Assume:
    - header_stamp is preserved at topic1 pub time.
    - stamp and stamp_steady are same
    """
    tree_node3 = TreeNode('topic3')
    tree_node2 = tree_node3.get_child('topic2')
    tree_node1 = tree_node2.get_child('topic1')

    stamp1 = t1_pub
    message_tracking_tag3 = MessageTrackingTag(
        'topic3',
        t3_pub, t3_pub, True, stamp1)
    message_tracking_tag3.add_input_info(
        'topic2', t2_sub, t2_sub,
        True, stamp1)
    tree_node3.add_data(message_tracking_tag3)

    message_tracking_tag2 = MessageTrackingTag(
        'topic2',
        t2_pub, t2_pub, True, stamp1)
    message_tracking_tag2.add_input_info(
        'topic1', t1_sub, t1_sub,
        True, stamp1)
    tree_node2.add_data(message_tracking_tag2)

    message_tracking_tag1 = MessageTrackingTag(
        'topic1',
        t1_pub, t1_pub, True, stamp1)
    tree_node1.add_data(message_tracking_tag1)

    return tree_node3


class TestUpdateStat(unittest.TestCase):

    def test_simple(self):
        """
        Simple case.

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

        topic2_result_data = update_stat_case1.name2child['topic2'].data
        self.assertEqual(topic2_result_data[0], (2, 2))

        topic1_result_data = \
            update_stat_case1.name2child['topic2'].name2child['topic1'].data
        self.assertEqual(topic1_result_data[0], (4, 4))


class TestListenerCallback(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_seq(self):
        topic_name = 'topic'
        node = LatencyViewerNode()
        msg = MessageTrackingTagMsg()
        msg.output_info.topic_name = topic_name

        msg.output_info.seq = 0
        msg.output_info.header_stamp = TimeMsg(sec=10, nanosec=0)
        node.listener_callback(msg)
        self.assertTrue(topic_name in node.topic_seq)
        self.assertEqual(len(node.message_tracking_tags.stamps(topic_name)), 1)
        self.assertEqual(node.topic_seq[topic_name], 0)

        msg.output_info.seq = 1
        msg.output_info.header_stamp = TimeMsg(sec=11, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.message_tracking_tags.stamps(topic_name)), 2)
        self.assertEqual(node.topic_seq[topic_name], 1)

        msg.output_info.seq = 3
        msg.output_info.header_stamp = TimeMsg(sec=13, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.message_tracking_tags.stamps(topic_name)), 3)
        self.assertEqual(node.topic_seq[topic_name], 3)

        msg.output_info.seq = 2
        msg.output_info.header_stamp = TimeMsg(sec=12, nanosec=0)
        node.listener_callback(msg)
        self.assertEqual(len(node.message_tracking_tags.stamps(topic_name)), 4)
        self.assertEqual(node.topic_seq[topic_name], 3)

        node.destroy_node()

    def test_seq_non_zero_start(self):
        topic_name = 'topic'
        node = LatencyViewerNode()
        msg = MessageTrackingTagMsg()
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


class TestTimerCallback(unittest.TestCase):

    def setUp(self):
        rclpy.init()

    def tearDown(self):
        rclpy.shutdown()

    def test_issues23_1(self):
        topic_name = 'topic'
        node = LatencyViewerNode()
        node.target_topic = topic_name

        msg = MessageTrackingTagMsg()
        msg.output_info.topic_name = topic_name
        msg.output_info.seq = 0
        msg.output_info.has_header_stamp = False
        msg.output_info.header_stamp = TimeMsg(sec=10, nanosec=0)

        node.listener_callback(msg)
        self.assertEqual(len(node.message_tracking_tags.topics()), 1)
        self.assertEqual(node.message_tracking_tags.topics()[0], topic_name)
        stamps = node.message_tracking_tags.stamps(topic_name)
        self.assertEqual(len(stamps), 1)

        node.wait_init = node.wait_sec_to_init_graph + 1

        try:
            node.timer_callback()
            self.assertTrue(True)
        except AttributeError:
            self.fail(msg='timer_callback causes AttributeError')

    def test_issues23_2(self):
        topic_name = 'topic'
        node = LatencyViewerNode()

        node.target_topic = topic_name

        msg = MessageTrackingTagMsg()
        msg.output_info.topic_name = topic_name
        msg.output_info.seq = 0
        msg.output_info.has_header_stamp = True
        msg.output_info.header_stamp = TimeMsg(sec=10, nanosec=0)

        node.listener_callback(msg)
        self.assertEqual(len(node.message_tracking_tags.topics()), 1)
        self.assertEqual(node.message_tracking_tags.topics()[0], topic_name)
        stamps = node.message_tracking_tags.stamps(topic_name)
        self.assertEqual(len(stamps), 1)

        node.wait_init = node.wait_sec_to_init_graph + 1

        try:
            node.timer_callback()
            self.assertTrue(True)
        except AttributeError:
            self.fail(msg='timer_callback causes AttributeError')


if __name__ == '__main__':
    unittest.main()
