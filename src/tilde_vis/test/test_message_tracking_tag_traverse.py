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

from rclpy.clock import ClockType
from rclpy.duration import Duration
from rclpy.time import Time

from tilde_vis.latency_viewer import (
    calc_one_hot,
    update_stat,
    )
from tilde_vis.message_tracking_tag import MessageTrackingTag, MessageTrackingTags
from tilde_vis.message_tracking_tag_traverse import (
    InputSensorStampSolver,
    TopicGraph,
    )


def get_scenario1():
    """
    Get the following scenario.

    topic1 --> topic2 --> topic3

    topic1 is published in 10Hz.
    topic2 takes 10ms to execute callback.


    """
    def get_topic2_times(start_time, start_time_steady):
        sub_time = start_time + nw_dur
        pub_time = sub_time + cb_dur

        sub_time_steady = start_time_steady + nw_dur
        pub_time_steady = sub_time_steady + cb_dur

        return {'sub': sub_time,
                'pub': pub_time,
                'sub_steady': sub_time_steady,
                'pub_steady': pub_time_steady}

    def get_topic3_times(start_time, start_time_steady):
        sub_time = start_time + nw_dur + cb_dur + nw_dur
        pub_time = sub_time + cb_dur

        sub_time_steady = start_time_steady + nw_dur + cb_dur + nw_dur
        pub_time_steady = sub_time_steady + cb_dur

        return {'sub': sub_time,
                'pub': pub_time,
                'sub_steady': sub_time_steady,
                'pub_steady': pub_time_steady}

    t1 = Time(seconds=10, nanoseconds=0)
    t1_steady = Time(seconds=0, nanoseconds=1,
                     clock_type=ClockType.STEADY_TIME)
    nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
    cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]

    message_tracking_tag_topic1_t1 = \
        MessageTrackingTag(
            'topic1',
            t1.to_msg(),
            t1_steady.to_msg(),
            True,
            t1.to_msg())

    topic2_t1_times = get_topic2_times(t1, t1_steady)
    message_tracking_tag_topic2_t1 = \
        MessageTrackingTag(
            'topic2',
            topic2_t1_times['pub'].to_msg(),
            topic2_t1_times['pub_steady'].to_msg(),
            True,
            t1.to_msg())
    message_tracking_tag_topic2_t1.add_input_info(
        'topic1',
        topic2_t1_times['sub'],
        topic2_t1_times['sub_steady'],
        True,
        t1.to_msg())

    topic3_t1_times = get_topic3_times(t1, t1_steady)
    message_tracking_tag_topic3_t1 = \
        MessageTrackingTag(
            'topic3',
            topic3_t1_times['pub'].to_msg(),
            topic3_t1_times['pub_steady'].to_msg(),
            True,
            t1.to_msg())
    message_tracking_tag_topic3_t1.add_input_info(
        'topic2',
        topic3_t1_times['sub'],
        topic3_t1_times['sub_steady'],
        True,
        t1.to_msg())

    return [message_tracking_tag_topic1_t1,
            message_tracking_tag_topic2_t1,
            message_tracking_tag_topic3_t1]


def dur_plus(lhs, rhs):
    """Plus of Duration."""
    return Duration(nanoseconds=lhs.nanoseconds + rhs.nanoseconds)


def gen_scenario2(
        st, st_steady,
        nw_dur, cb_dur):
    """
    Get the following scenario.

    topic1 --> topic3 --> topic4
    topic2 ---/

    Args:
      st: Time
      st_steady: Time
      nw_dur: Duration
      cb_dur: Duration

    Returns
    -------
    list of MessageTrackingTags such as
    (message_tracking_tag of topic1, message_tracking_tag of topic2, ...)

    """
    message_tracking_tag_topic1 = \
        MessageTrackingTag(
            'topic1',
            st.to_msg(),
            st_steady.to_msg(),
            True,
            st.to_msg())

    topic2_stamp = st + nw_dur
    message_tracking_tag_topic2 = \
        MessageTrackingTag(
            'topic2',
            st.to_msg(),
            st_steady.to_msg(),
            True,
            topic2_stamp.to_msg())

    sub_dur3 = nw_dur
    pub_dur3 = dur_plus(sub_dur3, cb_dur)
    topic3_stamp = st + pub_dur3
    message_tracking_tag_topic3 = \
        MessageTrackingTag(
            'topic3',
            (st + pub_dur3).to_msg(),
            (st_steady + pub_dur3).to_msg(),
            True,
            topic3_stamp.to_msg())
    message_tracking_tag_topic3.add_input_info(
        'topic1',
        (st + sub_dur3).to_msg(),
        (st_steady + sub_dur3).to_msg(),
        True,
        st.to_msg()
        )
    message_tracking_tag_topic3.add_input_info(
        'topic2',
        (st + sub_dur3).to_msg(),
        (st_steady + sub_dur3).to_msg(),
        True,
        topic2_stamp.to_msg())

    sub_dur4 = dur_plus(pub_dur3, nw_dur)
    pub_dur4 = dur_plus(sub_dur4, cb_dur)
    topic4_stamp = st + pub_dur4
    message_tracking_tag_topic4 = \
        MessageTrackingTag(
            'topic4',
            (st + pub_dur4).to_msg(),
            (st_steady + pub_dur4).to_msg(),
            True,
            topic4_stamp.to_msg())
    message_tracking_tag_topic4.add_input_info(
        'topic3',
        (st + sub_dur4).to_msg(),
        (st_steady + sub_dur4).to_msg(),
        True,
        topic3_stamp.to_msg())

    return [
        message_tracking_tag_topic1,
        message_tracking_tag_topic2,
        message_tracking_tag_topic3,
        message_tracking_tag_topic4,
        ]


class TestTopicGraph(unittest.TestCase):

    def test_straight(self):
        infos = get_scenario1()
        message_tracking_tags = MessageTrackingTags()
        for info in infos:
            message_tracking_tags.add(info)

        graph = TopicGraph(message_tracking_tags, skips={})

        self.assertEqual(graph.topics[0], 'topic1')
        self.assertEqual(graph.topics[1], 'topic2')
        self.assertEqual(graph.topics[2], 'topic3')

        edges = graph.topic_edges
        self.assertEqual(edges[0], {1})
        self.assertEqual(edges[1], {2})
        self.assertEqual(edges[2], set())

        rev_edges = graph.rev_edges
        self.assertEqual(rev_edges[0], set())
        self.assertEqual(rev_edges[1], {0})
        self.assertEqual(rev_edges[2], {1})

    def test_scenario2(self):
        t0 = Time(seconds=10, nanoseconds=0)
        t0_steady = Time(seconds=0, nanoseconds=1,
                         clock_type=ClockType.STEADY_TIME)
        nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
        cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]

        infos = gen_scenario2(t0, t0_steady, nw_dur, cb_dur)
        message_tracking_tags = MessageTrackingTags()
        for info in infos:
            message_tracking_tags.add(info)

        graph = TopicGraph(message_tracking_tags, skips={})

        self.assertEqual(graph.topics[0], 'topic1')
        self.assertEqual(graph.topics[1], 'topic2')
        self.assertEqual(graph.topics[2], 'topic3')
        self.assertEqual(graph.topics[3], 'topic4')

        edges = graph.topic_edges
        self.assertEqual(edges[0], {2})
        self.assertEqual(edges[1], {2})
        self.assertEqual(edges[2], {3})
        self.assertEqual(edges[3], set())

        rev_edges = graph.rev_edges
        self.assertEqual(rev_edges[0], set())
        self.assertEqual(rev_edges[1], set())
        self.assertEqual(rev_edges[2], {0, 1})
        self.assertEqual(rev_edges[3], {2})

    def test_scenario2_with_loss(self):
        """
        Graph creation with lossy MessageTrackingTags.

        t0: message_tracking_tag only topic1 and topic3
        t1: only topic2
        t2: only topic4
        """
        t0 = Time(seconds=10, nanoseconds=0)
        t0_steady = Time(seconds=0, nanoseconds=1,
                         clock_type=ClockType.STEADY_TIME)
        nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
        cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]
        period = Duration(seconds=1)
        t1, t1_steady = t0 + period, t0_steady + period
        t2, t2_steady = t1 + period, t1_steady + period

        infos_t0 = gen_scenario2(t0, t0_steady, nw_dur, cb_dur)
        infos_t1 = gen_scenario2(t1, t1_steady, nw_dur, cb_dur)
        infos_t2 = gen_scenario2(t2, t2_steady, nw_dur, cb_dur)
        message_tracking_tags = MessageTrackingTags()

        message_tracking_tags.add(infos_t0[0])
        message_tracking_tags.add(infos_t0[2])
        message_tracking_tags.add(infos_t1[1])
        message_tracking_tags.add(infos_t2[3])

        graph = TopicGraph(message_tracking_tags, skips={})

        self.assertEqual(graph.topics[0], 'topic1')
        self.assertEqual(graph.topics[1], 'topic2')
        self.assertEqual(graph.topics[2], 'topic3')
        self.assertEqual(graph.topics[3], 'topic4')

        edges = graph.topic_edges
        self.assertEqual(edges[0], {2})
        self.assertEqual(edges[1], {2})
        self.assertEqual(edges[2], {3})
        self.assertEqual(edges[3], set())

        rev_edges = graph.rev_edges
        self.assertEqual(rev_edges[0], set())
        self.assertEqual(rev_edges[1], set())
        self.assertEqual(rev_edges[2], {0, 1})
        self.assertEqual(rev_edges[3], {2})


class TestTreeNode(unittest.TestCase):

    def test_solve2_straight(self):
        infos = get_scenario1()
        message_tracking_tags = MessageTrackingTags()
        for info in infos:
            message_tracking_tags.add(info)
        graph = TopicGraph(message_tracking_tags, skips={})
        solver = InputSensorStampSolver(graph)

        tgt_stamp = sorted(message_tracking_tags.stamps('topic3'))[0]
        results = solver.solve2(
            message_tracking_tags,
            'topic3',
            tgt_stamp
        )

        r3 = results
        self.assertEqual(r3.name, 'topic3')
        self.assertEqual(len(r3.data), 1)
        self.assertEqual(len(r3.data[0].in_infos['topic2']), 1)
        self.assertEqual(r3.data[0].in_infos['topic2'][0].stamp.sec, 10)

        r2 = r3.name2child['topic2']
        self.assertEqual(r2.name, 'topic2')
        self.assertEqual(len(r2.data), 1)
        self.assertEqual(len(r2.data[0].in_infos['topic1']), 1)
        self.assertEqual(r2.data[0].in_infos['topic1'][0].stamp.sec, 10)

        r1 = r2.name2child['topic1']
        self.assertEqual(r1.name, 'topic1')
        self.assertEqual(len(r1.data), 1)
        self.assertEqual(len(r1.data[0].in_infos.keys()), 0)

    def test_solve2_branched(self):
        t0 = Time(seconds=10, nanoseconds=0)
        t0_steady = Time(seconds=0, nanoseconds=1,
                         clock_type=ClockType.STEADY_TIME)
        period = Duration(nanoseconds=100 * 10**6)
        nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
        cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]

        message_tracking_tags = MessageTrackingTags()
        st = t0
        st_steady = t0_steady

        # st: 10.0, 10.1, 10.2, ...

        for i in range(100):
            if i == 75:
                test_st = Time.from_msg(st.to_msg())

            infos = gen_scenario2(st, st_steady, nw_dur, cb_dur)
            for info in infos:
                message_tracking_tags.add(info)
            st += period
            st_steady += period

        graph = TopicGraph(message_tracking_tags, skips={})
        solver = InputSensorStampSolver(graph)

        sorted_stamps = sorted(message_tracking_tags.stamps('topic4'))

        tgt_stamp = sorted_stamps[75]
        results = solver.solve2(
            message_tracking_tags,
            'topic4',
            tgt_stamp)

        test_stamp1 = test_st
        test_stamp2 = test_st + nw_dur
        test_stamp3 = test_st + nw_dur + cb_dur
        test_stamp4 = test_stamp3 + nw_dur + cb_dur

        r4 = results
        self.assertEqual(r4.name, 'topic4')
        self.assertEqual(r4.data[0].out_info.stamp,
                         test_stamp4.to_msg())
        self.assertEqual(len(r4.data), 1)
        self.assertEqual(len(r4.data[0].in_infos['topic3']), 1)
        self.assertEqual(r4.data[0].in_infos['topic3'][0].stamp,
                         test_stamp3.to_msg())

        r3 = r4.name2child['topic3']
        self.assertEqual(r3.name, 'topic3')
        self.assertEqual(len(r3.data), 1)
        self.assertEqual(len(r3.data[0].in_infos['topic2']), 1)
        self.assertEqual(r3.data[0].in_infos['topic2'][0].stamp,
                         test_stamp2.to_msg())
        self.assertEqual(len(r3.data[0].in_infos['topic1']), 1)
        self.assertEqual(r3.data[0].in_infos['topic1'][0].stamp,
                         test_stamp1.to_msg())

        one_hot_durations = calc_one_hot(results)
        self.assertEqual(len(one_hot_durations), 4)
        dur4 = one_hot_durations[0]
        self.assertEqual(dur4[1], 'topic4')
        self.assertEqual(dur4[3], 0)

        dur3 = one_hot_durations[1]
        self.assertEqual(dur3[1], 'topic3')
        self.assertEqual(dur3[3], 11)

        dur2 = one_hot_durations[3]
        self.assertEqual(dur2[1], 'topic2')
        self.assertEqual(dur2[3], 22)

        dur1 = one_hot_durations[2]
        self.assertEqual(dur1[1], 'topic1')
        self.assertEqual(dur1[3], 22)

    def setup_only_topic4_scenario(self):
        """Test lossy scenario."""
        t0 = Time(seconds=10, nanoseconds=0)
        t0_steady = Time(seconds=0, nanoseconds=1,
                         clock_type=ClockType.STEADY_TIME)
        nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
        cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]
        period = Duration(seconds=1)

        def init_solver():
            infos = gen_scenario2(t0, t0_steady, nw_dur, cb_dur)
            message_tracking_tags = MessageTrackingTags()
            for info in infos:
                message_tracking_tags.add(info)
            graph = TopicGraph(message_tracking_tags, skips={})
            solver = InputSensorStampSolver(graph)
            return solver

        def get_message_tracking_tags_with_only_topic4():
            t1 = t0 + period
            t1_steady = t0_steady + period
            infos = gen_scenario2(t1, t1_steady, nw_dur, cb_dur)
            message_tracking_tags = MessageTrackingTags()
            message_tracking_tags.add(infos[-1])
            return message_tracking_tags

        solver = init_solver()
        tgt_message_tracking_tags = get_message_tracking_tags_with_only_topic4()

        return (solver, tgt_message_tracking_tags)

    def test_solve_empty(self):
        # setup
        solver, _ = self.setup_only_topic4_scenario()
        tgt_topic = 'topic4'

        results = solver._solve_empty(tgt_topic)

        result_topic4 = results
        self.assertEqual(result_topic4.name, 'topic4')
        self.assertEqual(len(result_topic4.data), 0)

        result_topic3 = result_topic4.name2child['topic3']
        self.assertEqual(result_topic3.name, 'topic3')
        self.assertEqual(len(result_topic3.data), 0)

        result_topic2 = result_topic3 .name2child['topic2']
        self.assertEqual(result_topic2.name, 'topic2')
        self.assertEqual(len(result_topic2.data), 0)

        result_topic1 = result_topic3.name2child['topic1']
        self.assertEqual(result_topic1.name, 'topic1')
        self.assertEqual(len(result_topic1.data), 0)

    def test_solve2_with_loss(self):
        """Solve with lossy MessageTrackingTags."""
        solver, tgt_message_tracking_tags = self.setup_only_topic4_scenario()

        tgt_topic = 'topic4'
        tgt_stamps = tgt_message_tracking_tags.stamps(tgt_topic)

        self.assertEqual(len(tgt_stamps), 1)
        results = solver.solve2(tgt_message_tracking_tags, tgt_topic, tgt_stamps[0])

        # is there full graph?
        result_topic4 = results
        self.assertEqual(result_topic4.name, 'topic4')
        self.assertEqual(len(result_topic4.data), 1)
        self.assertTrue(isinstance(result_topic4.data[0], MessageTrackingTag))

        result_topic3 = result_topic4.name2child['topic3']
        self.assertEqual(result_topic3.name, 'topic3')
        self.assertEqual(len(result_topic3.data), 0)

        result_topic2 = result_topic3.name2child['topic2']
        self.assertEqual(result_topic2.name, 'topic2')
        self.assertEqual(len(result_topic2.data), 0)

        result_topic1 = result_topic3.name2child['topic1']
        self.assertEqual(result_topic1.name, 'topic1')
        self.assertEqual(len(result_topic1.data), 0)

    def test_update_stat(self):
        t = []
        ts = []
        nw_dur = []
        cb_dur = []

        t.append(Time(seconds=10, nanoseconds=0))
        ts.append(Time(seconds=0, nanoseconds=1,
                       clock_type=ClockType.STEADY_TIME))
        nw_dur.append(Duration(nanoseconds=10 * 10**6))
        cb_dur.append(Duration(nanoseconds=1 * 10**6))

        period1 = Duration(nanoseconds=100 * 10**6)
        t.append(t[-1] + period1)
        ts.append(ts[-1] + period1)
        nw_dur.append(Duration(nanoseconds=20 * 10**6))
        cb_dur.append(Duration(nanoseconds=2 * 10**6))

        period2 = Duration(nanoseconds=100 * 10**6)
        t.append(t[-1] + period2)
        ts.append(ts[-1] + period2)
        nw_dur.append(Duration(nanoseconds=30 * 10**6))
        cb_dur.append(Duration(nanoseconds=3 * 10**6))

        message_tracking_tags = MessageTrackingTags()

        for i in range(len(t)):
            infos = gen_scenario2(t[i], ts[i],
                                  nw_dur[i], cb_dur[i])
            for info in infos:
                message_tracking_tags.add(info)

        graph = TopicGraph(message_tracking_tags, skips={})
        solver = InputSensorStampSolver(graph)

        tgt_stamp = sorted(message_tracking_tags.stamps('topic4'))[-1]
        results = solver.solve2(
            message_tracking_tags,
            'topic4',
            tgt_stamp)
        results = update_stat(results)

        self.assertEqual(results.data, [(0, 0)])
        topic3 = results.get_child('topic3')
        self.assertEqual(topic3.data, [(33, 33)])

        topic2 = topic3.get_child('topic2')
        self.assertEqual(topic2.data, [(66, 66)])

        topic1 = topic3.get_child('topic1')
        self.assertEqual(topic1.data, [(66, 66)])


if __name__ == '__main__':
    unittest.main()
