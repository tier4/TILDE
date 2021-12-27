#!/usr/bin/python3

import unittest

from rclpy.clock import ClockType
from rclpy.time import Time
from rclpy.duration import Duration

from pathnode_vis.pub_info import PubInfo, PubInfos
from pathnode_vis.pubinfo_traverse import (
    InputSensorStampSolver,
    TopicGraph,
    )
from pathnode_vis.latency_viewer import (
    calc_one_hot,
    )


def get_scenario1():
    """Get the following scenario

    topic1 --> topic2 --> topic3

    topic1 is published in 10Hz.
    topic2 takes 10ms to execute callback.


    """
    def get_topic2_times(start_time, start_time_steady):
        sub_time = start_time + nw_dur
        pub_time = sub_time + cb_dur

        sub_time_steady = start_time_steady + nw_dur
        pub_time_steady = sub_time_steady + cb_dur

        return {"sub": sub_time,
                "pub": pub_time,
                "sub_steady": sub_time_steady,
                "pub_steady": pub_time_steady}

    def get_topic3_times(start_time, start_time_steady):
        sub_time = start_time + nw_dur + cb_dur + nw_dur
        pub_time = sub_time + cb_dur

        sub_time_steady = start_time_steady + nw_dur + cb_dur + nw_dur
        pub_time_steady = sub_time_steady + cb_dur

        return {"sub": sub_time,
                "pub": pub_time,
                "sub_steady": sub_time_steady,
                "pub_steady": pub_time_steady}

    t1 = Time(seconds=10, nanoseconds=0)
    t1_steady = Time(seconds=0, nanoseconds=1,
                     clock_type=ClockType.STEADY_TIME)
    nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
    cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]

    pubinfo_topic1_t1 = \
        PubInfo("topic1",
                t1.to_msg(),
                t1_steady.to_msg(),
                t1.to_msg())

    topic2_t1_times = get_topic2_times(t1, t1_steady)
    pubinfo_topic2_t1 = \
        PubInfo("topic2",
                topic2_t1_times["pub"].to_msg(),
                topic2_t1_times["pub_steady"].to_msg(),
                t1.to_msg())
    pubinfo_topic2_t1.add_input_info(
        "topic1",
        topic2_t1_times["sub"],
        topic2_t1_times["sub_steady"],
        True,
        t1.to_msg())

    topic3_t1_times = get_topic3_times(t1, t1_steady)
    pubinfo_topic3_t1 = \
        PubInfo("topic3",
                topic3_t1_times["pub"].to_msg(),
                topic3_t1_times["pub_steady"].to_msg(),
                t1.to_msg())
    pubinfo_topic3_t1.add_input_info(
        "topic2",
        topic3_t1_times["sub"],
        topic3_t1_times["sub_steady"],
        True,
        t1.to_msg())

    return [pubinfo_topic1_t1,
            pubinfo_topic2_t1,
            pubinfo_topic3_t1]


def dur_plus(lhs, rhs):
    """
    plus of Duration
    """
    return Duration(nanoseconds=lhs.nanoseconds + rhs.nanoseconds)


def gen_scenario2(
        st, st_steady,
        nw_dur, cb_dur):
    """Get the following scenario

    topic1 --> topic3 --> topic4
    topic2 ---/

    Parameters
    ----------
    st: Time
    st: Time
    nw_dur: Duration
    db_dur: Duration

    Returns
    -------
    list of PubInfos such as
    (pubinfo of topic1, pubinfo of topic2, ...)
    """
    pubinfo_topic1 = \
        PubInfo("topic1",
                st.to_msg(),
                st_steady.to_msg(),
                st.to_msg())

    topic2_stamp = st + nw_dur
    pubinfo_topic2 = \
        PubInfo("topic2",
                st.to_msg(),
                st_steady.to_msg(),
                topic2_stamp.to_msg())

    sub_dur3 = nw_dur
    pub_dur3 = dur_plus(sub_dur3, cb_dur)
    topic3_stamp = st + pub_dur3
    pubinfo_topic3 = \
        PubInfo("topic3",
                (st + pub_dur3).to_msg(),
                (st_steady + pub_dur3).to_msg(),
                topic3_stamp.to_msg())
    pubinfo_topic3.add_input_info(
        "topic1",
        (st + sub_dur3).to_msg(),
        (st_steady + sub_dur3).to_msg(),
        True,
        st.to_msg()
        )
    pubinfo_topic3.add_input_info(
        "topic2",
        (st + sub_dur3).to_msg(),
        (st_steady + sub_dur3).to_msg(),
        True,
        topic2_stamp.to_msg())

    sub_dur4 = dur_plus(pub_dur3, nw_dur)
    pub_dur4 = dur_plus(sub_dur4, cb_dur)
    topic4_stamp = st + pub_dur4
    pubinfo_topic4 = \
        PubInfo("topic4",
                (st + pub_dur4).to_msg(),
                (st_steady + pub_dur4).to_msg(),
                topic4_stamp.to_msg())
    pubinfo_topic4.add_input_info(
        "topic3",
        (st + sub_dur4).to_msg(),
        (st_steady + sub_dur4).to_msg(),
        True,
        topic3_stamp.to_msg())

    return [
        pubinfo_topic1,
        pubinfo_topic2,
        pubinfo_topic3,
        pubinfo_topic4,
        ]


class TestTreeNode(unittest.TestCase):
    def test_straight(self):
        infos = get_scenario1()
        pubinfos = PubInfos()
        for info in infos:
            pubinfos.add(info)
        graph = TopicGraph(pubinfos, skips={})
        solver = InputSensorStampSolver(graph)

        tgt_stamp = sorted(pubinfos.stamps("topic3"))[0]
        results = solver.solve2(
            pubinfos,
            "topic3",
            tgt_stamp
        )

        r3 = results
        self.assertEqual(r3.name, "topic3")
        self.assertEqual(len(r3.data), 1)
        self.assertEqual(len(r3.data[0].in_infos["topic2"]), 1)
        self.assertEqual(r3.data[0].in_infos["topic2"][0].stamp.sec, 10)

        r2 = r3.name2child["topic2"]
        self.assertEqual(r2.name, "topic2")
        self.assertEqual(len(r2.data), 1)
        self.assertEqual(len(r2.data[0].in_infos["topic1"]), 1)
        self.assertEqual(r2.data[0].in_infos["topic1"][0].stamp.sec, 10)

        r1 = r2.name2child["topic1"]
        self.assertEqual(r1.name, "topic1")
        self.assertEqual(len(r1.data), 1)
        self.assertEqual(len(r1.data[0].in_infos.keys()), 0)

    def test_branched(self):
        t0 = Time(seconds=10, nanoseconds=0)
        t0_steady = Time(seconds=0, nanoseconds=1,
                         clock_type=ClockType.STEADY_TIME)
        period = Duration(nanoseconds=100 * 10**6)
        nw_dur = Duration(nanoseconds=1 * 10**6)  # 1 [ms]
        cb_dur = Duration(nanoseconds=10 * 10**6)  # 10 [ms]

        pubinfos = PubInfos()
        st = t0
        st_steady = t0_steady

        # st: 10.0, 10.1, 10.2, ...
        # 

        for i in range(100):
            if i == 75:
                test_st = Time.from_msg(st.to_msg())
                print("test_st")
                print(test_st)

            infos = gen_scenario2(st, st_steady, nw_dur, cb_dur)
            for info in infos:
                pubinfos.add(info)
            st += period
            st_steady += period

        graph = TopicGraph(pubinfos, skips={})
        solver = InputSensorStampSolver(graph)

        sorted_stamps = sorted(pubinfos.stamps("topic4"))
        print(sorted_stamps[0])
        print(sorted_stamps[99])

        tgt_stamp = sorted_stamps[75]
        results = solver.solve2(
            pubinfos,
            "topic4",
            tgt_stamp)

        test_stamp1 = test_st
        test_stamp2 = test_st + nw_dur
        test_stamp3 = test_st + nw_dur + cb_dur
        test_stamp4 = test_stamp3 + nw_dur + cb_dur

        r4 = results
        self.assertEqual(r4.name, "topic4")
        self.assertEqual(r4.data[0].out_info.stamp,
                         test_stamp4.to_msg())
        self.assertEqual(len(r4.data), 1)
        self.assertEqual(len(r4.data[0].in_infos["topic3"]), 1)
        self.assertEqual(r4.data[0].in_infos["topic3"][0].stamp,
                         test_stamp3.to_msg())

        r3 = r4.name2child["topic3"]
        self.assertEqual(r3.name, "topic3")
        self.assertEqual(len(r3.data), 1)
        self.assertEqual(len(r3.data[0].in_infos["topic2"]), 1)
        self.assertEqual(r3.data[0].in_infos["topic2"][0].stamp,
                         test_stamp2.to_msg())
        self.assertEqual(len(r3.data[0].in_infos["topic1"]), 1)
        self.assertEqual(r3.data[0].in_infos["topic1"][0].stamp,
                         test_stamp1.to_msg())

        onehot_durs = calc_one_hot(results)
        self.assertEqual(len(onehot_durs), 4)
        dur4 = onehot_durs[0]
        self.assertEqual(dur4[1], "topic4")
        self.assertEqual(dur4[3], 0)

        dur3 = onehot_durs[1]
        self.assertEqual(dur3[1], "topic3")
        self.assertEqual(dur3[3], 11)

        dur2 = onehot_durs[3]
        self.assertEqual(dur2[1], "topic2")
        self.assertEqual(dur2[3], 22)

        dur1 = onehot_durs[2]
        self.assertEqual(dur1[1], "topic1")
        self.assertEqual(dur1[3], 22)


if __name__ == '__main__':
    unittest.main()
