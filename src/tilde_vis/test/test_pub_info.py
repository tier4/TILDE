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

from rclpy.time import Time

from tilde_vis.pub_info import (
    PubInfo,
    PubInfos
    )


def time_msg(sec, ms):
    """Get builtin.msg.Time"""
    return Time(seconds=sec, nanoseconds=ms * 10**6).to_msg()


class TestPubInfos(unittest.TestCase):
    def test_erase_until(self):
        infos = PubInfos()
        infos.add(PubInfo(
            "topic1",
            time_msg(8, 0),
            time_msg(9, 0),
            time_msg(10, 0)
        ))
        infos.add(PubInfo(
            "topic2",
            time_msg(18, 0),
            time_msg(19, 0),
            time_msg(20, 0)
        ))
        infos.add(PubInfo(
            "topic1",
            time_msg(28, 0),
            time_msg(29, 0),
            time_msg(30, 0)
        ))

        self.assertEqual(len(infos.topic_vs_pubinfos.keys()), 2)
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 2)
        self.assertEqual(len(infos.topic_vs_pubinfos["topic2"].keys()), 1)

        infos.erase_until(time_msg(9, 999))
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 2)
        self.assertEqual(len(infos.topic_vs_pubinfos["topic2"].keys()), 1)

        # boundary condition - not deleted
        infos.erase_until(time_msg(10, 0))
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 2)
        self.assertEqual(len(infos.topic_vs_pubinfos["topic2"].keys()), 1)

        infos.erase_until(time_msg(10, 1))
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 1)
        self.assertTrue("30.000000000" in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertEqual(len(infos.topic_vs_pubinfos["topic2"].keys()), 1)

        # topic2 at t=30.0 is deleted, but key is preserved
        infos.erase_until(time_msg(20, 1))
        self.assertEqual(len(infos.topic_vs_pubinfos.keys()), 2)
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 1)
        self.assertTrue("30.000000000" in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertEqual(len(infos.topic_vs_pubinfos["topic2"]), 0)

    def test_erase_until_many(self):
        infos = PubInfos()
        for i in range(0, 10):
            infos.add(PubInfo(
                "topic1",
                time_msg(i, 0),
                time_msg(i, 0),
                time_msg(i, 0)
            ))

        infos.erase_until(time_msg(3, 1))
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 6)
        self.assertTrue("0.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("1.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("2.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("3.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())

        infos.erase_until(time_msg(7, 1))
        self.assertEqual(len(infos.topic_vs_pubinfos["topic1"].keys()), 2)
        self.assertTrue("4.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("5.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("6.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())
        self.assertTrue("7.000000000" not in
                        infos.topic_vs_pubinfos["topic1"].keys())


if __name__ == '__main__':
    unittest.main()
