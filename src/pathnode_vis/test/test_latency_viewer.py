#!/usr/bin/python3

import unittest

from pathnode_vis.data_as_tree import TreeNode
from pathnode_vis.latency_viewer import (
    calc_stat
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


if __name__ == '__main__':
    unittest.main()
