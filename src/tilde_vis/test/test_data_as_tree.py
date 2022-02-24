#!/usr/bin/python3

import unittest
from collections import deque

from tilde_vis.data_as_tree import TreeNode


def dict2tree(adict):
    """Convert dictionary to TreeNode.

    Parameters
    ----------
    adict: dictionary. If value is a dictionary,
           then sub tree is created.
           Otherwise, value are stored on data.

    Return
    ------
    TreeNode starts from "root"
    """
    Q = deque()
    root = TreeNode("root")
    Q.append((root, adict))

    while len(Q) > 0:
        cnode, subdict = Q.pop()

        for k, v in subdict.items():
            child = cnode.get_child(k)
            if isinstance(v, dict):
                Q.append((child, v))
            else:
                child.add_data(v)

    return root


def get_complex_tree():
    """Get complex tree.

    Return
    ------
    see the following `sample` variable.
    The name of the top node is "root".
    So, call `ret.get_chile("p1")` and so on to access nodes.
    """
    sample = {
        "p1": {
            "c1": "c11",
            "c2": {
                "c21": {
                    "c211": "c2111",
                    },
                "c22": "c221",
                }
            }
        }

    # (TreeNode, sub dictionary)
    Q = deque()
    root = TreeNode("root")
    Q.append((root, sample))

    while len(Q) > 0:
        cnode, subdict = Q.pop()

        if not isinstance(subdict, dict):
            cnode.get_child(subdict)
            continue

        for k in subdict.keys():
            nnode = cnode.get_child(k)
            Q.append((nnode, subdict[k]))

    return root


class TestTreeNode(unittest.TestCase):
    def test_construct(self):
        node = TreeNode("foo")
        self.assertTrue(node is not None)

    def test_construct_complex(self):
        root = get_complex_tree()

        # check tree structure
        self.assertEqual(len(root.children), 1)
        p1 = root.name2child["p1"]
        self.assertEqual(len(p1.children), 2)
        c1 = p1.name2child["c1"]
        self.assertEqual(len(c1.children), 1)
        c2 = p1.name2child["c2"]
        self.assertEqual(len(c2.children), 2)

    def test_apply(self):
        root = get_complex_tree()
        ret = root.apply(lambda n: n.name)

        self.assertEqual(len(ret), 10)
        self.assertEqual(ret,
                         ["root",
                          "p1",
                          "c1",
                          "c11",
                          "c2",
                          "c21",
                          "c211",
                          "c2111",
                          "c22",
                          "c221"])

    def test_apply_with_depth(self):
        root = get_complex_tree()
        ret = root.apply_with_depth(lambda n, depth: f"{n.name}_{depth}")

        self.assertEqual(len(ret), 10)
        self.assertEqual(ret,
                         ["root_0",
                          "p1_1",
                          "c1_2",
                          "c11_3",
                          "c2_2",
                          "c21_3",
                          "c211_4",
                          "c2111_5",
                          "c22_3",
                          "c221_4"])

    def test_merge(self):
        lhs_dict = {
            "c1": {
                "c11": 1,
                "c13": 2,
                },
            "c2": {
                "c21": 3,
                },
            }
        rhs_dict = {
            "c1": {
                "c11": 10,
                "c12": {
                    "c121": 11,
                    },
                },
            "c2": 12,
            }
        lhs = dict2tree(lhs_dict)
        rhs = dict2tree(rhs_dict)

        def to_s(tree_node):
            name = tree_node.name
            data = tree_node.data
            return f"{name}: {data}"

        lhs.apply(lambda x: print(to_s(x)))
        lhs.merge(rhs)
        lhs.apply(lambda x: print(to_s(x)))

        self.assertEqual(len(lhs.children), 2)
        c1 = lhs.get_child("c1")
        self.assertEqual(len(c1.children), 3)
        self.assertEqual(c1.get_child("c11").data, [1, 10])
        self.assertEqual(c1.get_child("c12").get_child("c121").data, [11])
        self.assertEqual(c1.get_child("c13").data, [2])

        c2 = lhs.get_child("c2")
        self.assertEqual(c2.data, [12])
        self.assertEqual(c2.get_child("c21").data, [3])


if __name__ == '__main__':
    unittest.main()
