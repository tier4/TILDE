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

from collections import deque
import unittest

from tilde_vis.data_as_tree import TreeNode


def dict2tree(dict_):
    """
    Convert dictionary to TreeNode.

    Parameters
    ----------
    dict_: dictionary. If value is a dictionary,
           then sub tree is created.
           Otherwise, value are stored on data.

    Return
    ------
    TreeNode starts from "root".

    """
    Q = deque()
    root = TreeNode('root')
    Q.append((root, dict_))

    while len(Q) > 0:
        current_node, sub_dict = Q.pop()

        for k, v in sub_dict.items():
            child = current_node.get_child(k)
            if isinstance(v, dict):
                Q.append((child, v))
            else:
                child.add_data(v)

    return root


def get_complex_tree():
    """
    Get complex tree.

    Return:
    ------
    see the following `sample` variable.
    The name of the top node is "root".
    So, call `ret.get_chile("p1")` and so on to access nodes.

    """
    sample = {
        'p1': {
            'c1': 'c11',
            'c2': {
                'c21': {
                    'c211': 'c2111',
                    },
                'c22': 'c221',
                }
            }
        }

    # (TreeNode, sub dictionary)
    Q = deque()
    root = TreeNode('root')
    Q.append((root, sample))

    while len(Q) > 0:
        current_node, sub_dict = Q.pop()

        if not isinstance(sub_dict, dict):
            current_node.get_child(sub_dict)
            continue

        for k in sub_dict.keys():
            child_node = current_node.get_child(k)
            Q.append((child_node, sub_dict[k]))

    return root


class TestTreeNode(unittest.TestCase):

    def test_construct(self):
        node = TreeNode('foo')
        self.assertTrue(node is not None)

    def test_construct_complex(self):
        root = get_complex_tree()

        # check tree structure
        self.assertEqual(len(root.children), 1)
        p1 = root.name2child['p1']
        self.assertEqual(len(p1.children), 2)
        c1 = p1.name2child['c1']
        self.assertEqual(len(c1.children), 1)
        c2 = p1.name2child['c2']
        self.assertEqual(len(c2.children), 2)

    def test_apply(self):
        root = get_complex_tree()
        ret = root.apply(lambda n: n.name)

        self.assertEqual(len(ret), 10)
        self.assertEqual(ret,
                         ['root',
                          'p1',
                          'c1',
                          'c11',
                          'c2',
                          'c21',
                          'c211',
                          'c2111',
                          'c22',
                          'c221'])

    def test_apply_with_depth(self):
        root = get_complex_tree()
        ret = root.apply_with_depth(lambda n, depth: f'{n.name}_{depth}')

        self.assertEqual(len(ret), 10)
        self.assertEqual(ret,
                         ['root_0',
                          'p1_1',
                          'c1_2',
                          'c11_3',
                          'c2_2',
                          'c21_3',
                          'c211_4',
                          'c2111_5',
                          'c22_3',
                          'c221_4'])

    def test_merge(self):
        lhs_dict = {
            'c1': {
                'c11': 1,
                'c13': 2,
                },
            'c2': {
                'c21': 3,
                },
            }
        rhs_dict = {
            'c1': {
                'c11': 10,
                'c12': {
                    'c121': 11,
                    },
                },
            'c2': 12,
            }
        lhs = dict2tree(lhs_dict)
        rhs = dict2tree(rhs_dict)

        def to_s(tree_node):
            name = tree_node.name
            data = tree_node.data
            return f'{name}: {data}'

        lhs.apply(lambda x: print(to_s(x)))
        lhs.merge(rhs)
        lhs.apply(lambda x: print(to_s(x)))

        self.assertEqual(len(lhs.children), 2)
        c1 = lhs.get_child('c1')
        self.assertEqual(len(c1.children), 3)
        self.assertEqual(c1.get_child('c11').data, [1, 10])
        self.assertEqual(c1.get_child('c12').get_child('c121').data, [11])
        self.assertEqual(c1.get_child('c13').data, [2])

        c2 = lhs.get_child('c2')
        self.assertEqual(c2.data, [12])
        self.assertEqual(c2.get_child('c21').data, [3])


if __name__ == '__main__':
    unittest.main()
