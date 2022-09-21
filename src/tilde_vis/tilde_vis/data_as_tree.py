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

"""Tree like data structure for MessageTrackingTag traversal."""


class TreeNode(object):
    """
    Node of DataAsTree.

    This hold data list and children.

    """

    def __init__(self, name):
        """Initialize tree structure."""
        self.name = name
        self.children = []
        self.name2child = {}
        self.data = []
        self.value = None

    def num_children(self):
        """Get the number of children."""
        return len(self.children)

    def get_child(self, child_name):
        """
        Get child.

        If not exists, then append.

        """
        children = self.children
        name2child = self.name2child

        if child_name in name2child.keys():
            return name2child[child_name]

        c = TreeNode(child_name)
        children.append(c)
        children.sort(key=lambda x: x.name)
        name2child[child_name] = c

        return c

    def add_data(self, d):
        """
        Add data.

        :param d: any value or object

        """
        self.data.append(d)

    def apply(self, fn):
        """
        Apply fn recursively.

        Internally, pre-order DFS is used.

        :param fn: callable, which takes TreeNode
        :return list of fn return

        """
        children = self.children
        ret = []

        ret.append(fn(self))
        for c in children:
            ret.extend(c.apply(fn))

        return ret

    def apply_with_depth(self, fn, depth=0):
        """
        Apply with depth option.

        Internally, pre-order DFS is used.

        :param fn: callback such as fn(tree_node_obj, depth)
        :return list of fn return

        """
        children = self.children
        ret = []

        ret.append(fn(self, depth))
        for c in children:
            ret.extend(c.apply_with_depth(fn, depth + 1))

        return ret

    def merge(self, rhs):
        """
        Merge data of another tree.

        If self does not have some keys which rhs has,
        then new nodes are added.

        :param rhs: another TreeNode

        """
        def _merge(lhs, rhs):
            """
            Merge data.

            lhs: TreeNode which is updated
            rhs: TreeNode which is const
            """
            lhs.data.extend(rhs.data)

            for rhs_c in rhs.children:
                lhs_c = lhs.get_child(rhs_c.name)
                _merge(lhs_c, rhs_c)

        _merge(self, rhs)
