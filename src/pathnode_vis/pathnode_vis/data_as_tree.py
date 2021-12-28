class TreeNode(object):
    """Node of DataAsTree.
    This hold data list and children.
    """
    def __init__(self, name):
        self.name = name
        self.children = []
        self.name2child = {}
        self.data = []
        self.value = None

    def get_child(self, child_name):
        """ Get child.
        If not exists, then append.
        """
        children = self.children
        name2child = self.name2child

        if child_name in name2child.keys():
            return name2child[child_name]

        c = TreeNode(child_name)
        children.append(c)
        name2child[child_name] = c

        return c

    def add_data(self, d):
        self.data.append(d)

    def apply(self, fn):
        """Apply fn recursively.
        Internally, preordering DFS is used.

        Parameters
        ----------
        fn: callable, which takes TreeNode

        Return
        ------
        list of fn return
        """
        children = self.children
        ret = []

        ret.append(fn(self))
        for c in children:
            ret.extend(c.apply(fn))

        return ret

    def apply_with_depth(self, fn, depth=0):
        """Apply with depth option
        Internally, preordering DFS is used.

        Parameters
        ----------
        fn: callback such as fn(tree_node_obj, depth)

        Return
        ------
        list of fn return
        """
        children = self.children
        ret = []

        ret.append(fn(self, depth))
        for c in children:
            ret.extend(c.apply_with_depth(fn, depth + 1))

        return ret

    def merge(self, rhs):
        """Merge data of another tree.
        If self does not have some keys which rhs has,
        then new nodes are added.

        Parameters
        ----------
        rhs: another TreeNode
        """
        def _merge(lhs, rhs):
            """
            lhs: self sub node
            rhs: rhs sub node
            """
            lhs.data.extend(rhs.data)

            for rhs_c in rhs.children:
                lhs_c = lhs.get_child(rhs_c.name)
                _merge(lhs_c, rhs_c)

        _merge(self, rhs)
