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

from collections import defaultdict

from bokeh.models import CrosshairTool
from bokeh.plotting import figure, show
import numpy as np
import pandas as pd

from tilde_vis.pub_info import (
    PubInfo,
    PubInfos
)
from tilde_vis.pubinfo_traverse import (
    InputSensorStampSolver,
    TopicGraph
)


def time2str(t):
    """Convert builtin_interfaces.msg.Time to string."""
    return f'{t.sec}.{t.nanosec:09d}'


def time2int(t):
    """Convert builtin_interfaces.msg.Time to int."""
    return t.sec * 10**9 + t.nanosec


def _rosbag_iter(rosbag_path):
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py

    storage_option = rosbag2_py.StorageOptions(rosbag_path, 'sqlite3')

    converter_option = rosbag2_py.ConverterOptions('cdr', 'cdr')
    reader = rosbag2_py.SequentialReader()

    reader.open(storage_option, converter_option)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))}

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        yield topic, msg, t


def read_msgs(rosbag_path):
    """Read demo messages from rosbag.

    Returns
    -------
    dictionary such as
    {
       <topic_name>: [<message>]
    }

    """
    msg_topics = defaultdict(list)
    for topic, msg, t in _rosbag_iter(rosbag_path):
        msg_topics[topic].append(msg)
    return msg_topics


def read_pubinfo(raw_msgs):
    pubinfos = PubInfos()
    for i in range(5):
        msgs = raw_msgs[f'/topic{i+1}/info/pub']
        for msg in msgs:
            pubinfos.add(PubInfo.fromMsg(msg))
    return pubinfos


def get_uuid2msg(raw_msgs):
    hashed_msgs = {}
    hash_target_keys = ['/topic1', '/topic2', '/topic3', '/topic4', '/topic5']
    for hash_target_key in hash_target_keys:
        hashed_msgs.update({
            msg.uuid: msg for msg in raw_msgs[hash_target_key]
        })
    return hashed_msgs


def build_latency_table(traces, ds):
    """Calculate latencies.

    Parameters
    ----------
    traces: CARET traces
    ds: list to output

    """
    traces_dict = {trace.uuid: trace for trace in traces}

    from copy import deepcopy

    def search_flow(start_uuid):
        flows = []

        # 後続のトレースを取得
        def get_subsequents(uuid):
            return [trace for trace in traces if uuid in trace.used_uuids]

        def search_local(uuid, flow):
            flow.append(uuid)

            subsequents = get_subsequents(uuid)
            if len(subsequents) == 0:
                flows.append(flow)

            for sub in subsequents:
                search_local(sub.uuid, deepcopy(flow))

        search_local(start_uuid, [])

        return flows

    # start_msgs = msgs_topics['/topic1']
    start_msgs = [trace.uuid for trace in traces
                  if '/topic1' in trace.uuid and trace.trace_type == 'publish']

    uid_flows = []
    for msg in start_msgs:
        uid_flows += search_flow(msg)

    def trace_to_column(trace):
        return f'{trace.node_name}_{trace.trace_type}'

    from collections import defaultdict
    for uid_flow in uid_flows:
        try:
            flow = [traces_dict[uid] for uid in uid_flow]
        except KeyError:
            flow = []
        d = defaultdict()

        d.update({trace_to_column(trace): trace.steady_t for trace in flow})

        ds.append(d)

    return ds

############################
# visualize tilde
############################


class Trace(object):

    def __init__(self, node_name, uuid, stamp, is_publish, uuids):
        self.node_name = node_name  # "<topic>"
        self.uuid = uuid  # "<topic>_<stamp>"
        self.steady_t = stamp
        self.trace_type = 'publish' if is_publish else 'callback_start'
        self.used_uuids = uuids

    def __repr__(self):
        return (
            '<repr> '
            f'node_name={self.node_name} uuid={self.uuid} '
            f'steady_t={self.steady_t} trace_type={self.trace_type} '
            f'used_uuids={self.used_uuids}')


def vis_tilde(pub_infos):
    tgt_topic = '/topic5'
    topic5_stamps = pub_infos.stamps(tgt_topic)

    graph = TopicGraph(pub_infos)
    solver = InputSensorStampSolver(graph)

    ds = []
    for stamp in topic5_stamps:
        tilde_path = solver.solve2(pub_infos, tgt_topic, stamp)

        traces = []

        def update_traces(node):
            topic = node.name
            for d in node.data:
                stamp = time2int(d.out_info.stamp)
                pubtime = time2int(d.out_info.pubsub_stamp)
                uuid = f'{topic}_{stamp}'
                uuids = []
                for in_topic, infos in d.in_infos.items():
                    for i in infos:
                        i_stamp = time2int(i.stamp)
                        subtime = time2int(i.pubsub_stamp)
                        i_uuid = f'{in_topic}_{i_stamp}'
                        uuids.append(i_uuid)

                        # append callback_start
                        trace = Trace(in_topic, i_uuid, subtime, False, [])
                        traces.append(trace)
                # append publish
                traces.append(Trace(topic, uuid, pubtime, True, uuids))

        tilde_path.apply(update_traces)

        ds = build_latency_table(traces, ds)

    return ds


def plot_latency_table(df):
    # dfからフローの可視化
    from tqdm import tqdm
    from bokeh.palettes import Bokeh8

    def get_color(i):
        cmap = Bokeh8
        i = i % (len(cmap))
        return cmap[i]

    fig = figure(
        x_axis_label='Time [s]',
        y_axis_label='',
        plot_width=1000,
        plot_height=400,
        active_scroll='wheel_zoom',
    )
    fig.add_tools(CrosshairTool(line_alpha=0.4))

    for i, row in tqdm(df.iterrows()):
        row_ = row.dropna()

        y = np.array(list(range(len(row_)))) * -1
        x = list(row_.values)

        fig.line(x, y, line_color=get_color(i), line_alpha=0.4)
    show(fig)


def main():
    bagfile = 'rosbag2_2022_02_16-18_12_46'
    raw_msgs = read_msgs(bagfile)
    pub_infos = read_pubinfo(raw_msgs)
    ds = vis_tilde(pub_infos)
    df = pd.DataFrame.from_dict(ds)

    plot_latency_table(df)


if __name__ == '__main__':
    main()
