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

import argparse
import pickle

from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message

from tilde_vis.pub_info import PubInfo, PubInfos


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def run(args):
    bag_path = args.bag_path

    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))}

    # topic => list of record
    out_per_topic = PubInfos()

    skip_topic_vs_count = {}

    cnt = 0
    while reader.has_next():
        if 0 <= args.cnt and args.cnt < cnt:
            break

        (topic, data, t) = reader.read_next()
        # TODO: need more accurate check
        if '/info/pub' not in topic:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        pub_info = PubInfo.fromMsg(msg)
        out_per_topic.add(pub_info)

        if 0 < cnt and cnt % 1000 == 0:
            print(cnt)
        cnt += 1

    pickle.dump(out_per_topic, open('topic_infos.pkl', 'wb'),
                protocol=pickle.HIGHEST_PROTOCOL)

    for topic, count in skip_topic_vs_count.items():
        print(f'skipped {topic} {count} times')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path')
    parser.add_argument(
        '--cnt', type=int, default=-1,
        help='number of messages to dump (whole */info/pub, not per topic)')
    args = parser.parse_args()

    run(args)


if __name__ == '__main__':
    main()
