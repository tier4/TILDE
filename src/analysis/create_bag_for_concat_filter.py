#!/usr/bin/python3

import argparse
import os
import shutil

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

serialization_format = 'cdr'

def main(args):
    inbag_path = args.inbag
    outbag_path = args.outdir

    if os.path.exists(outbag_path):
        shutil.rmtree(outbag_path)

    in_storage_options = rosbag2_py.StorageOptions(uri=inbag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    reader = rosbag2_py.SequentialReader()
    reader.open(in_storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    topics = [
        "/sensing/lidar/left/outlier_filtered/pointcloud",
        "/sensing/lidar/right/outlier_filtered/pointcloud",
        "/sensing/lidar/top/outlier_filtered/pointcloud",
        "/vehicle/status/twist",
        "/clock"
    ]

    # topics to save.
    # design:
    #   
    # We save all of twist and clock.
    topics_to_save = {
        "/sensing/lidar/left/outlier_filtered/pointcloud": [
            500, 505
        ],
        "/sensing/lidar/right/outlier_filtered/pointcloud": [
            500, 504, 505
        ],
        "/sensing/lidar/top/outlier_filtered/pointcloud": [
            500,
        ]
    }

    storage_filter = rosbag2_py.StorageFilter(topics=topics)
    reader.set_filter(storage_filter)

    out_storage_options = rosbag2_py.StorageOptions(outbag_path, "sqlite3")
    writer = rosbag2_py.SequentialWriter()
    writer.open(out_storage_options, converter_options)

    for meta in reader.get_all_topics_and_types():
        writer.create_topic(meta)

    counts = {k: 0 for k in topics}
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        counts[topic] += 1

        if "clock" in topic:
            writer.write(topic, data, t)
        elif "twist" in topic:
            writer.write(topic, data, t)
        elif counts[topic] in topics_to_save[topic]:
            print(f"{topic}: {msg.header.stamp}")
            writer.write(topic, data, t)

        if counts["/sensing/lidar/left/outlier_filtered/pointcloud"] == 1000:
            break

    del writer
    rosbag2_py.Reindexer().reindex(out_storage_options)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("inbag", help="input bagfile")
    parser.add_argument("outdir", help="output directory")
    args = parser.parse_args()

    main(args)
