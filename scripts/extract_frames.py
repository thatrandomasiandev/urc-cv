#!/usr/bin/env python3
"""Extract JPEG frames from a ROS2 bag for manual labeling in Roboflow."""

import argparse
import math
import os
import sys

import cv2
import rclpy
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPIC = '/camera/color/image_raw'


def open_reader(bag_path: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def topic_type_map(reader: rosbag2_py.SequentialReader) -> dict[str, str]:
    return {topic.name: topic.type for topic in reader.get_all_topics_and_types()}


def scan_topic(reader: rosbag2_py.SequentialReader, topic: str) -> tuple[int, int | None, int | None]:
    reader.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    count = 0
    first_ts: int | None = None
    last_ts: int | None = None
    while reader.has_next():
        _, _, timestamp = reader.read_next()
        if first_ts is None:
            first_ts = timestamp
        last_ts = timestamp
        count += 1
    return count, first_ts, last_ts


def estimate_bag_fps(count: int, first_ts: int | None, last_ts: int | None) -> float:
    if count < 2 or first_ts is None or last_ts is None:
        return 30.0
    duration_s = (last_ts - first_ts) / 1e9
    if duration_s <= 0.0:
        return 30.0
    return (count - 1) / duration_s


def extract_frames(bag_path: str, output_dir: str, target_fps: float) -> int:
    reader = open_reader(bag_path)
    types = topic_type_map(reader)
    if IMAGE_TOPIC not in types:
        raise ValueError(f'Topic {IMAGE_TOPIC} not found in bag: {bag_path}')

    msg_type = get_message(types[IMAGE_TOPIC])
    count, first_ts, last_ts = scan_topic(reader, IMAGE_TOPIC)
    if count == 0:
        return 0

    bag_fps = estimate_bag_fps(count, first_ts, last_ts)
    stride = max(1, math.floor(bag_fps / target_fps))

    reader = open_reader(bag_path)
    reader.set_filter(rosbag2_py.StorageFilter(topics=[IMAGE_TOPIC]))
    bridge = CvBridge()
    os.makedirs(output_dir, exist_ok=True)

    frame_idx = 0
    saved = 0
    while reader.has_next():
        _, data, _ = reader.read_next()
        if frame_idx % stride == 0:
            msg = deserialize_message(data, msg_type)
            image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            out_path = os.path.join(output_dir, f'frame_{saved:06d}.jpg')
            cv2.imwrite(out_path, image)
            saved += 1
        frame_idx += 1

    return saved


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Extract JPEG frames from a ROS2 bag for Roboflow labeling.',
    )
    parser.add_argument('bag_path', help='Path to the bag directory')
    parser.add_argument('output_dir', help='Directory for extracted JPEG frames')
    parser.add_argument('--fps', type=float, default=2.0, help='Target extraction rate (default: 2)')
    args = parser.parse_args()

    if args.fps <= 0.0:
        print('Error: --fps must be positive', file=sys.stderr)
        return 1

    rclpy.init()
    try:
        total = extract_frames(args.bag_path, args.output_dir, args.fps)
    except ValueError as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()

    print(f'Total frames extracted: {total}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
