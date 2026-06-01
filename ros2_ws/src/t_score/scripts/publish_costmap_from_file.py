#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
import time

import numpy as np

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def load_costmap(path):
    snapshot = np.load(Path(path).expanduser(), allow_pickle=False)
    data = np.asarray(snapshot["data"], dtype=np.int16)
    metadata = json.loads(str(snapshot["metadata"]))
    return data, metadata


def make_message(data, metadata, topic_override=None):
    msg = OccupancyGrid()
    msg.header.frame_id = metadata["frame_id"]
    msg.header.stamp.sec = int(metadata["stamp"]["sec"])
    msg.header.stamp.nanosec = int(metadata["stamp"]["nanosec"])
    msg.info.resolution = float(metadata["resolution"])
    msg.info.width = int(metadata["width"])
    msg.info.height = int(metadata["height"])

    origin = metadata["origin"]
    msg.info.origin.position.x = float(origin["position"]["x"])
    msg.info.origin.position.y = float(origin["position"]["y"])
    msg.info.origin.position.z = float(origin["position"]["z"])
    msg.info.origin.orientation.x = float(origin["orientation"]["x"])
    msg.info.origin.orientation.y = float(origin["orientation"]["y"])
    msg.info.origin.orientation.z = float(origin["orientation"]["z"])
    msg.info.origin.orientation.w = float(origin["orientation"]["w"])

    expected_shape = (msg.info.height, msg.info.width)
    if data.shape != expected_shape:
        raise RuntimeError(f"Raw costmap shape {data.shape} does not match metadata {expected_shape}")

    msg.data = np.clip(data.reshape(-1), -1, 100).astype(np.int8).tolist()
    topic = topic_override or metadata.get("topic", "/traversability_costmap")
    return msg, topic


def parse_args():
    parser = argparse.ArgumentParser(
        description="Republish a saved t_score raw costmap snapshot as nav_msgs/OccupancyGrid."
    )
    parser.add_argument(
        "input",
        help="Input .npz file produced by costmap_to_image.py.",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="Override output topic. Defaults to the topic stored in the snapshot.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=1.0,
        help="Publish rate in Hz.",
    )
    parser.add_argument(
        "--once",
        action="store_true",
        help="Publish once and exit.",
    )
    parser.add_argument(
        "--update-stamp",
        action="store_true",
        help="Use current ROS time for the message stamp each time it is published.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    data, metadata = load_costmap(args.input)

    rclpy.init()
    node = rclpy.create_node("publish_costmap_from_file")
    msg, topic = make_message(data, metadata, args.topic)

    qos = QoSProfile(depth=1)
    qos.reliability = ReliabilityPolicy.RELIABLE
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    publisher = node.create_publisher(OccupancyGrid, topic, qos)

    node.get_logger().info(
        f"Publishing saved costmap {msg.info.width}x{msg.info.height} "
        f"at {msg.info.resolution:.3f} m/cell on {topic}"
    )

    period = 1.0 / max(args.rate, 0.1)
    while rclpy.ok():
        if args.update_stamp:
            msg.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.01)
        if args.once:
            break
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
