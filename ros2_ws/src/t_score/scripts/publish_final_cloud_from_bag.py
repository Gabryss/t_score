#!/usr/bin/env python3

import argparse
from pathlib import Path
import time

import rclpy
import rosbag2_py
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import yaml
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


def normalize_namespace(namespace):
    if not namespace or namespace == "/":
        return ""
    namespace = namespace.strip()
    if not namespace.startswith("/"):
        namespace = "/" + namespace
    return namespace.rstrip("/")


def resolve_topic_name(topic, namespace):
    namespace = normalize_namespace(namespace)
    if not topic or not namespace:
        return topic
    if topic.startswith(namespace + "/"):
        return topic
    if topic.startswith("/"):
        return namespace + topic
    return namespace + "/" + topic


def bag_time_window(bag_uri):
    metadata_path = Path(bag_uri) / "metadata.yaml"
    metadata = yaml.safe_load(metadata_path.read_text())
    info = metadata["rosbag2_bagfile_information"]
    start = info["starting_time"]["nanoseconds_since_epoch"]
    duration = info["duration"]["nanoseconds"]
    return start, start + duration


def read_final_cloud_time(bag_uri, cloud_topic, seek_back_sec):
    start, end = bag_time_window(bag_uri)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_uri, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[cloud_topic]))
    reader.seek(max(start, end - int(seek_back_sec * 1e9)))

    final_time = None
    final_cloud = None
    while reader.has_next():
        _, data, stamp = reader.read_next()
        final_time = stamp
        final_cloud = deserialize_message(data, PointCloud2)

    if final_cloud is None:
        raise RuntimeError(f"No PointCloud2 messages found on {cloud_topic} near the end of the bag")
    return final_time, final_cloud


def make_identity_robot_tf(map_frame, robot_frame):
    transform = TransformStamped()
    transform.header.frame_id = map_frame
    transform.child_frame_id = robot_frame
    transform.transform.rotation.w = 1.0
    return TFMessage(transforms=[transform])


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish the final cloud map from a rosbag plus its latest TF."
    )
    parser.add_argument("bag", help="Rosbag directory.")
    parser.add_argument("--cloud-topic", default="/rtabmap/cloud_map")
    parser.add_argument(
        "--publish-cloud-topic",
        default=None,
        help="Output cloud topic. Defaults to --cloud-topic after namespace prefixing.",
    )
    parser.add_argument("--namespace", default="", help="Optional robot namespace for published topics.")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--robot-frame", default="base_link")
    parser.add_argument("--seek-back-sec", type=float, default=600.0)
    parser.add_argument("--publish-seconds", type=float, default=8.0)
    parser.add_argument("--rate", type=float, default=5.0)
    return parser.parse_args()


def main():
    args = parse_args()
    final_time, final_cloud = read_final_cloud_time(args.bag, args.cloud_topic, args.seek_back_sec)
    publish_cloud_topic = resolve_topic_name(args.publish_cloud_topic or args.cloud_topic, args.namespace)
    tf_topic = resolve_topic_name("/tf", args.namespace)
    tf_static_topic = resolve_topic_name("/tf_static", args.namespace)
    tf_msg = make_identity_robot_tf(args.map_frame, args.robot_frame)
    tf_static_msg = make_identity_robot_tf(args.map_frame, args.robot_frame)

    rclpy.init()
    node = rclpy.create_node("publish_final_cloud_from_bag")

    cloud_qos = QoSProfile(depth=1)
    cloud_qos.reliability = ReliabilityPolicy.RELIABLE
    cloud_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    tf_qos = QoSProfile(depth=100)
    tf_static_qos = QoSProfile(depth=1)
    tf_static_qos.reliability = ReliabilityPolicy.RELIABLE
    tf_static_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

    cloud_pub = node.create_publisher(PointCloud2, publish_cloud_topic, cloud_qos)
    tf_pub = node.create_publisher(TFMessage, tf_topic, tf_qos)
    tf_static_pub = node.create_publisher(TFMessage, tf_static_topic, tf_static_qos)

    node.get_logger().info(
        f"Publishing final {args.cloud_topic} as {publish_cloud_topic} at bag timestamp {final_time} "
        f"with {len(final_cloud.data)} serialized cloud bytes, "
        f"{len(tf_msg.transforms)} dynamic TF transforms"
    )

    period = 1.0 / max(args.rate, 0.1)
    deadline = time.monotonic() + args.publish_seconds
    while rclpy.ok() and time.monotonic() < deadline:
        tf_static_pub.publish(tf_static_msg)
        tf_pub.publish(tf_msg)
        cloud_pub.publish(final_cloud)
        rclpy.spin_once(node, timeout_sec=0.01)
        time.sleep(period)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
