#!/usr/bin/env python3

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

import rclpy
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from nav_msgs.msg import OccupancyGrid


class CostmapImageSaver:
    def __init__(self, args):
        self.args = args
        self.node = rclpy.create_node("costmap_to_image")
        self.received = False
        self.last_msg = None
        self.last_msg_time = None
        self.subscription = self.node.create_subscription(
            OccupancyGrid,
            args.topic,
            self.callback,
            10,
        )

    def callback(self, msg):
        self.last_msg = msg
        self.last_msg_time = self.node.get_clock().now().nanoseconds
        if self.args.save_policy == "first":
            self.save_message(msg)
            self.received = True

    def save_latest_if_idle(self):
        if self.args.save_policy != "latest" or self.last_msg is None:
            return

        now = self.node.get_clock().now().nanoseconds
        idle_ns = int(self.args.idle_timeout * 1e9)
        if now - self.last_msg_time >= idle_ns:
            self.save_message(self.last_msg)
            self.received = True

    def save_message(self, msg):
        output_path = Path(self.args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        data = np.asarray(msg.data, dtype=np.int16).reshape(
            (msg.info.height, msg.info.width)
        )

        if self.args.palette == "gray":
            image = self.make_gray_image(data)
        else:
            image = self.make_risk_image(data)

        if self.args.flip_y:
            image = np.flipud(image)

        pil_image = Image.fromarray(image, mode="L" if image.ndim == 2 else "RGB")
        if output_path.suffix.lower() in [".jpg", ".jpeg"]:
            pil_image = pil_image.convert("RGB")
        pil_image.save(output_path)

        metadata = {
            "topic": self.args.topic,
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "position": {
                    "x": msg.info.origin.position.x,
                    "y": msg.info.origin.position.y,
                    "z": msg.info.origin.position.z,
                },
                "orientation": {
                    "x": msg.info.origin.orientation.x,
                    "y": msg.info.origin.orientation.y,
                    "z": msg.info.origin.orientation.z,
                    "w": msg.info.origin.orientation.w,
                },
            },
        }
        metadata_path = Path(self.args.metadata_output)
        metadata_path.parent.mkdir(parents=True, exist_ok=True)
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

        self.node.get_logger().info(
            f"Saved {msg.info.width}x{msg.info.height} costmap to {output_path}"
        )
        self.node.get_logger().info(f"Saved metadata to {metadata_path}")
        self.received = True

    def make_gray_image(self, data):
        image = np.full(data.shape, self.args.unknown_gray, dtype=np.uint8)
        known = data >= 0
        image[known] = 255 - np.clip(data[known], 0, 100) * 255 // 100
        return image

    def make_risk_image(self, data):
        image = np.full((*data.shape, 3), self.args.unknown_gray, dtype=np.uint8)
        known = data >= 0
        lethal = data >= self.args.lethal_threshold
        risk = known & ~lethal

        cost = np.clip(data[risk], 0, self.args.lethal_threshold - 1).astype(np.float32)
        denom = max(1, self.args.lethal_threshold - 1)
        t = cost / float(denom)

        # Blue -> cyan -> yellow -> red, while reserving black for lethal cells.
        image[risk, 0] = np.clip(255.0 * np.maximum(0.0, 2.0 * t - 0.5), 0, 255).astype(np.uint8)
        image[risk, 1] = np.clip(255.0 * (1.0 - np.abs(2.0 * t - 1.0)), 0, 255).astype(np.uint8)
        image[risk, 2] = np.clip(255.0 * np.maximum(0.0, 1.0 - 2.0 * t), 0, 255).astype(np.uint8)
        image[lethal] = (0, 0, 0)
        return image


def default_param_path():
    try:
        return Path(get_package_share_directory("t_score")) / "config" / "params.json"
    except PackageNotFoundError:
        return Path(__file__).resolve().parents[1] / "config" / "params.json"


def load_config(param_path):
    if param_path is None:
        return {}

    path = Path(param_path).expanduser()
    if not path.exists():
        return {}

    return json.loads(path.read_text())


def config_value(args, config, arg_name, config_name, fallback):
    value = getattr(args, arg_name)
    if value is not None:
        return value
    return config.get(config_name, fallback)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Save one nav_msgs/OccupancyGrid message as an image."
    )
    parser.add_argument(
        "--param-path",
        default=str(default_param_path()),
        help="Path to t_score params.json. Values in this file provide exporter defaults.",
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="OccupancyGrid topic to snapshot.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output image path. Supported by Pillow, e.g. .jpg or .png.",
    )
    parser.add_argument(
        "--metadata-output",
        default=None,
        help="Output metadata JSON path.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Seconds to wait for one costmap message.",
    )
    parser.add_argument(
        "--unknown-gray",
        type=int,
        default=None,
        help="Grayscale value for unknown cells.",
    )
    parser.add_argument(
        "--palette",
        choices=["gray", "risk"],
        default=None,
        help="Color palette: gray or risk. Risk uses blue=safe, red=danger, gray=unknown, black=lethal.",
    )
    parser.add_argument(
        "--lethal-threshold",
        type=int,
        default=None,
        help="Cells at or above this cost are rendered black in risk mode.",
    )
    parser.add_argument(
        "--save-policy",
        choices=["first", "latest"],
        default=None,
        help="Save the first message, or keep the latest message and save after the stream goes idle.",
    )
    parser.add_argument(
        "--idle-timeout",
        type=float,
        default=None,
        help="Seconds without a new message before saving in latest mode.",
    )
    parser.add_argument(
        "--flip-y",
        dest="flip_y",
        action="store_true",
        default=None,
        help="Flip ROS OccupancyGrid row order so image y points up.",
    )
    parser.add_argument(
        "--no-flip-y",
        dest="flip_y",
        action="store_false",
        help="Keep ROS OccupancyGrid row order instead of making image y point up.",
    )
    args = parser.parse_args()
    config = load_config(args.param_path)

    args.topic = config_value(
        args, config, "topic", "costmap_image_topic", config.get("traversability_topic_global", "/traversability_costmap")
    )
    args.output = config_value(
        args, config, "output", "costmap_image_output_path", "/tmp/traversability_costmap.png"
    )
    args.metadata_output = config_value(
        args,
        config,
        "metadata_output",
        "costmap_metadata_output_path",
        str(Path(args.output).with_suffix(Path(args.output).suffix + ".json")),
    )
    args.timeout = float(config_value(args, config, "timeout", "costmap_image_timeout", 30.0))
    args.unknown_gray = int(config_value(args, config, "unknown_gray", "costmap_image_unknown_gray", 127))
    args.palette = config_value(args, config, "palette", "costmap_image_palette", "gray")
    args.lethal_threshold = int(config_value(args, config, "lethal_threshold", "costmap_image_lethal_threshold", 100))
    args.save_policy = config_value(args, config, "save_policy", "costmap_image_save_policy", "first")
    args.idle_timeout = float(config_value(args, config, "idle_timeout", "costmap_image_idle_timeout", 3.0))
    args.flip_y = bool(config_value(args, config, "flip_y", "costmap_image_flip_y", True))

    if args.palette not in ["gray", "risk"]:
        raise SystemExit(f"Invalid costmap_image_palette '{args.palette}'. Expected 'gray' or 'risk'.")
    if args.save_policy not in ["first", "latest"]:
        raise SystemExit(f"Invalid costmap_image_save_policy '{args.save_policy}'. Expected 'first' or 'latest'.")

    return args


def main():
    args = parse_args()
    rclpy.init()
    saver = CostmapImageSaver(args)
    deadline = saver.node.get_clock().now().nanoseconds + int(args.timeout * 1e9)

    while rclpy.ok() and not saver.received:
        rclpy.spin_once(saver.node, timeout_sec=0.1)
        saver.save_latest_if_idle()
        if saver.node.get_clock().now().nanoseconds > deadline:
            saver.node.get_logger().error(
                f"Timed out waiting for OccupancyGrid on {args.topic}"
            )
            rclpy.shutdown()
            raise SystemExit(1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
