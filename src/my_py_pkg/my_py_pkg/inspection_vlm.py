#!/usr/bin/env python3
"""ROS 2 inspection VLM node.

Subscribes: /inspection/capture_done (std_msgs/String, JSON)
Publishes:  /inspection/result (std_msgs/String, JSON)
Writes:     ~/ros2_ws/data/inspection_results.jsonl

Works separately from waypoint_controller; an image-save acknowledgment and a
VLM inspection result are deliberately different events.
"""

import base64
import json
import os
import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from openai import OpenAI


TARGETS = {
    "A": "the BLUE equipment cabinet (usually on the RIGHT in the original scene)",
    "B": "the ORANGE/BROWN equipment cabinet (usually on the LEFT in the original scene)",
}
COLORS = {"red", "green", "yellow", "other", "unknown"}


class InspectionVLM(Node):
    def __init__(self):
        super().__init__("inspection_vlm")
        self.model = os.getenv("OPENAI_VLM_MODEL", "gpt-4o-mini")
        self.allowed_dir = (Path.home() / "ros2_ws/data/captures").resolve()
        self.result_file = Path.home() / "ros2_ws/data/inspection_results.jsonl"
        self.result_file.parent.mkdir(parents=True, exist_ok=True)

        self.pending = queue.Queue()
        self.executor_pool = ThreadPoolExecutor(max_workers=1)
        self.seen = set()
        self.seen_lock = threading.Lock()
        self.future = None
        self.publish_result = self.create_publisher(String, "/inspection/result", 10)
        self.create_subscription(String, "/inspection/capture_done", self.capture_callback, 10)
        self.timer = self.create_timer(0.1, self.drain_results)
        self.get_logger().info(f"Inspection VLM started: model={self.model}")
        self.get_logger().info("Waiting for /inspection/capture_done ...")

    def capture_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict):
                raise ValueError("Expected JSON object")
            equipment = data.get("equipment")
            if equipment not in TARGETS:
                raise ValueError("equipment must be A or B")
            if data.get("success") is not True:
                self.get_logger().warning(f"Skipping unsuccessful capture for {equipment}")
                return
            raw_path = data.get("image_path")
            if not isinstance(raw_path, str) or not raw_path:
                raise ValueError("Missing image_path")
            image_path = Path(raw_path).resolve(strict=True)
            if not image_path.is_relative_to(self.allowed_dir):
                raise ValueError("Image is outside capture directory")
            if image_path.suffix.lower() != ".png":
                raise ValueError("Only PNG capture files are supported")
            if not image_path.name.startswith(f"equipment_{equipment}_"):
                raise ValueError("Image filename does not match equipment")
            if image_path.stat().st_size > 10_000_000:
                raise ValueError("Image too large")

            with self.seen_lock:
                if str(image_path) in self.seen:
                    self.get_logger().info(f"Skipping duplicate: {image_path.name}")
                    return
                self.seen.add(str(image_path))

            self.get_logger().info(f"Queued VLM inspection for {equipment}: {image_path.name}")
            self.executor_pool.submit(self.inspect_and_queue, equipment, image_path)
        except (ValueError, OSError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f"Invalid capture_done: {exc}")

    def inspect_and_queue(self, equipment, image_path):
        started = time.monotonic()
        try:
            encoded = base64.b64encode(image_path.read_bytes()).decode("ascii")
            client = OpenAI(timeout=30.0, max_retries=0)
            prompt = (
                "Inspect a rendered camera image from a robot simulation. "
                f"The target is equipment {equipment}: {TARGETS[equipment]}. "
                "The scene may contain BOTH equipment cabinets. Find the target by BODY COLOR, "
                "then inspect the SMALL SPHERICAL INDICATOR LIGHT ON TOP OF THAT CABINET ONLY. "
                "Do not use an indicator on another cabinet. "
                "Do not assume the light color from the equipment name or body color. "
                "If the target or its indicator is not clearly visible, return unknown. "
                "Return ONLY valid JSON with fields: "
                '"target_visible" (boolean), "indicator_color" '
                '(one of "red", "green", "yellow", "other", "unknown"), '
                '"reason" (short Japanese sentence describing the visually observed evidence). '
                "No Markdown."
            )
            response = client.responses.create(
                model=self.model,
                input=[{
                    "role": "user",
                    "content": [
                        {"type": "input_text", "text": prompt},
                        {"type": "input_image", "image_url": f"data:image/png;base64,{encoded}", "detail": "high"},
                    ],
                }],
            )
            raw_text = response.output_text.strip()
            # Remove unexpected Markdown fences if returned by the model.
            if raw_text.startswith("```"):
                lines = raw_text.splitlines()
                raw_text = "\n".join(lines[1:-1]).strip()
            prediction = json.loads(raw_text)
            if not isinstance(prediction, dict):
                raise ValueError("VLM output must be a JSON object")
            visible = prediction.get("target_visible")
            color = prediction.get("indicator_color")
            if not isinstance(visible, bool) or color not in COLORS:
                raise ValueError("Invalid VLM output fields")
            if not visible:
                color = "unknown"
            status = {
                "green": "NORMAL",
                "red": "ABNORMAL",
                "yellow": "WARNING",
                "other": "UNKNOWN",
                "unknown": "UNKNOWN",
            }[color]
            result = {
                "equipment": equipment,
                "success": True,
                "image_path": str(image_path),
                "target_visible": visible,
                "indicator_color": color,
                "status": status,
                "reason": str(prediction.get("reason", ""))[:400],
                "model": self.model,
                "inference_sec": round(time.monotonic() - started, 2),
            }
        except Exception as exc:
            result = {
                "equipment": equipment,
                "success": False,
                "image_path": str(image_path),
                "target_visible": False,
                "indicator_color": "unknown",
                "status": "ERROR",
                "reason": f"VLM processing failed: {type(exc).__name__}: {exc}"[:400],
                "model": self.model,
                "inference_sec": round(time.monotonic() - started, 2),
            }
        self.pending.put(result)

    def drain_results(self):
        while True:
            try:
                result = self.pending.get_nowait()
            except queue.Empty:
                return
            serialized = json.dumps(result, ensure_ascii=False)
            msg = String()
            msg.data = serialized
            self.publish_result.publish(msg)
            try:
                with self.result_file.open("a", encoding="utf-8") as f:
                    f.write(serialized + "\n")
            except OSError as exc:
                self.get_logger().error(f"Unable to write result log: {exc}")
            level_msg = f"Inspection result: equipment={result['equipment']} status={result['status']} color={result['indicator_color']}"
            if result["success"]:
                self.get_logger().info(level_msg)
            else:
                self.get_logger().error(level_msg + ": " + result["reason"])

    def close(self):
        self.executor_pool.shutdown(wait=False, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = InspectionVLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
