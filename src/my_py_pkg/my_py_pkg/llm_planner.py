
import json
import os
import queue
import threading
import time
import traceback

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI


ALLOWED_LOCATIONS = {"HOME", "A", "B", "C"}


class LLMPlanner(Node):

    def __init__(self):
        super().__init__('llm_planner')

        self.model = os.getenv(
            "OPENAI_MODEL", "gpt-4o-mini"
        )

        self.client = OpenAI(
            api_key=os.environ["OPENAI_API_KEY"],
            timeout=20.0,
            max_retries=0
        )

        # 自然言語指示を購読
        self.instruction_sub = self.create_subscription(
            String,
            '/inspection/instruction',
            self.instruction_callback,
            10
        )

        # AIが作成した計画を配信
        self.plan_pub = self.create_publisher(
            String,
            '/inspection/plan',
            10
        )

        # ワーカースレッドから結果を受け取る
        self.results = queue.Queue()
        self.busy = False

        self.timer = self.create_timer(
            0.1,
            self.check_results
        )

        self.get_logger().info(
            f'LLM Planner started: {self.model}'
        )

    def instruction_callback(self, msg):
        instruction = msg.data.strip()

        if not instruction:
            self.get_logger().warning(
                'Empty instruction rejected'
            )
            return

        if self.busy:
            self.get_logger().warning(
                'Planner busy: instruction rejected'
            )
            return

        self.busy = True

        self.get_logger().info(
            f'Instruction received: {instruction}'
        )

        threading.Thread(
            target=self.run_inference,
            args=(instruction,),
            daemon=True
        ).start()

    def run_inference(self, instruction):
        start = time.monotonic()

        try:
            response = self.client.responses.create(
                model=self.model,
                instructions=(
                    "You are a factory robot task planner. "
                    "Convert Japanese inspection instructions "
                    "into an ordered list of location IDs. "
                    "Allowed IDs: HOME, A, B, C. "
                    "Preserve the requested visiting order. "
                    "Never invent location IDs. "
                    "If the instruction is ambiguous, "
                    "unsupported or mentions unknown "
                    "locations, return an empty list. "
                    "Do not add unrequested destinations."
                ),
                input=instruction,
                text={
                    "format": {
                        "type": "json_schema",
                        "name": "inspection_plan",
                        "strict": True,
                        "schema": {
                            "type": "object",
                            "properties": {
                                "waypoints": {
                                    "type": "array",
                                    "items": {
                                        "type": "string"
                                    }
                                }
                            },
                            "required": ["waypoints"],
                            "additionalProperties": False
                        }
                    }
                }
            )

            if not response.output_text:
                raise ValueError(
                    'Empty model response'
                )

            data = json.loads(response.output_text)
            waypoints = data["waypoints"]

            if (
                not isinstance(waypoints, list)
                or not 1 <= len(waypoints) <= 4
                or any(
                    not isinstance(w, str)
                    or w not in ALLOWED_LOCATIONS
                    for w in waypoints
                )
            ):
                raise ValueError(
                    f'Invalid plan: {waypoints}'
                )

            elapsed = time.monotonic() - start

            self.results.put(
                ("success", waypoints, elapsed)
            )

        except Exception:
            error_detail = traceback.format_exc()

            self.results.put(
                ("error", error_detail, 0.0)
            )

    def check_results(self):
        try:
            status, value, elapsed = (
                self.results.get_nowait()
            )

        except queue.Empty:
            return

        self.busy = False

        if status == "error":
            self.get_logger().error(
                f'LLM inference failed: {value}'
            )
            return

        msg = String()
        msg.data = json.dumps(
            {"waypoints": value},
            ensure_ascii=False
        )

        self.plan_pub.publish(msg)

        self.get_logger().info(
            f'AI plan: {msg.data}'
        )

        self.get_logger().info(
            f'Inference time: {elapsed:.2f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = LLMPlanner()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
