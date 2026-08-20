import json
from urllib.error import URLError
from urllib.request import Request, urlopen

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from amr_interfaces.msg import CliffState, McuStatus, ObstacleInfo, SafetyState

from .status_mapper import make_monitoring_payload


class MonitoringAdapterNode(Node):
    def __init__(self) -> None:
        super().__init__("monitoring_adapter_node")
        self.declare_parameter("monitoring_url", "http://127.0.0.1:8080/api/status")
        self.declare_parameter("publish_rate_hz", 5.0)
        self.url = str(self.get_parameter("monitoring_url").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        self.safety = None
        self.mcu = None
        self.obstacle = None
        self.cliff = None
        self.velocity = None
        self.battery = None
        self.dummy_active = False
        self.scenario_name = ""
        self.failure_reported = False

        self.create_subscription(SafetyState, "/safety/state", lambda m: setattr(self, "safety", m), 10)
        self.create_subscription(McuStatus, "/mcu/status", lambda m: setattr(self, "mcu", m), 10)
        self.create_subscription(ObstacleInfo, "/obstacle/info", lambda m: setattr(self, "obstacle", m), 10)
        self.create_subscription(CliffState, "/cliff/state", lambda m: setattr(self, "cliff", m), 10)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: setattr(self, "velocity", m), 10)
        self.create_subscription(BatteryState, "/battery/state", lambda m: setattr(self, "battery", m), 10)
        self.create_subscription(Bool, "/dummy/active", self._on_dummy, 10)
        self.create_subscription(String, "/dummy/scenario_name", self._on_scenario, 10)
        self.timer = self.create_timer(1.0 / rate, self._publish)

    def _on_dummy(self, message: Bool) -> None:
        self.dummy_active = bool(message.data)

    def _on_scenario(self, message: String) -> None:
        self.scenario_name = message.data

    def _publish(self) -> None:
        payload = make_monitoring_payload(
            safety=self.safety,
            mcu=self.mcu,
            obstacle=self.obstacle,
            cliff=self.cliff,
            velocity=self.velocity,
            battery=self.battery,
            dummy_active=self.dummy_active,
            scenario_name=self.scenario_name,
        )
        request = Request(
            self.url,
            data=json.dumps(payload, allow_nan=False).encode("utf-8"),
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urlopen(request, timeout=0.15) as response:
                if response.status != 200:
                    raise URLError(f"HTTP {response.status}")
            self.failure_reported = False
        except (URLError, TimeoutError) as exc:
            if not self.failure_reported:
                self.get_logger().warning(f"monitoring API unavailable: {exc}")
                self.failure_reported = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MonitoringAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

