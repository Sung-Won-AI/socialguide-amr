import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range

from amr_interfaces.msg import McuStatus, SafetyState
from jetson.amr_core.packet import DriveCommand
from jetson.amr_core.serial_bridge import SerialBridge
from jetson.amr_core.transport import SerialTransport
from protocol.protocol_constants import DriveControlFlag, SystemState


class McuBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("mcu_bridge_node")
        self.declare_parameter("port", "/dev/ttyUSB_mcu")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("status_timeout_s", 0.35)
        self.declare_parameter("maximum_speed_mps", 0.5)
        transport = SerialTransport(
            str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
        )
        self.bridge = SerialBridge(
            transport,
            status_timeout_s=float(self.get_parameter("status_timeout_s").value),
        )
        self.velocity = Twist()
        self.safety: SafetyState | None = None
        self.command_id = 0
        self.publisher = self.create_publisher(McuStatus, "/mcu/status", 10)
        self.ultrasonic_pub = self.create_publisher(Range, "/ultrasonic/front", 10)
        self.sharp_left_pub = self.create_publisher(Range, "/sharp/left", 10)
        self.sharp_right_pub = self.create_publisher(Range, "/sharp/right", 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_velocity, 10)
        self.create_subscription(SafetyState, "/safety/state", self._on_safety, 10)
        rate = float(self.get_parameter("command_rate_hz").value)
        self.create_timer(1.0 / rate, self._tick)

    def _on_velocity(self, message: Twist) -> None:
        self.velocity = message

    def _on_safety(self, message: SafetyState) -> None:
        self.safety = message

    def _drive_command(self) -> DriveCommand:
        self.command_id = (self.command_id + 1) & 0xFFFF
        flags = DriveControlFlag.NONE
        state = SystemState.INIT if self.safety is None else SystemState(int(self.safety.state))
        if state in (SystemState.RUN, SystemState.SLOW):
            flags |= DriveControlFlag.DRIVE_ENABLE
        if state == SystemState.SLOW:
            flags |= DriveControlFlag.SLOW_MODE
        if state == SystemState.CONTROLLED_STOP:
            flags |= DriveControlFlag.CONTROLLED_STOP
        maximum = float(self.get_parameter("maximum_speed_mps").value)
        if self.safety is not None:
            maximum = min(maximum, max(0.0, float(self.safety.speed_limit_mps)))
        enabled = bool(flags & DriveControlFlag.DRIVE_ENABLE)
        return DriveCommand(
            command_id=self.command_id,
            linear_velocity_mm_s=round(self.velocity.linear.x * 1000) if enabled else 0,
            angular_velocity_mrad_s=round(self.velocity.angular.z * 1000) if enabled else 0,
            speed_limit_mm_s=round(maximum * 1000) if enabled else 0,
            control_flags=int(flags),
        )

    def _tick(self) -> None:
        try:
            self.bridge.send_drive_command(self._drive_command())
            statuses = self.bridge.poll()
        except (OSError, RuntimeError, ValueError) as error:
            self.get_logger().error(f"MCU serial error: {error}", throttle_duration_sec=2.0)
            statuses = []
        diagnostics = self.bridge.diagnostics()
        if statuses:
            status = statuses[-1]
            message = McuStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.system_state = int(status.system_state)
            message.safety_flags = int(status.safety_flags)
            message.left_velocity_mps = status.left_velocity_mm_s / 1000.0
            message.right_velocity_mps = status.right_velocity_mm_s / 1000.0
            message.battery_voltage_v = status.battery_voltage_mv / 1000.0
            message.motor_error = status.motor_error
            message.last_command_id = status.last_command_id
            message.rx_error_count = status.rx_error_count
            message.uptime_ms = status.uptime_ms
            message.connected = not diagnostics.status_timed_out
            message.dummy = False
            message.ultrasonic_front_m = self._meters(status.ultrasonic_front_mm)
            message.sharp_left_m = self._meters(status.sharp_left_mm)
            message.sharp_right_m = self._meters(status.sharp_right_mm)
            self.publisher.publish(message)
            self._publish_range(
                self.ultrasonic_pub,
                "ultrasonic_front_link",
                Range.ULTRASOUND,
                status.ultrasonic_front_mm,
                0.02,
                4.0,
            )
            self._publish_range(
                self.sharp_left_pub,
                "sharp_left_link",
                Range.INFRARED,
                status.sharp_left_mm,
                0.04,
                1.5,
            )
            self._publish_range(
                self.sharp_right_pub,
                "sharp_right_link",
                Range.INFRARED,
                status.sharp_right_mm,
                0.04,
                1.5,
            )
        elif diagnostics.status_timed_out:
            message = McuStatus()
            message.stamp = self.get_clock().now().to_msg()
            message.connected = False
            message.dummy = False
            self.publisher.publish(message)

    @staticmethod
    def _meters(distance_mm: int) -> float:
        return float("nan") if distance_mm == 0xFFFF else distance_mm / 1000.0

    def _publish_range(
        self,
        publisher,
        frame_id: str,
        radiation: int,
        distance_mm: int,
        minimum: float,
        maximum: float,
    ) -> None:
        if distance_mm == 0xFFFF:
            return
        message = Range()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = frame_id
        message.radiation_type = radiation
        message.field_of_view = 0.35
        message.min_range = minimum
        message.max_range = maximum
        message.range = distance_mm / 1000.0
        publisher.publish(message)

    def destroy_node(self):
        self.bridge.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = McuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
