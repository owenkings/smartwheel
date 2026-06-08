import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
except ImportError:
    rclpy = None
    Node = object
    Twist = None
    Odometry = None
    String = None
    TransformStamped = None
    TransformBroadcaster = None

from wheelchair_base.kinematics import DifferentialDriveModel, OdometryState, yaw_to_quaternion
from wheelchair_base.modbus_rtu import ModbusRtuClient, ModbusSerialConfig, from_i16


@dataclass
class ZlacRegisterMap:
    command_left_register: int = -1
    command_right_register: int = -1
    feedback_left_register: int = -1
    feedback_right_register: int = -1
    enable_register: int = -1
    enable_value: int = 1
    disable_value: int = 0
    control_mode_register: int = -1
    control_word_register: int = -1
    async_mode_register: int = -1
    velocity_mode_value: int = 3
    async_mode_value: int = 0
    clear_fault_value: int = 6
    drive_enable_value: int = 8
    stop_value: int = 7
    emergency_stop_value: int = 5

    @property
    def command_enabled(self) -> bool:
        return self.command_left_register >= 0 and self.command_right_register >= 0

    @property
    def feedback_enabled(self) -> bool:
        return self.feedback_left_register >= 0 and self.feedback_right_register >= 0


class Zlac8030DriverNode(Node):
    """Differential base driver for ZLAC8030/KeepLINK installations.

    The transport is Modbus RTU over a serial device exposed by the KeepLINK
    adapter or USB-RS485 bridge. Register addresses are intentionally YAML
    parameters because ZLAC8030 firmware/register maps vary by model and wiring.
    """

    def __init__(self):
        super().__init__("zlac8030_driver_node")
        self.declare_parameter("mode", "real")
        self.declare_parameter("serial_port", "/dev/smartwheel_zlac8030")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("serial_timeout_sec", 0.05)
        self.declare_parameter("left_slave_id", 1)
        self.declare_parameter("right_slave_id", 2)
        self.declare_parameter("single_slave_dual_axis", False)
        self.declare_parameter("rpm_to_register_scale", 1.0)
        self.declare_parameter("register_to_rpm_scale", 1.0)
        self.declare_parameter("invert_left", False)
        self.declare_parameter("invert_right", True)
        self.declare_parameter("wheel_radius_m", 0.165)
        self.declare_parameter("wheel_separation_m", 0.58)
        self.declare_parameter("max_linear_mps", 0.4)
        self.declare_parameter("max_angular_rps", 1.0)
        self.declare_parameter("command_timeout_sec", 0.5)
        self.declare_parameter("motion_control_enabled", False)
        self.declare_parameter("write_dual_axis_command_together", False)
        self.declare_parameter("initialize_motion_on_first_command", True)
        self.declare_parameter("hold_zero_before_motion_init", False)
        self.declare_parameter("release_motion_after_zero_sec", 0.75)
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("command_left_register", -1)
        self.declare_parameter("command_right_register", -1)
        self.declare_parameter("feedback_left_register", -1)
        self.declare_parameter("feedback_right_register", -1)
        self.declare_parameter("enable_register", -1)
        self.declare_parameter("enable_value", 1)
        self.declare_parameter("disable_value", 0)
        self.declare_parameter("control_mode_register", -1)
        self.declare_parameter("control_word_register", -1)
        self.declare_parameter("async_mode_register", -1)
        self.declare_parameter("velocity_mode_value", 3)
        self.declare_parameter("async_mode_value", 0)
        self.declare_parameter("clear_fault_value", 6)
        self.declare_parameter("drive_enable_value", 8)
        self.declare_parameter("stop_value", 7)
        self.declare_parameter("emergency_stop_value", 5)

        self.mode = self.get_parameter("mode").value
        self.left_slave_id = int(self.get_parameter("left_slave_id").value)
        self.right_slave_id = int(self.get_parameter("right_slave_id").value)
        self.single_slave_dual_axis = bool(self.get_parameter("single_slave_dual_axis").value)
        self.rpm_to_register_scale = float(self.get_parameter("rpm_to_register_scale").value)
        self.register_to_rpm_scale = float(self.get_parameter("register_to_rpm_scale").value)
        self.invert_left = bool(self.get_parameter("invert_left").value)
        self.invert_right = bool(self.get_parameter("invert_right").value)
        self.command_timeout_sec = float(self.get_parameter("command_timeout_sec").value)
        self.motion_control_enabled = bool(self.get_parameter("motion_control_enabled").value)
        self.write_dual_axis_command_together = bool(
            self.get_parameter("write_dual_axis_command_together").value
        )
        self.initialize_motion_on_first_command = bool(
            self.get_parameter("initialize_motion_on_first_command").value
        )
        self.hold_zero_before_motion_init = bool(
            self.get_parameter("hold_zero_before_motion_init").value
        )
        self.release_motion_after_zero_sec = float(
            self.get_parameter("release_motion_after_zero_sec").value
        )
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.base_frame_id = self.get_parameter("base_frame_id").value
        self.registers = ZlacRegisterMap(
            command_left_register=int(self.get_parameter("command_left_register").value),
            command_right_register=int(self.get_parameter("command_right_register").value),
            feedback_left_register=int(self.get_parameter("feedback_left_register").value),
            feedback_right_register=int(self.get_parameter("feedback_right_register").value),
            enable_register=int(self.get_parameter("enable_register").value),
            enable_value=int(self.get_parameter("enable_value").value),
            disable_value=int(self.get_parameter("disable_value").value),
            control_mode_register=int(self.get_parameter("control_mode_register").value),
            control_word_register=int(self.get_parameter("control_word_register").value),
            async_mode_register=int(self.get_parameter("async_mode_register").value),
            velocity_mode_value=int(self.get_parameter("velocity_mode_value").value),
            async_mode_value=int(self.get_parameter("async_mode_value").value),
            clear_fault_value=int(self.get_parameter("clear_fault_value").value),
            drive_enable_value=int(self.get_parameter("drive_enable_value").value),
            stop_value=int(self.get_parameter("stop_value").value),
            emergency_stop_value=int(self.get_parameter("emergency_stop_value").value),
        )
        self.model = DifferentialDriveModel(
            wheel_radius_m=float(self.get_parameter("wheel_radius_m").value),
            wheel_separation_m=float(self.get_parameter("wheel_separation_m").value),
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_rps=float(self.get_parameter("max_angular_rps").value),
        )
        self.modbus = ModbusRtuClient(
            ModbusSerialConfig(
                port=self.get_parameter("serial_port").value,
                baud_rate=int(self.get_parameter("baud_rate").value),
                timeout_sec=float(self.get_parameter("serial_timeout_sec").value),
            )
        )
        self.odom_state = OdometryState()
        self.last_cmd = Twist()
        self.last_cmd_time = self.get_clock().now()
        self.last_odom_time = self.get_clock().now()
        self.last_wheel_rpm: Tuple[float, float] = (0.0, 0.0)
        self.warned_no_registers = False
        self.warned_motion_disabled = False
        self.warned_zero_before_motion_init = False
        self.warned_feedback_read_failed = False
        self.last_command_write_ok = False
        self.motion_initialized = False
        self.last_nonzero_command_monotonic = time.monotonic()

        self.odom_pub = self.create_publisher(Odometry, "/wheel/odom", 20)
        self.status_pub = self.create_publisher(String, "/base/status", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel, 10)
        self.timer = self.create_timer(
            1.0 / float(self.get_parameter("publish_rate_hz").value), self.tick
        )

    def on_cmd_vel(self, msg):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            self.get_logger().warning("ignoring /cmd_vel_safe with non-finite values")
            return
        self.last_cmd = msg
        self.last_cmd_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = now
        cmd_age = (now - self.last_cmd_time).nanoseconds / 1e9
        linear = self.last_cmd.linear.x if cmd_age <= self.command_timeout_sec else 0.0
        angular = self.last_cmd.angular.z if cmd_age <= self.command_timeout_sec else 0.0
        linear, angular = self.model.clamp_twist(linear, angular)
        target_left_rpm, target_right_rpm = self.model.twist_to_wheel_rpm(linear, angular)
        target_left_rpm, target_right_rpm = self._apply_direction(target_left_rpm, target_right_rpm)

        actual_left_rpm, actual_right_rpm = (
            (target_left_rpm, target_right_rpm) if self.mode != "real" else (0.0, 0.0)
        )
        if self.mode == "real":
            self.last_command_write_ok = self._write_wheel_commands(
                target_left_rpm, target_right_rpm
            )
            feedback = self._read_feedback()
            if feedback is not None:
                actual_left_rpm, actual_right_rpm = feedback
            elif not self.registers.feedback_enabled and self.last_command_write_ok:
                # Open-loop odom only: with no feedback registers the commanded
                # speed is the sole motion estimate. When feedback IS configured
                # but the read failed, do NOT disguise the target as measured
                # motion; actual stays 0.0 for this cycle.
                actual_left_rpm, actual_right_rpm = target_left_rpm, target_right_rpm
        else:
            self.last_command_write_ok = True
        self.last_wheel_rpm = (actual_left_rpm, actual_right_rpm)

        odom_left_rpm, odom_right_rpm = self._remove_direction(actual_left_rpm, actual_right_rpm)
        odom_linear, odom_angular = self.model.wheel_rpm_to_twist(odom_left_rpm, odom_right_rpm)
        self.odom_state.integrate(odom_linear, odom_angular, dt)
        self._publish_odom(odom_linear, odom_angular, now)
        self._publish_status(cmd_age)

    def _apply_direction(self, left_rpm: float, right_rpm: float) -> Tuple[float, float]:
        return (-left_rpm if self.invert_left else left_rpm, -right_rpm if self.invert_right else right_rpm)

    def _remove_direction(self, left_rpm: float, right_rpm: float) -> Tuple[float, float]:
        return (-left_rpm if self.invert_left else left_rpm, -right_rpm if self.invert_right else right_rpm)

    def _write_wheel_commands(self, left_rpm: float, right_rpm: float) -> bool:
        command_is_zero = abs(left_rpm) <= 1e-6 and abs(right_rpm) <= 1e-6
        now_monotonic = time.monotonic()
        if not command_is_zero:
            self.last_nonzero_command_monotonic = now_monotonic
        if not self.registers.command_enabled:
            if not self.warned_no_registers:
                self.get_logger().warning(
                    "ZLAC8030 command registers are disabled; real mode will not send motor speed commands"
                )
                self.warned_no_registers = True
            return False
        if not self.motion_control_enabled:
            if not command_is_zero and not self.warned_motion_disabled:
                self.get_logger().warning(
                    "ZLAC8030 motion_control_enabled is false; non-zero wheel commands are blocked"
                )
                self.warned_motion_disabled = True
            return False
        if (
            self.motion_control_enabled
            and self.initialize_motion_on_first_command
            and not self.motion_initialized
            and command_is_zero
            and not self.hold_zero_before_motion_init
        ):
            if not self.warned_zero_before_motion_init:
                self.get_logger().warning(
                    "ZLAC8030 zero command received before motion initialization; "
                    "skipping drive_enable to avoid engaging the holding brake while idle"
                )
                self.warned_zero_before_motion_init = True
            return True
        if (
            self.motion_control_enabled
            and self.motion_initialized
            and command_is_zero
            and self.release_motion_after_zero_sec >= 0.0
            and now_monotonic - self.last_nonzero_command_monotonic >= self.release_motion_after_zero_sec
        ):
            return self._write_control_stop(emergency=False)
        left_value = int(round(left_rpm * self.rpm_to_register_scale))
        right_value = int(round(right_rpm * self.rpm_to_register_scale))
        try:
            if (
                self.motion_control_enabled
                and self.initialize_motion_on_first_command
                and not self.motion_initialized
            ):
                if not self._initialize_motion_mode():
                    return False
            if self.single_slave_dual_axis:
                if (
                    self.write_dual_axis_command_together
                    and self.registers.command_right_register == self.registers.command_left_register + 1
                ):
                    self.modbus.write_multiple_registers(
                        self.left_slave_id,
                        self.registers.command_left_register,
                        [left_value, right_value],
                    )
                else:
                    self.modbus.write_single_register(self.left_slave_id, self.registers.command_left_register, left_value)
                    self.modbus.write_single_register(self.left_slave_id, self.registers.command_right_register, right_value)
            else:
                self.modbus.write_single_register(self.left_slave_id, self.registers.command_left_register, left_value)
                self.modbus.write_single_register(self.right_slave_id, self.registers.command_right_register, right_value)
        except Exception as exc:
            self.get_logger().warning(f"ZLAC8030 command write failed: {exc}")
            return False
        return True

    def _read_feedback(self) -> Optional[Tuple[float, float]]:
        if not self.registers.feedback_enabled:
            return None
        try:
            if self.single_slave_dual_axis:
                left_raw = self.modbus.read_holding_registers(self.left_slave_id, self.registers.feedback_left_register, 1)[0]
                right_raw = self.modbus.read_holding_registers(self.left_slave_id, self.registers.feedback_right_register, 1)[0]
            else:
                left_raw = self.modbus.read_holding_registers(self.left_slave_id, self.registers.feedback_left_register, 1)[0]
                right_raw = self.modbus.read_holding_registers(self.right_slave_id, self.registers.feedback_right_register, 1)[0]
            self.warned_feedback_read_failed = False
            return from_i16(left_raw) * self.register_to_rpm_scale, from_i16(right_raw) * self.register_to_rpm_scale
        except Exception as exc:
            if not self.warned_feedback_read_failed:
                self.get_logger().warning(f"ZLAC8030 feedback read failed: {exc}")
                self.warned_feedback_read_failed = True
            return None

    def _write_if_configured(self, register: int, value: int) -> bool:
        if register < 0:
            return False
        self.modbus.write_single_register(self.left_slave_id, register, value)
        if not self.single_slave_dual_axis:
            self.modbus.write_single_register(self.right_slave_id, register, value)
        return True

    def _initialize_motion_mode(self) -> bool:
        try:
            self._write_if_configured(
                self.registers.control_mode_register,
                self.registers.velocity_mode_value,
            )
            self._write_if_configured(
                self.registers.async_mode_register,
                self.registers.async_mode_value,
            )
            self._write_if_configured(
                self.registers.control_word_register,
                self.registers.clear_fault_value,
            )
            self._write_if_configured(
                self.registers.control_word_register,
                self.registers.drive_enable_value,
            )
            self.motion_initialized = True
            return True
        except Exception as exc:
            self.get_logger().warning(f"ZLAC8030 motion initialization failed: {exc}")
            self.motion_initialized = False
            return False

    def _write_control_stop(self, emergency: bool = True) -> bool:
        if self.registers.control_word_register < 0:
            return False
        value = (
            self.registers.emergency_stop_value
            if emergency
            else self.registers.stop_value
        )
        try:
            self.modbus.write_single_register(
                self.left_slave_id,
                self.registers.control_word_register,
                value,
            )
            if not self.single_slave_dual_axis:
                self.modbus.write_single_register(
                    self.right_slave_id,
                    self.registers.control_word_register,
                    value,
                )
            self.motion_initialized = False
            return True
        except Exception as exc:
            self.get_logger().warning(f"ZLAC8030 control stop write failed: {exc}")
            return False

    def _write_enable_state(self, enabled: bool) -> bool:
        if self.registers.enable_register < 0:
            return False
        value = self.registers.enable_value if enabled else self.registers.disable_value
        try:
            self.modbus.write_single_register(
                self.left_slave_id,
                self.registers.enable_register,
                value,
            )
            if not self.single_slave_dual_axis:
                self.modbus.write_single_register(
                    self.right_slave_id,
                    self.registers.enable_register,
                    value,
                )
        except Exception as exc:
            state = "enable" if enabled else "disable"
            self.get_logger().warning(f"ZLAC8030 {state} write failed: {exc}")
            return False
        return True

    def _shutdown_hardware(self):
        if self.mode != "real":
            return
        if self.registers.command_enabled and self.motion_control_enabled:
            self._write_wheel_commands(0.0, 0.0)
        if not self._write_control_stop(emergency=True):
            self._write_enable_state(False)

    def _publish_odom(self, linear: float, angular: float, now):
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.odom_frame_id
        msg.child_frame_id = self.base_frame_id
        msg.pose.pose.position.x = self.odom_state.x
        msg.pose.pose.position.y = self.odom_state.y
        qx, qy, qz, qw = yaw_to_quaternion(self.odom_state.yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = linear
        msg.twist.twist.angular.z = angular
        large = 1e6
        msg.pose.covariance[0] = 0.05     # x
        msg.pose.covariance[7] = 0.05     # y
        msg.pose.covariance[14] = large   # z (unobservable)
        msg.pose.covariance[21] = large   # roll (unobservable)
        msg.pose.covariance[28] = large   # pitch (unobservable)
        msg.pose.covariance[35] = 0.10    # yaw
        msg.twist.covariance[0] = 0.05    # vx
        msg.twist.covariance[7] = large   # vy (unobservable)
        msg.twist.covariance[14] = large  # vz (unobservable)
        msg.twist.covariance[21] = large  # wx (unobservable)
        msg.twist.covariance[28] = large  # wy (unobservable)
        msg.twist.covariance[35] = 0.10   # wz
        self.odom_pub.publish(msg)
        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header = msg.header
            tf.child_frame_id = msg.child_frame_id
            tf.transform.translation.x = msg.pose.pose.position.x
            tf.transform.translation.y = msg.pose.pose.position.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = msg.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def _publish_status(self, cmd_age: float):
        msg = String()
        register_state = "configured" if self.registers.command_enabled else "registers_disabled"
        feedback_state = "feedback" if self.registers.feedback_enabled else "open_loop_odom"
        real_motion_enabled = (
            self.mode != "real"
            or (self.registers.command_enabled and self.motion_control_enabled)
        )
        msg.data = (
            f"mode={self.mode}; command={register_state}; odom={feedback_state}; "
            f"real_motion_enabled={str(real_motion_enabled).lower()}; "
            f"motion_control_enabled={str(self.motion_control_enabled).lower()}; "
            f"motion_initialized={str(self.motion_initialized).lower()}; "
            f"last_command_write_ok={str(self.last_command_write_ok).lower()}; "
            f"left_rpm={self.last_wheel_rpm[0]:.2f}; right_rpm={self.last_wheel_rpm[1]:.2f}; "
            f"cmd_age={cmd_age:.2f}"
        )
        self.status_pub.publish(msg)

    def destroy_node(self):
        try:
            self._shutdown_hardware()
        finally:
            self.modbus.close()
            super().destroy_node()


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 Python packages are required to run this node")
    rclpy.init(args=args)
    node = Zlac8030DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
