"""
Joystick control for the Pico-driven servos and linear actuator.
"""

import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy


SERVO_LIMITS = {
    "20": (0.42, 0.54),
    "01": (0.45, 1.00),
}
A_BUTTON = 0
B_BUTTON = 1
LEFT_BUMPER = 4
RIGHT_BUMPER = 5
DPAD_HORIZONTAL_AXIS = 6
SERVO_OPEN = 0.0
SERVO_CLOSED = 1.0
SERVO_STOP = 0.5


class AuxiliaryControl(Node):
    def __init__(self):
        super().__init__("auxiliary_control")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("open_claw_button", A_BUTTON)
        self.declare_parameter("close_claw_button", B_BUTTON)
        self.declare_parameter("retract_actuator_button", LEFT_BUMPER)
        self.declare_parameter("extend_actuator_button", RIGHT_BUMPER)
        self.declare_parameter("rotate_claw_axis", DPAD_HORIZONTAL_AXIS)
        self.declare_parameter("actuator_direction_pin", "21")
        self.declare_parameter("actuator_speed_pin", "28")
        self.declare_parameter("actuator_speed", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.open_claw_button = int(self.get_parameter("open_claw_button").value)
        self.close_claw_button = int(self.get_parameter("close_claw_button").value)
        self.retract_actuator_button = int(
            self.get_parameter("retract_actuator_button").value
        )
        self.extend_actuator_button = int(
            self.get_parameter("extend_actuator_button").value
        )
        self.rotate_claw_axis = int(self.get_parameter("rotate_claw_axis").value)
        self.actuator_direction_pin = str(
            self.get_parameter("actuator_direction_pin").value
        ).zfill(2)
        self.actuator_speed_pin = str(
            self.get_parameter("actuator_speed_pin").value
        ).zfill(2)
        self.actuator_speed = max(
            0.0, min(1.0, float(self.get_parameter("actuator_speed").value))
        )

        self.servo_values = {"20": None, "01": None}
        self.actuator_state = "stop"

        self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)
        self.get_logger().info(f"Using Pico serial control on {self.port} @ {self.baud}")

        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)

    def joy_callback(self, msg):
        self._handle_claw(msg)
        self._handle_claw_rotation(msg)
        self._handle_actuator(msg)

    def _handle_claw(self, msg):
        if self._button_pressed(msg, self.open_claw_button):
            self._write_servo("20", SERVO_OPEN)
        elif self._button_pressed(msg, self.close_claw_button):
            self._write_servo("20", SERVO_CLOSED)

    def _handle_claw_rotation(self, msg):
        dpad_horizontal = self._axis_value(msg, self.rotate_claw_axis)
        if dpad_horizontal > 0.5:
            self._write_servo("01", SERVO_OPEN)
        elif dpad_horizontal < -0.5:
            self._write_servo("01", SERVO_CLOSED)
        else:
            self._write_servo("01", SERVO_STOP)

    def _handle_actuator(self, msg):
        retract_pressed = self._button_pressed(msg, self.retract_actuator_button)
        extend_pressed = self._button_pressed(msg, self.extend_actuator_button)

        if extend_pressed and not retract_pressed:
            next_state = "forward"
        elif retract_pressed and not extend_pressed:
            next_state = "back"
        else:
            next_state = "stop"

        if next_state == self.actuator_state:
            return

        if next_state == "forward":
            self._write_command(self.actuator_direction_pin, 1.0)
            self._write_command(self.actuator_speed_pin, self.actuator_speed)
        elif next_state == "back":
            self._write_command(self.actuator_direction_pin, 0.0)
            self._write_command(self.actuator_speed_pin, self.actuator_speed)
        else:
            self._stop_actuator()

        self.actuator_state = next_state
        self.get_logger().info(f"Actuator state: {next_state}")

    def _button_pressed(self, msg, index):
        return 0 <= index < len(msg.buttons) and msg.buttons[index] == 1

    def _axis_value(self, msg, index):
        if 0 <= index < len(msg.axes):
            return msg.axes[index]
        return 0.0

    def _write_servo(self, pin, normalized):
        normalized = max(0.0, min(1.0, float(normalized)))
        if self.servo_values.get(pin) == normalized:
            return

        servo_min, servo_max = SERVO_LIMITS[pin]
        value = servo_min + ((servo_max - servo_min) * normalized)
        self._write_command(pin, value)
        self.servo_values[pin] = normalized
        self.get_logger().info(f"Servo {pin}: {normalized:.2f} ({value:.2f})")

    def _write_command(self, pin, value):
        value = max(0.0, min(1.0, float(value)))
        command = f"z{str(pin).zfill(2)}{value:04.2f}x\n"
        self.serial_conn.write(command.encode())
        self.serial_conn.flush()

    def _stop_actuator(self):
        self._write_command(self.actuator_direction_pin, 0.5)
        self._write_command(self.actuator_speed_pin, 0.0)

    def destroy_node(self):
        if hasattr(self, "serial_conn") and self.serial_conn.is_open:
            self._stop_actuator()
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AuxiliaryControl()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
