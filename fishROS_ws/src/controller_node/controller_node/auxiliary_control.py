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


class AuxiliaryControl(Node):
    def __init__(self):
        super().__init__("auxiliary_control")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("trigger_pressed_threshold", 0.5)
        self.declare_parameter("servo_20_button", 4)  # Left bumper
        self.declare_parameter("servo_01_button", 5)  # Right bumper
        self.declare_parameter("left_trigger_axis", 2)
        self.declare_parameter("right_trigger_axis", 5)
        self.declare_parameter("actuator_direction_pin", "21")
        self.declare_parameter("actuator_speed_pin", "28")
        self.declare_parameter("actuator_speed", 1.0)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.trigger_pressed_threshold = float(
            self.get_parameter("trigger_pressed_threshold").value
        )
        self.servo_buttons = {
            "20": int(self.get_parameter("servo_20_button").value),
            "01": int(self.get_parameter("servo_01_button").value),
        }
        self.left_trigger_axis = int(self.get_parameter("left_trigger_axis").value)
        self.right_trigger_axis = int(self.get_parameter("right_trigger_axis").value)
        self.actuator_direction_pin = str(
            self.get_parameter("actuator_direction_pin").value
        ).zfill(2)
        self.actuator_speed_pin = str(
            self.get_parameter("actuator_speed_pin").value
        ).zfill(2)
        self.actuator_speed = max(
            0.0, min(1.0, float(self.get_parameter("actuator_speed").value))
        )

        self.button_was_pressed = {pin: False for pin in self.servo_buttons}
        self.servo_at_max = {pin: False for pin in self.servo_buttons}
        self.actuator_state = "stop"

        self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2)
        self.get_logger().info(f"Using Pico serial control on {self.port} @ {self.baud}")

        self.subscription = self.create_subscription(Joy, "joy", self.joy_callback, 10)

    def joy_callback(self, msg):
        self._handle_servo_toggles(msg)
        self._handle_actuator(msg)

    def _handle_servo_toggles(self, msg):
        for pin, button_index in self.servo_buttons.items():
            pressed = self._button_pressed(msg, button_index)
            if pressed and not self.button_was_pressed[pin]:
                self.servo_at_max[pin] = not self.servo_at_max[pin]
                servo_min, servo_max = SERVO_LIMITS[pin]
                value = servo_max if self.servo_at_max[pin] else servo_min
                self._write_command(pin, value)
                state = "max" if self.servo_at_max[pin] else "min"
                self.get_logger().info(f"Servo {pin} toggled to {state} ({value:.2f})")

            self.button_was_pressed[pin] = pressed

    def _handle_actuator(self, msg):
        left_pressed = self._trigger_pressed(msg, self.left_trigger_axis)
        right_pressed = self._trigger_pressed(msg, self.right_trigger_axis)

        if right_pressed and not left_pressed:
            next_state = "forward"
        elif left_pressed and not right_pressed:
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

    def _trigger_pressed(self, msg, index):
        return (
            0 <= index < len(msg.axes)
            and msg.axes[index] < self.trigger_pressed_threshold
        )

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
