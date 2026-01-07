import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from holoocean_interfaces.msg import AgentCommand
from enum import Enum


class AgentType(Enum):
    BLUEROV = 1
    SURFACE_VESSEL = 2


agent_keys = {
    'bluerov': AgentType.BLUEROV,
    'surface_vessel': AgentType.SURFACE_VESSEL
}


class JoyToAgentCommand(Node):
    def __init__(self):
        super().__init__('joy_holoocean')

        self.publisher_ = self.create_publisher(AgentCommand, '/holoocean/command/agent', 10)
        agent_param = self.declare_parameter("agent", "bluerov").value
        self.agent = agent_keys.get(agent_param.lower(), AgentType.BLUEROV)
        timer_period = 0.02
        self.timer = self.create_timer(timer_period, self.move)
        self.get_logger().info('Joy to AgentCommand node has started.')

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))

    def get_axis_value(self, msg, index):
        return msg.axes[index] if len(msg.axes) > index else 0.0

    def update_trim(self, trim_value, input_value):
        if input_value > 0.5:
            trim_value += self.trim_step
        elif input_value < -0.5:
            trim_value -= self.trim_step
        return self.clamp(trim_value, -self.max_trim, self.max_trim)

    def move(self):
        agent_cmd = AgentCommand()
        agent_cmd.header.frame_id = 'auv0'  # TODO: get from parameter
        # this goes down
        agent_cmd.command = [
            (0.),
            (0.),
            (0.),
            (0.),
            (2.),
            (-2.),
            (-2.),
            (2.)
        ]
        """
        agent_cmd.command = [
                (900000000.0),
                (900000000.0)
        ]
        """
        self.get_logger().info("Sending command!")
        self.publisher_.publish(agent_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToAgentCommand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
