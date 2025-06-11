"""
Date Created: 06/5/2025
Author(s): Larry Zhao
Description: Publishes a Twist message to maintain depth
Subscribers: bar02/depth, stabilization_toggle, twist
Publishers: Twist
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, Int32

class StabilizationPub(Node):
    def __init__(self):
        super().__init__('stabilization_node')
        
        # Parameters
        self.kp = -0.5  # Proportional gain, negative because going down increases depth
        self.deadzone = 0.05 # deadzone in meters
        self.deadzone_thrust = -0.4 # thrust when in deadzone
        self.max_thrust = 0.6  # Max value of twist message
        
        self.depth_hold_enabled = False
        self.target_depth = None
        self.current_depth = 0.0
        
        self.manual_z_input = 0.0
        self.last_manual_input_time = self.get_clock().now()
        self.manual_input_timeout = 0.5  # seconds to wait before re-engaging auto-hold

        self.status = 0 # 0 = Off, 1 = On, 2 = Manual Override
        
        # Subscribers
        self.depthSub = self.create_subscription(
            Float32, 
            'bar02/depth', 
            self.depth_callback, 
            10)
        self.toggleSub = self.create_subscription(
            Bool, 
            'stabilization_toggle', 
            self.toggle_callback, 
            10)
        self.twistSub = self.create_subscription(
            Twist, 
            'twist', 
            self.manual_input_callback, 
            10)
        
        # Publishers
        self.stab_pub = self.create_publisher(Twist, 'stabilization', 10)
        self.status_pub = self.create_publisher(Int32, 'stabilization_status', 10)
        
        # Timer
        self.create_timer(0.04, self.control_loop)  # 25 Hz
        self.create_timer(0.10, self.status_loop) # 10 Hz
        
        
    # Toggles on and Off
    def toggle_callback(self, msg: Bool):
        self.depth_hold_enabled = msg.data
        if self.depth_hold_enabled:
            self.target_depth = self.current_depth
            self.get_logger().info(f'Depth hold ENABLED. Target depth set to {self.target_depth:.2f} m')
            self.status = 1
        else:
            self.get_logger().info('Depth hold DISABLED')
            self.status = 0
            
    # Get data from depth sensor
    def depth_callback(self, msg: Float32):
        self.current_depth = msg.data
    
    # Manual override
    def manual_input_callback(self, msg: Twist):
        self.manual_z_input = msg.linear.z

        if abs(self.manual_z_input) > 0.08:
            # Manual override detected
            #self.get_logger().info('Manual Override Detected')
            self.status = 2
            self.last_manual_input_time = self.get_clock().now()

    # PID control loop
    def control_loop(self):
        # Toggle
        if not self.depth_hold_enabled:
            return
        
        # Override depth hold
        if abs(self.manual_z_input) > 0.08:
            return
        
        time_since_input = (self.get_clock().now() - self.last_manual_input_time).nanoseconds / 1e9

        # Wait for ROV to settle before resuming stabilization
        if time_since_input < self.manual_input_timeout:
            self.target_depth = self.current_depth
            
            # Publish zero command to indicate "no stabilization right now"
            twist = Twist()
            twist.linear.z = 0.0
            self.stab_pub.publish(twist)
            return

        error = self.target_depth - self.current_depth

        twist = Twist()

        # Apply deadzone
        if abs(error) < self.deadzone:
            twist.linear.z = self.deadzone_thrust
        else:
            output = self.kp * error
            twist.linear.z = max(-self.max_thrust, min(self.max_thrust, output))  # clamp

        self.stab_pub.publish(twist)

    def status_loop(self):
        self.status_pub.publish(self.status)

def main(args=None):
    rclpy.init(args=args)

    stabilizationNode = StabilizationPub()

    rclpy.spin(stabilizationNode)

    stabilizationNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()