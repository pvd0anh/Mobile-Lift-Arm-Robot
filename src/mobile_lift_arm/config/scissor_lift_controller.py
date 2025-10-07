#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import math
import time

class ScissorLiftController(Node):
    def __init__(self):
        super().__init__('scissor_lift_controller')

        # Publishers
        self.hydraulic_cmd_pub = self.create_publisher(
            Float64, '/hydraulic_position_controller/command', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

        # Parameters
        self.max_height = 2.5  # Maximum lift height in meters
        self.min_height = 0.3  # Minimum lift height in meters
        self.current_position = 0.0
        self.target_position = 0.0
        self.current_height = 0.0

        # Control parameters
        self.position_tolerance = 0.01  # Position tolerance in meters
        self.max_velocity = 0.2  # Maximum velocity in m/s

        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_callback)

        self.get_logger().info('Scissor lift controller initialized')

    def joint_state_callback(self, msg):
        """Callback for joint state updates"""
        try:
            # Find hydraulic actuator position
            hydraulic_idx = msg.name.index('hydraulic_actuator')
            self.current_position = msg.position[hydraulic_idx]

            # Calculate platform height from hydraulic position
            self.current_height = self.calculate_height_from_position(self.current_position)

        except (ValueError, IndexError):
            pass

    def calculate_height_from_position(self, hydraulic_pos):
        """Calculate platform height from hydraulic cylinder position"""
        # Simplified kinematic calculation
        # In reality, this would involve more complex trigonometry
        base_height = 0.3
        scissor_length = 1.2

        # Approximate relationship between hydraulic extension and height
        # This is a simplified model - real implementation would need proper kinematics
        angle = math.atan2(hydraulic_pos + 0.8, 1.0)  # Base geometry
        height = base_height + scissor_length * math.sin(angle)

        return max(self.min_height, min(self.max_height, height))

    def calculate_position_from_height(self, target_height):
        """Calculate required hydraulic position for target height"""
        base_height = 0.3
        scissor_length = 1.2

        # Clamp target height
        target_height = max(self.min_height, min(self.max_height, target_height))

        # Reverse calculation
        required_angle = math.asin((target_height - base_height) / scissor_length)
        hydraulic_pos = 1.0 * math.tan(required_angle) - 0.8

        return max(0.0, min(0.6, hydraulic_pos))  # Clamp to joint limits

    def set_target_height(self, height):
        """Set target height for the platform"""
        self.target_position = self.calculate_position_from_height(height)
        self.get_logger().info(f'Target height set to {height:.2f}m (position: {self.target_position:.3f}m)')

    def control_callback(self):
        """Main control loop"""
        if abs(self.current_position - self.target_position) > self.position_tolerance:
            # Simple position control
            msg = Float64()
            msg.data = self.target_position
            self.hydraulic_cmd_pub.publish(msg)

    def move_to_height(self, height):
        """Move platform to specified height"""
        self.set_target_height(height)

        # Wait for movement to complete
        start_time = time.time()
        timeout = 30.0  # 30 second timeout

        while (abs(self.current_position - self.target_position) > self.position_tolerance 
               and time.time() - start_time < timeout):
            rclpy.spin_once(self, timeout_sec=0.1)

        if time.time() - start_time >= timeout:
            self.get_logger().warn('Movement timeout reached')
            return False
        else:
            self.get_logger().info(f'Reached target height: {self.current_height:.2f}m')
            return True

    def emergency_stop(self):
        """Emergency stop - hold current position"""
        self.target_position = self.current_position
        self.get_logger().warn('Emergency stop activated')

    def get_status(self):
        """Get current status information"""
        return {
            'current_height': self.current_height,
            'target_height': self.calculate_height_from_position(self.target_position),
            'hydraulic_position': self.current_position,
            'target_position': self.target_position,
            'is_moving': abs(self.current_position - self.target_position) > self.position_tolerance
        }

class ScissorLiftSequenceController(ScissorLiftController):
    """Extended controller for running predefined sequences"""

    def __init__(self):
        super().__init__()

        # Predefined sequences
        self.sequences = {
            'demo': [0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5],
            'test': [0.3, 0.8, 1.2, 0.8, 0.3],
            'inspection': [1.8, 1.8, 1.8, 0.3]  # High position for inspection
        }

        self.get_logger().info('Sequence controller ready')

    def run_sequence(self, sequence_name, delay=2.0):
        """Run a predefined sequence"""
        if sequence_name not in self.sequences:
            self.get_logger().error(f'Unknown sequence: {sequence_name}')
            return False

        sequence = self.sequences[sequence_name]
        self.get_logger().info(f'Running sequence: {sequence_name}')

        for i, height in enumerate(sequence):
            self.get_logger().info(f'Step {i+1}/{len(sequence)}: Moving to {height}m')

            if not self.move_to_height(height):
                self.get_logger().error(f'Failed to reach height {height}m')
                return False

            # Wait between steps
            if i < len(sequence) - 1:  # Don't wait after last step
                time.sleep(delay)

        self.get_logger().info('Sequence completed successfully')
        return True

def main(args=None):
    rclpy.init(args=args)

    # Create controller
    controller = ScissorLiftSequenceController()

    try:
        # Run a demo sequence
        controller.get_logger().info('Starting scissor lift demo...')
        controller.run_sequence('demo', delay=3.0)

        # Keep node alive
        rclpy.spin(controller)

    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down scissor lift controller')
        controller.emergency_stop()

    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
