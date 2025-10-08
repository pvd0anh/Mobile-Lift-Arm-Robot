#!/usr/bin/env python3
"""ROS2 scissor lift controller.

This module defines a pair of classes for controlling a simulated
scissor‑lift.  The `ScissorLiftController` provides basic
position/height conversion routines and a simple control loop that
publishes commands on a ROS2 topic.  The `ScissorLiftSequenceController`
extends this with the ability to run predefined sequences of heights.

The original version of this code in the provided repository had
incorrect indentation and lived outside of a proper Python package.
This refactored version fixes the indentation, groups the code into
a package and exposes a ``main()`` function that can be used as a
console entry point via ``setup.py``.
"""

from __future__ import annotations

import math
import time
from typing import Dict, List

try:
    import rclpy  # type: ignore
    from rclpy.node import Node  # type: ignore
    from std_msgs.msg import Float64  # type: ignore
    from sensor_msgs.msg import JointState  # type: ignore
except ImportError:
    # When running outside of a ROS2 environment the rclpy package may
    # not be available.  We provide simple stubs so that the module
    # remains importable and syntax errors can be caught without ROS2.
    rclpy = None  # type: ignore
    Node = object  # type: ignore
    Float64 = object  # type: ignore
    JointState = object  # type: ignore


class ScissorLiftController(Node):  # type: ignore[misc]
    """Basic position controller for a scissor lift.

    Given a desired platform height this controller computes the
    corresponding hydraulic cylinder position and publishes it on a
    ROS2 topic.  It also keeps track of the current height by
    subscribing to joint state messages.  The controller uses a
    simple proportional position control loop with configurable
    tolerance and maximum velocity.
    """

    def __init__(self) -> None:
        # If ROS2 is not available we fall back to a base class that
        # does nothing.  This allows unit tests to import the module
        # without requiring a ROS2 installation.
        if rclpy is not None and isinstance(Node, type):
            super().__init__('scissor_lift_controller')

            # Publisher for the hydraulic actuator command
            self.hydraulic_cmd_pub = self.create_publisher(
                Float64, '/hydraulic_position_controller/command', 10)

            # Subscriber for joint state updates
            self.joint_state_sub = self.create_subscription(
                JointState, '/joint_states', self.joint_state_callback, 10)

            # Timer for the control loop (10 Hz)
            self.control_timer = self.create_timer(0.1, self.control_callback)

            # Log start up
            self.get_logger().info('Scissor lift controller initialized')
        else:
            # Without ROS2 the parent class does nothing so there is no
            # timer or publishers.  The attributes are still defined
            # to avoid attribute errors in unit tests.
            self.hydraulic_cmd_pub = None
            self.joint_state_sub = None
            self.control_timer = None

        # Height limits (in metres)
        self.max_height: float = 2.5
        self.min_height: float = 0.3

        # Current and target state
        self.current_position: float = 0.0
        self.target_position: float = 0.0
        self.current_height: float = 0.0

        # Control parameters
        self.position_tolerance: float = 0.01  # m
        self.max_velocity: float = 0.2  # m/s

    # ------------------------------------------------------------------
    # Callback and conversion helpers
    # ------------------------------------------------------------------
    def joint_state_callback(self, msg: JointState) -> None:
        """Update the current position and height from joint state messages."""
        try:
            # Find the index of the hydraulic actuator in the joint state
            hydraulic_idx = msg.name.index('hydraulic_actuator')
            self.current_position = float(msg.position[hydraulic_idx])
            self.current_height = self.calculate_height_from_position(
                self.current_position)
        except (ValueError, IndexError):
            # If the actuator is not found just ignore the message
            pass

    def calculate_height_from_position(self, hydraulic_pos: float) -> float:
        """Convert a hydraulic actuator position into a platform height.

        This uses a simplified kinematic model of the scissor mechanism.
        In a real system this would involve solving the geometry of the
        scissor mechanism using trigonometry and link lengths.
        """
        base_height = 0.3
        scissor_length = 1.2
        # Approximate the lifting angle based on the actuator extension
        angle = math.atan2(hydraulic_pos + 0.8, 1.0)
        height = base_height + scissor_length * math.sin(angle)
        return max(self.min_height, min(self.max_height, height))

    def calculate_position_from_height(self, target_height: float) -> float:
        """Convert a desired platform height into a hydraulic actuator position."""
        base_height = 0.3
        scissor_length = 1.2
        # Clamp the target height within limits
        target_height = max(self.min_height, min(self.max_height, target_height))
        # Invert the simple height model to get the actuator angle
        required_angle = math.asin((target_height - base_height) / scissor_length)
        hydraulic_pos = 1.0 * math.tan(required_angle) - 0.8
        # Clamp actuator travel to physical joint limits
        return max(0.0, min(0.6, hydraulic_pos))

    # ------------------------------------------------------------------
    # Control loop and movement helpers
    # ------------------------------------------------------------------
    def set_target_height(self, height: float) -> None:
        """Set the target platform height (updates target position)."""
        self.target_position = self.calculate_position_from_height(height)
        if rclpy is not None and isinstance(Node, type):
            self.get_logger().info(
                f'Target height set to {height:.2f}m '
                f'(position: {self.target_position:.3f}m)')

    def control_callback(self) -> None:
        """Periodic control loop that commands the actuator towards the target."""
        # Only act if we have a publisher and the error exceeds the tolerance
        if (self.hydraulic_cmd_pub is not None and
                abs(self.current_position - self.target_position) > self.position_tolerance):
            msg = Float64()  # type: ignore[call-arg]
            msg.data = float(self.target_position)  # type: ignore[attr-defined]
            self.hydraulic_cmd_pub.publish(msg)  # type: ignore[call-arg]

    def move_to_height(self, height: float, timeout: float = 30.0) -> bool:
        """Synchronously move the platform to a specific height.

        Returns True if the target was reached before timeout, otherwise False.
        """
        self.set_target_height(height)
        start_time = time.time()
        while (abs(self.current_position - self.target_position) > self.position_tolerance
               and time.time() - start_time < timeout):
            if rclpy is not None:
                rclpy.spin_once(self, timeout_sec=0.1)  # type: ignore[call-arg]
            else:
                time.sleep(0.1)
        if time.time() - start_time >= timeout:
            if rclpy is not None:
                self.get_logger().warn('Movement timeout reached')
            return False
        else:
            if rclpy is not None:
                self.get_logger().info(
                    f'Reached target height: {self.current_height:.2f}m')
            return True

    def emergency_stop(self) -> None:
        """Immediately stop movement by setting the target to the current position."""
        self.target_position = self.current_position
        if rclpy is not None:
            self.get_logger().warn('Emergency stop activated')

    def get_status(self) -> Dict[str, float]:
        """Return a dictionary of the current state for external monitoring."""
        return {
            'current_height': self.current_height,
            'target_height': self.calculate_height_from_position(self.target_position),
            'hydraulic_position': self.current_position,
            'target_position': self.target_position,
            'is_moving': abs(self.current_position - self.target_position) > self.position_tolerance
        }


class ScissorLiftSequenceController(ScissorLiftController):
    """Controller that can execute predefined sequences of heights."""

    def __init__(self) -> None:
        super().__init__()
        # Predefined sequences used for demos or testing
        self.sequences: Dict[str, List[float]] = {
            'demo': [0.5, 1.0, 1.5, 2.0, 1.5, 1.0, 0.5],
            'test': [0.3, 0.8, 1.2, 0.8, 0.3],
            'inspection': [1.8, 1.8, 1.8, 0.3],
        }
        if rclpy is not None:
            self.get_logger().info('Sequence controller ready')

    def run_sequence(self, sequence_name: str, delay: float = 2.0) -> bool:
        """Run a named sequence of heights.

        Each entry in the sequence is sent to ``move_to_height`` and the
        controller waits for the move to finish before proceeding to the
        next height.  A short delay is inserted between steps.
        Returns True if the entire sequence ran without timing out, otherwise
        False.
        """
        if sequence_name not in self.sequences:
            if rclpy is not None:
                self.get_logger().error(f'Unknown sequence: {sequence_name}')
            return False

        sequence = self.sequences[sequence_name]
        if rclpy is not None:
            self.get_logger().info(f'Running sequence: {sequence_name}')
        for i, height in enumerate(sequence):
            if rclpy is not None:
                self.get_logger().info(f'Step {i + 1}/{len(sequence)}: Moving to {height}m')
            if not self.move_to_height(height):
                if rclpy is not None:
                    self.get_logger().error(f'Failed to reach height {height}m')
                return False
            # Delay between steps except after the last one
            if i < len(sequence) - 1:
                time.sleep(delay)
        if rclpy is not None:
            self.get_logger().info('Sequence completed successfully')
        return True


def main(args: List[str] | None = None) -> None:
    """Entry point for console_scripts.

    This function initialises the ROS2 rclpy library (if available),
    creates a sequence controller and runs a default demo sequence.
    Afterwards it keeps the node alive until interrupted.
    """
    if rclpy is None:
        raise RuntimeError(
            'rclpy is not available – this script must be run inside a ROS2 environment')

    rclpy.init(args=args)
    controller = ScissorLiftSequenceController()
    try:
        controller.get_logger().info('Starting scissor lift demo...')
        controller.run_sequence('demo', delay=3.0)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down scissor lift controller')
        controller.emergency_stop()
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()