#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and a disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Jaeyoung Lim"
__contact__ = "jalim@ethz.ch"

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from enum import Enum

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleAttitude
from std_srvs.srv import Trigger


class VehicleState(Enum):
    """Simple state machine for the vehicle's behavior."""

    IDLE = 0
    HOVERING = 1
    CIRCLING = 2
    LANDING = 3


class OffboardControl(Node):

    def __init__(self):
        super().__init__("hover_circle_land_node")

        # --- QoS Profiles ---
        qos_profile_control = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # --- Publishers & Subscribers ---
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile_control
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile_control
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sub,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            "fmu/out/vehicle_attitude",
            self.attitude_callback,
            qos_profile_sub,
        )

        # --- Services ---
        self.land_service = self.create_service(Trigger, "~/land", self.land_callback)

        # --- Parameters ---
        param_defaults = {
            "radius": 5.0,
            "omega": 0.5,
            "altitude": 5.0,
            "hover_duration": 5.0,
            "landing_descent_rate": 0.5,
        }
        self.get_logger().info("--- Loading and Getting Parameters ---")
        for name, default_value in param_defaults.items():
            self.declare_parameter(name, default_value)

            param_value = self.get_parameter(name).value
            setattr(self, name, param_value)
            self.get_logger().info(f"  - {name}: {param_value}")

        # --- State and Timing ---
        self.state = VehicleState.IDLE
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.theta = 0.0
        self.hover_start_time = None
        self.current_yaw = 0.0
        self.initial_yaw = 0.0
        self.landing_position = [0.0, 0.0, 0.0]
        self.landing_yaw = 0.0

        timer_period = 0.02  # 50Hz
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.get_logger().info(
            "Node initialized. Waiting for vehicle to be armed and in offboard mode."
        )

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def attitude_callback(self, msg):
        self.current_yaw = self.quaternion_to_euler_yaw(msg.q)

    def quaternion_to_euler_yaw(self, q):
        t3 = +2.0 * (q[0] * q[3] + q[1] * q[2])
        t4 = +1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        return np.arctan2(t3, t4)

    def land_callback(self, request, response):
        """Service callback to initiate landing from offboard mode."""
        if self.state == VehicleState.CIRCLING or self.state == VehicleState.HOVERING:
            # --- Capture current position and yaw to land there ---
            self.landing_position = [
                self.radius * np.cos(self.theta),
                self.radius * np.sin(self.theta),
                -self.altitude,
            ]
            self.landing_yaw = self.theta + np.pi / 2.0

            # If we are in the initial hover, the position is at origin
            if self.state == VehicleState.HOVERING:
                self.landing_position = [0.0, 0.0, -self.altitude]
                # Use last commanded yaw from gradual turn
                hover_elapsed_time = (
                    self.get_clock().now() - self.hover_start_time
                ).nanoseconds / 1e9
                t = min(hover_elapsed_time / self.hover_duration, 1.0)
                delta_yaw = -self.initial_yaw  # Target is 0.0
                if delta_yaw > np.pi:
                    delta_yaw -= 2 * np.pi
                elif delta_yaw < -np.pi:
                    delta_yaw += 2 * np.pi
                self.landing_yaw = self.initial_yaw + delta_yaw * t

            self.state = VehicleState.LANDING
            self.get_logger().info(
                f"Offboard landing initiated at position {self.landing_position[0]:.2f}, "
                f"{self.landing_position[1]:.2f} with yaw {np.rad2deg(self.landing_yaw):.1f} deg"
            )
            response.success = True
            response.message = "Offboard landing sequence initiated."
        else:
            response.success = False
            response.message = f"Cannot land from current state: {self.state.name}"
            self.get_logger().warn(response.message)
        return response

    def publish_hover_setpoint(self, yaw):
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.altitude]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)

    def publish_circling_setpoint(self):
        self.theta += self.omega * self.dt
        msg = TrajectorySetpoint()
        msg.position = [
            self.radius * np.cos(self.theta),
            self.radius * np.sin(self.theta),
            -self.altitude,
        ]
        msg.yaw = self.theta + np.pi / 2.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)

    def publish_landing_setpoint(self):
        """Commands the drone to descend at its captured landing position."""
        # Decrease the Z component of the landing position over time
        self.landing_position[2] += self.landing_descent_rate * self.dt
        # Make sure we don't command to go through the ground
        self.landing_position[2] = min(
            self.landing_position[2], 0.5
        )  # Target slightly above ground

        msg = TrajectorySetpoint()
        msg.position = self.landing_position
        msg.yaw = self.landing_yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)

    def cmdloop_callback(self):
        offboard_msg = OffboardControlMode(
            position=True,
            velocity=False,
            acceleration=False,
            attitude=False,
            body_rate=False,
            timestamp=int(self.get_clock().now().nanoseconds / 1000),
        )
        self.publisher_offboard_mode.publish(offboard_msg)

        is_armed = self.arming_state == VehicleStatus.ARMING_STATE_ARMED
        is_offboard = self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

        if self.state == VehicleState.IDLE:
            if is_armed and is_offboard:
                self.get_logger().info(
                    "Takeoff conditions met. Starting gradual turn to North."
                )
                self.state = VehicleState.HOVERING
                self.hover_start_time = self.get_clock().now()
                self.initial_yaw = self.current_yaw

        elif self.state == VehicleState.HOVERING:
            hover_elapsed_time = (
                self.get_clock().now() - self.hover_start_time
            ).nanoseconds / 1e9
            t = min(hover_elapsed_time / self.hover_duration, 1.0)
            target_yaw = 0.0
            delta_yaw = target_yaw - self.initial_yaw
            if delta_yaw > np.pi:
                delta_yaw -= 2 * np.pi
            elif delta_yaw < -np.pi:
                delta_yaw += 2 * np.pi
            interpolated_yaw = self.initial_yaw + delta_yaw * t
            self.publish_hover_setpoint(yaw=interpolated_yaw)

            self.get_logger().info(
                f"Hovering and turning... Commanded Yaw: {np.rad2deg(interpolated_yaw):.1f} deg"
            )

            if t >= 1.0:
                self.get_logger().info(
                    "Hover and turn complete. Switching to CIRCLING."
                )
                self.state = VehicleState.CIRCLING

        elif self.state == VehicleState.CIRCLING:
            self.publish_circling_setpoint()
            self.get_logger().info(
                f"Circling... Angle: {np.rad2deg(self.theta):.1f} degrees"
            )

        elif self.state == VehicleState.LANDING:
            self.publish_landing_setpoint()
            self.get_logger().info(
                f"Landing... Target Alt: {-self.landing_position[2]:.2f} m"
            )

            if not is_armed:
                self.get_logger().info(
                    "Landing complete. Vehicle has disarmed. Returning to IDLE."
                )
                self.state = VehicleState.IDLE


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
