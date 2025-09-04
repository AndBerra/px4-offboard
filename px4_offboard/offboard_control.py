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

        # --- QoS Profiles for Control Data ---
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

        # --- Publishers ---
        self.publisher_offboard_mode = self.create_publisher(
            OffboardControlMode, "fmu/in/offboard_control_mode", qos_profile_control
        )
        self.publisher_trajectory = self.create_publisher(
            TrajectorySetpoint, "fmu/in/trajectory_setpoint", qos_profile_control
        )
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, "fmu/in/vehicle_command", qos_profile_control
        )

        # --- Subscribers ---
        self.status_sub = self.create_subscription(
            VehicleStatus,
            "fmu/out/vehicle_status",
            self.vehicle_status_callback,
            qos_profile_sub,
        )

        # --- Services ---
        self.land_service = self.create_service(Trigger, "~/land", self.land_callback)

        # --- Parameters ---
        self.declare_parameter("radius", 5.0)
        self.declare_parameter("omega", 0.5)  # rad/s
        self.declare_parameter("altitude", 5.0)
        self.declare_parameter("hover_duration", 5.0)  # seconds
        self.radius = self.get_parameter("radius").value
        self.omega = self.get_parameter("omega").value
        self.altitude = self.get_parameter("altitude").value
        self.hover_duration = self.get_parameter("hover_duration").value

        # --- State and Timing ---
        self.state = VehicleState.IDLE
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.theta = 0.0
        self.hover_start_time = None

        timer_period = 0.02  # 50Hz
        self.dt = timer_period
        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
        self.get_logger().info(
            "Node initialized. Waiting for vehicle to be armed and in offboard mode."
        )

    def vehicle_status_callback(self, msg):
        self.get_logger().debug(
            f"Vehicle Status Update: Nav State={msg.nav_state}, Arming State={msg.arming_state}"
        )
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def land_callback(self, request, response):
        """Service callback to initiate landing."""
        if self.state == VehicleState.CIRCLING or self.state == VehicleState.HOVERING:
            self.state = VehicleState.LANDING
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Land command issued.")
            response.success = True
            response.message = "Landing sequence initiated."
        else:
            response.success = False
            response.message = f"Cannot land from current state: {self.state.name}"
            self.get_logger().warn(response.message)
        return response

    def publish_vehicle_command(self, command, **params):
        """Publish a vehicle command."""
        msg = VehicleCommand(
            timestamp=int(self.get_clock().now().nanoseconds / 1000),
            param1=params.get("param1", 0.0),
            param2=params.get("param2", 0.0),
            command=command,
            target_system=1,
            target_component=1,
            source_system=1,
            source_component=1,
            from_external=True,
        )
        self.vehicle_command_publisher.publish(msg)

    def publish_hover_setpoint(self):
        """Publishes a setpoint to hover at the origin."""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, -self.altitude]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)

    def publish_circling_setpoint(self):
        """Publishes a setpoint to fly in a circle."""
        msg = TrajectorySetpoint()
        msg.position = [
            self.radius * np.cos(self.theta),
            self.radius * np.sin(self.theta),
            -self.altitude,
        ]
        msg.yaw = self.theta + np.pi / 2.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.publisher_trajectory.publish(msg)
        self.theta += self.omega * self.dt

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
                self.get_logger().info("Takeoff conditions met. Switching to HOVERING.")
                self.state = VehicleState.HOVERING
                self.hover_start_time = self.get_clock().now()
            else:
                if self.get_clock().now().nanoseconds % (1 * 1e9) < (self.dt * 1e9):
                    self.get_logger().warn(
                        f"Waiting for arm and offboard mode. "
                        f"Is Armed: {is_armed}, Is Offboard: {is_offboard}"
                    )

        elif self.state == VehicleState.HOVERING:
            self.publish_hover_setpoint()
            hover_elapsed_time = (
                self.get_clock().now() - self.hover_start_time
            ).nanoseconds / 1e9
            self.get_logger().info(
                f"Hovering... {hover_elapsed_time:.1f}s / {self.hover_duration}s"
            )

            if hover_elapsed_time >= self.hover_duration:
                self.get_logger().info("Hover complete. Switching to CIRCLING.")
                self.state = VehicleState.CIRCLING

        elif self.state == VehicleState.CIRCLING:
            self.publish_circling_setpoint()
            self.get_logger().info(
                f"Circling... Angle: {np.rad2deg(self.theta):.1f} degrees"
            )

        elif self.state == VehicleState.LANDING:
            self.get_logger().info("Landing in progress...")
            if not is_armed:
                self.get_logger().info(
                    "Landing complete. Vehicle has disarmed. Returning to IDLE."
                )
                self.state = VehicleState.IDLE

        else:
            self.get_logger().error("Reached an unknown state!")


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
