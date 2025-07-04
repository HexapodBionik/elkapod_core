#
# Created by Piotr Patek.
#
# Copyright (c) 2025.
# Elkapod Bionik, Warsaw University of Technology. All rights reserved.
#

import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import Float64MultiArray, Float64, Int32
from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response
from geometry_msgs.msg import Twist

from .elkapod_leg_path import ElkapodLegPathBase
from .utils import rotZ, translate


qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class State(Enum):
    IDLE = 0
    WALKING = 1
    DISABLED = 2

class GaitType(Enum):
    WAVE = 0
    TRIPOID = 1


class ElkapodGait(Node):
    def __init__(self):
        super().__init__(node_name="elkapod_gait")
        self.declare_parameter("trajectory_frequency", value=50.0, descriptor=ParameterDescriptor(description="Leg trajectory frequency in Hz"))
        self.declare_parameter("min_cycle_time", value=0.5, descriptor=ParameterDescriptor(description="Minimal gait cycle time in seconds"))

        self.declare_parameter("leg_spacing", value=0.175, descriptor=ParameterDescriptor(description="Leg spacing from base in meters"))
        self.declare_parameter("step_length", value=0.2, descriptor=ParameterDescriptor(description="Length of single step in meters (swing and stance phases)"))
        self.declare_parameter("step_height", value=0.1, descriptor=ParameterDescriptor(description="Height of single step in meters (swing phase)"))
        self.declare_parameter("phase_lag", value=0.03, descriptor=ParameterDescriptor(description="Additional lag on start of swing phase which helps to stabilizate the body, value is a percentage of cycle time"))

        # Leg parameters
        self._rotations_from_base_link = np.array([0.63973287, -0.63973287, np.pi/2, -np.pi/2, 2.38414364, -2.38414364])

        self._translations = [translate(0.17841, 0.13276, -0.03), translate(0.17841, -0.13276, -0.03),
                                translate(0.0138, 0.1643, -0.03), translate(0.0138, -0.1643, -0.03),
                                translate(-0.15903, 0.15038, -0.03), translate(-0.15903, -0.15038, -0.03)
        ]

        # Main variables
        self._gait_type = GaitType.TRIPOID
        self._state = State.DISABLED

        self._current_vel = np.array([0.0, 0.0, 0.0])  # Linear vx, vy and angular wz
        self._current_vel_scalar = 0.0
        self._current_angular_velocity = 0.0
        self._current_base_direction = 0.0
        self._current_velocity = np.array([[0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]])

        self._last_leg_position = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

        self._cycle_time = 2.0                      # in seconds
        self._base_height = 0.17
        self._set_base_height = 0.17

        self._phase_offset = None
        self._phase_offset = None

        self._leg_clock = [0 for _ in range(6)]
        self._leg_phase = [0 for _ in range(6)]
        self._leg_phase_shift = [0 for _ in range(6)]
        self._leg_start_phase_shift = 0.0

        self._trajectory_frequency = self.get_parameter("trajectory_frequency").get_parameter_value().double_value
        self._min_cycle_time = self.get_parameter("min_cycle_time").get_parameter_value().double_value
        self._step_length = self.get_parameter("step_length").get_parameter_value().double_value
        self._step_height = self.get_parameter("step_height").get_parameter_value().double_value
        self._leg_spacing = self.get_parameter("leg_spacing").get_parameter_value().double_value
        self._phase_lag = self.get_parameter("phase_lag").get_parameter_value().double_value

        self._base_traj = ElkapodLegPathBase(self._step_length, self._step_height)
        self._base_traj.init()

        self._leg_clock_timer = self.create_timer(1/self._trajectory_frequency, self.legClockCallback, autostart=False)

        self._velocity_sub = self.create_subscription(Twist, "/cmd_vel", qos_profile=10, callback=self.velocityTopicCallback, callback_group=ReentrantCallbackGroup())

        # TODO
        # In final version of gait generation && execution node those topic subscriptions should be changed into services with custom messages, for now this is left like this
        # for simplicity.
        self._parameters_sub = self.create_subscription(Float64, "/cmd_base_param", qos_profile=10, callback=self._paramTopicCallback, callback_group=ReentrantCallbackGroup())
        self._gait_type_sub = self.create_subscription(Int32, "/cmd_gait_type", qos_profile=10, callback=self._paramGaitTypeTopicCallabck, callback_group=ReentrantCallbackGroup())

        self._enable_service = self.create_service(Trigger, "/gait_gen_enable", self._enable_service_callback)
        self._disable_service = self.create_service(Trigger, "/gait_gen_disable", self._disable_service_callback)

        self._leg_signal = self.create_publisher(Float64MultiArray, "/elkapod_leg_positions", qos_profile=10, callback_group=ReentrantCallbackGroup())
        self._leg_phase_signal = self.create_publisher(Float64MultiArray, "leg_phase_signal", qos_profile=10, callback_group=ReentrantCallbackGroup())


        self.get_logger().info("Elkapog gait generator prototype started! Use service to activate the node!")
        self._init_time = self.get_clock().now().nanoseconds

    def init(self):
        self._base_traj.init()
        self.changeGait(self._gait_type)
        self._leg_clock_timer.reset()
        self.get_logger().info("Elkapod gait generator prototype initialized!")
        self._state = State.IDLE

    def deinit(self):
        self._leg_clock_timer.cancel()
        self._state = State.DISABLED


    def _enable_service_callback(self, request: Trigger_Request, response: Trigger_Response):
        if self._state == State.DISABLED:
            self.init()
            response.success = True
        else:
            response.success = False
            response.message = "Couldn't enable gait gen because it's already enabled and working"
        return response

    def _disable_service_callback(self, request: Trigger_Request, response: Trigger_Response):
        if self._state == State.DISABLED:
            response.success = False
            response.message = "Couldn't disable gait gen because it's already disabled"
        elif self._state == State.WALKING:
            response.success = False
            response.message = "Couldn't disable gait gen when robot is actively walking"
        elif self._state == State.IDLE:
            response.success = True
            self.deinit()

        return response

    def _paramGaitTypeTopicCallabck(self, msg: Int32):
        if msg.data == 0 and self._state == State.IDLE:
            self.get_logger().info("Gait set to WAVE")
            self._gait_type = GaitType.WAVE
            self.changeGait(self._gait_type)
        elif msg.data == 1 and self._state == State.IDLE:
            self._gait_type = GaitType.TRIPOID
            self.changeGait(self._gait_type)
            self.get_logger().info("Gait set to TRIPOD")

    def _paramTopicCallback(self, msg: Float64):
        if 0.1 < msg.data < 0.2:
            self._set_base_height = msg.data
        else:
            self.get_logger().warning("Couldn't set new base height goal - value out of allowed range")

    def velocityTopicCallback(self, msg: Twist):
        self._current_vel[0] = msg.linear.x
        self._current_vel[1] = msg.linear.y
        self._current_vel[2] = msg.angular.z

        vel = np.linalg.norm(self._current_vel[:2])
        direction = np.arctan2(self._current_vel[1], self._current_vel[0])

        # For now is is possible only to set either linear or angular speed 
        if vel != 0 and (vel != self._current_vel_scalar or direction != self._current_base_direction):
            T = self._step_length / vel
            if self._min_cycle_time > T:
                self.get_logger().warning(f"Max speed {self._step_length/self._min_cycle_time} m/s reached! Cannot set required speed - {vel} m/s")
            else:
                self.get_logger().info(f"New velocity command received: Vx: {msg.linear.x}\tVy: {msg.linear.y} Wz: {msg.angular.z}")
                self._current_base_direction = direction
                for leg_nb in range(6):
                    self._current_velocity[leg_nb] = np.array([self._current_vel[0], self._current_vel[1]])
                self._current_vel_scalar = vel
                self._current_angular_velocity = 0.0
                self._cycle_time = T
                self._leg_phase_shift = [0 for _ in range(6)]
                self.changeGait(self._gait_type)
                
        elif self._current_vel[2] != 0 and not np.isclose(self._current_angular_velocity, self._current_vel[2]):
            for leg_nb in range(6):
                    self._current_velocity[leg_nb] = np.array([0.0, 0.0])
            self._current_vel_scalar = 0
            self._current_angular_velocity = self._current_vel[2]
            # For now R ~ self._leg_spacing + 0.15 (half body width) approx

            R = self._leg_spacing + 0.22
            self._cycle_time = self._step_length / (0.5 * R * abs(self._current_vel[2]))
            self._leg_phase_shift = [0 for _ in range(6)]
            self.changeGait(self._gait_type)

        if not self._current_vel.any() and self._state == State.WALKING:
            self.get_logger().info("Going to IDLE state")
            self._state = State.IDLE
        elif self._current_vel.any() and self._state == State.IDLE:
            self.get_logger().info("Going to WALKING state")
            self._init_time = self.get_clock().now().nanoseconds
            self._state = State.WALKING
            self._leg_phase_shift = [0 for _ in range(6)]
    

    def changeGait(self, gait_type: GaitType):
        if gait_type == GaitType.TRIPOID:
            self._swing_percentage = 0.5

            self._phase_offset = [0, self._cycle_time/2, self._cycle_time/2, 0, 0, self._cycle_time/2]
        elif gait_type == GaitType.WAVE:
            self._swing_percentage = 1/6
            self._phase_offset = [i*self._cycle_time/6 for i in range(6)]
        
    def clockFunction(self, t: float, cycle_time: float, phase_shift: float, leg_nb: int):
        T = cycle_time
        if not np.isclose(self._leg_phase_shift[leg_nb], phase_shift, atol=1e-3):
            if self._leg_phase_shift[leg_nb] < phase_shift:
                self._leg_phase_shift[leg_nb] += 0.05 * phase_shift
            else:
                self._leg_phase_shift[leg_nb] -= 0.05 * phase_shift

        t_mod = (t + 3*T/4 + self._leg_phase_shift[leg_nb]) % T
        
        if t_mod < T*self._phase_lag:                                       # swing lag phase
            self._leg_phase[leg_nb] = 1
            self._leg_clock[leg_nb] = 0.0
        elif T*self._phase_lag < t_mod < T * self._swing_percentage:        # swing
            self._leg_phase[leg_nb] = 1
            self._leg_clock[leg_nb] = (t_mod - T * self._phase_lag) / (T * self._swing_percentage - T * self._phase_lag)                  
        else:                                                               # stance
            self._leg_phase[leg_nb] = 0
            self._leg_clock[leg_nb] = (t_mod - T * self._swing_percentage) / (T - T * self._swing_percentage)

    def legClockCallback(self):
        msg_phase = Float64MultiArray()
        msg_phase.data = [0 for _ in range(6)]

        if self._state == State.IDLE:
            for leg_nb in range(6):
                self._leg_phase[leg_nb], self._leg_clock[leg_nb] = 0.0, 0.0
        else:
            current_time = self.get_clock().now().nanoseconds
            elapsed_time_sec = (current_time - self._init_time) / 1e9
            for leg_nb in range(6):
                self.clockFunction(elapsed_time_sec, self._cycle_time, self._phase_offset[leg_nb], leg_nb)
                msg_phase.data[leg_nb] = self._leg_phase[leg_nb]

        self._leg_phase_signal.publish(msg_phase)

        msg3 = Float64MultiArray()
        msg3.data = [0 for _ in range(18)]

        # string = f"Cycle time: {self._cycle_time:.3f}s Offsets: "
        # for leg_nb in range(6):
        #     string += f" {self._leg_phase_shift[leg_nb]:.3f}/{self._phase_offset[leg_nb]:.3f}\t"
        # self.get_logger().info(string)


        if not np.isclose(self._base_height, self._set_base_height, atol=1e-3):
                if self._base_height < self._set_base_height:
                    self._base_height += 0.005
                else:
                    self._base_height -= 0.005

        for leg_nb in range(6):
            if self._state == State.WALKING:
                p = self._base_traj(self._leg_clock[leg_nb], self._leg_phase[leg_nb])

                omega = self._current_angular_velocity
                angular_part = omega * np.array([-self._last_leg_position[leg_nb][1], self._last_leg_position[leg_nb][0]])

                vel = self._current_velocity[leg_nb] + angular_part   

                p = rotZ(np.arctan2(vel[1], vel[0])) @ p
            else:
                p = np.array([0.0, 0.0, 0.0])

            
            p = rotZ(-self._rotations_from_base_link[leg_nb]) @ p

            # Add base height and leg spacing
            p = p + np.array([self._leg_spacing, 0.0, -self._base_height])

            
            rot = self._rotations_from_base_link[leg_nb]
            trans = self._translations[leg_nb]
            L_B_H = np.array([
                [np.cos(rot), -np.sin(rot), 0, -(np.cos(rot) * self._leg_spacing + trans[0])],
                [np.sin(rot), np.cos(rot), 0, -(np.sin(rot) * self._leg_spacing + trans[1])],
                [0, 0, 1, self._base_height],
                [0, 0, 0, 1]
            ])

            p_homogeneus = np.array([p[0], p[1], p[2], 1])
            p_base_homogeneus = L_B_H @ p_homogeneus


            self._last_leg_position[leg_nb] = p_base_homogeneus[:3]

            msg3.data[leg_nb*3] = p[0]
            msg3.data[leg_nb*3+1] = p[1]
            msg3.data[leg_nb*3+2] = p[2]

        self._leg_signal.publish(msg3)
    

def main(args=None):
    rclpy.init(args=args)
    node = ElkapodGait()
    #node.init()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
