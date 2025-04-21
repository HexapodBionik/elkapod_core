import numpy as np
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from std_msgs.msg import Float64MultiArray
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

class GaitType(Enum):
    WAVE = 0
    RIPPLE = 1
    TRIPOID = 2


class ElkapodGait(Node):
    def __init__(self):
        super().__init__(node_name="elkapod_gait")

        self._gait_type = GaitType.TRIPOID
        self._state = State.IDLE

        # Leg parameters
        self._rotations_from_base_link = np.array([0.63973287, -0.63973287, np.pi/2, -np.pi/2, 2.38414364, -2.38414364])

        self._translations = [translate(0.17841, 0.13276, -0.03), translate(0.17841, -0.13276, -0.03),
                                translate(0.0138, 0.1643, -0.03), translate(0.0138, -0.1643, -0.03),
                                translate(-0.15903, 0.15038, -0.03), translate(-0.15903, -0.15038, -0.03)
        ]
        

        self._current_velocity = np.array([0.0, 0.0, 0.0])  # Linear vx, vy and angular wz
        self._leg_clock_freq = 50                   # in Hz
        self._cycle_time = 2.0                      # in seconds
        self._min_cycle_time = 0.5
        self._step_length = 0.1
        self._step_height = 0.05
        self._base_height = 0.15
        self._current_vel = 0.0

        self._phase_offset = None
        self._phase_offset = None

        self._leg_clock = [0 for _ in range(6)]
        self._leg_phase = [0 for _ in range(6)]

        self._base_traj = ElkapodLegPathBase(self._step_length, self._step_height)
        self._base_traj.init()

        self._leg_clock_timer = self.create_timer(1/self._leg_clock_freq, self.legClockCallback, callback_group=ReentrantCallbackGroup(), autostart=False)



        self._velocity_sub = self.create_subscription(Twist, "/cmd_vel", qos_profile=10, callback=self.velocityTopicCallback, callback_group=ReentrantCallbackGroup())

        self._leg_clock_pub = self.create_publisher(Float64MultiArray, "leg_clock", qos_profile=qos, callback_group=ReentrantCallbackGroup())
        self._leg_clock_thresh_pub = self.create_publisher(Float64MultiArray, "leg_clock_thresh", qos_profile=qos, callback_group=ReentrantCallbackGroup())
        self._leg_signal = self.create_publisher(Float64MultiArray, "elkapod_leg_positions", qos_profile=qos, callback_group=ReentrantCallbackGroup())

        self._init_time = self.get_clock().now().nanoseconds

    def init(self):
        self._base_traj.init()
        self.changeGait(self._gait_type)
        self._leg_clock_timer.reset()

    def velocityTopicCallback(self, msg: Twist):
        self._current_velocity[0] = msg.linear.x
        self._current_velocity[1] = msg.linear.y
        self._current_velocity[2] = msg.angular.z

        self.get_logger().info(f"New velocity command received: Vx: {msg.linear.x}\tVy: {msg.linear.y} Wz: {msg.angular.z}")

        # TODO check vel calc
        vel = np.linalg.norm(self._current_velocity[:2])
        if vel != 0 and vel != self._current_vel:
            T = self._step_length / vel
            if self._min_cycle_time > T:
                self.get_logger().warning(f"Max speed {self._step_length/self._min_cycle_time} m/s reached! Cannot set required speed - {vel} m/s")
            else:
                self._current_vel = vel
                self._cycle_time = T
                self.changeGait(self._gait_type)

        if not self._current_velocity.any() and self._state == State.WALKING:
            self.get_logger().info("Going to IDLE state")
            self._state = State.IDLE
        elif self._current_velocity.any() and self._state == State.IDLE:
            self.get_logger().info("Going to WALKING state")
            self._init_time = self.get_clock().now().nanoseconds
            self._state = State.WALKING

    

    def changeGait(self, gait_type: GaitType):
        if gait_type == GaitType.TRIPOID:
            self._swing_percentage = 0.5
            self._phase_offset = [0, self._cycle_time/2, self._cycle_time/2, 0, 0, self._cycle_time/2]
        elif gait_type == GaitType.WAVE:
            self._swing_percentage = 1/6
            self._phase_offset = [i*self._cycle_time/6 for i in range(6)]
        
    def clockFunction(self, t: float, cycle_time: float, phase_shift: float):
        T = cycle_time
        t_mod = (t + phase_shift) % T

        phase = 0
        s = 0

        if t_mod < T * self._swing_percentage:      # swing
            phase = 1
            s = t_mod / (T * self._swing_percentage)                          
        else:                                       # stance
            phase = 0
            s = (t_mod - T * self._swing_percentage) / (T - T * self._swing_percentage)
    
        return phase, s

    def legClockCallback(self):
        if self._state == State.IDLE:
            for leg_nb in range(6):
                self._leg_phase[leg_nb], self._leg_clock[leg_nb] = 0.0, 0.0
        else:
            for leg_nb in range(6):
                current_time = self.get_clock().now().nanoseconds
                elapsed_time_sec = (current_time - self._init_time) / 1e9
                self._leg_phase[leg_nb], self._leg_clock[leg_nb] = self.clockFunction(elapsed_time_sec, self._cycle_time, self._phase_offset[leg_nb])

        msg = Float64MultiArray()
        msg.data = self._leg_clock

        self._leg_clock_pub.publish(msg)

        msg2 = Float64MultiArray()
        msg2.data = self._leg_phase
        self._leg_clock_thresh_pub.publish(msg2)

        msg3 = Float64MultiArray()
        msg3.data = [0 for _ in range(18)]
        for leg_nb in range(6):

            if self._state == State.WALKING:
                p = self._base_traj(self._leg_clock[leg_nb], self._leg_phase[leg_nb])
                p = rotZ(np.arctan2(self._current_velocity[1], self._current_velocity[0])) @ p
            else:
                p = np.array([0.0, 0.0, 0.0])

            
            #p = p + self._translations[leg_nb]
            p = rotZ(self._rotations_from_base_link[leg_nb]).transpose() @ p

            # Add base height and leg spacing
            p = p + np.array([0.2, 0.0, -self._base_height])

            msg3.data[leg_nb*3] = p[0]
            msg3.data[leg_nb*3+1] = p[1]
            msg3.data[leg_nb*3+2] = p[2]

        self._leg_signal.publish(msg3)
    

def main(args=None):
    rclpy.init(args=args)
    node = ElkapodGait()
    node.init()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
