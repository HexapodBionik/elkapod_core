import numpy as np
from rclpy.node import Node
from elkapod_msgs.msg import LegFrames

class Leg:
    """! Leg of Elkapod robot.

    Provides interface for MegaLynx robot.
    """

    def __init__(self, s1, s2, s3, transform):
        """
        @param s1: Servo 1 instance
        @param s2: Servo 2 instance
        @param s3: Servo 3 instance
        @param transform: Coordinate system transform to be applied
        """
        self._s1 = s1
        self._s2 = s2
        self._s3 = s3

        self.transform = transform

    def set_angles(self, q: np.ndarray) -> None:
        q = np.deg2rad(q)
        q = self.transform(q)

        self._s1.setPosition(q[0])
        self._s2.setPosition(q[1])
        self._s3.setPosition(q[2])


class ElkapodCommServer(Node):
    def __init__(self, robot):
        super().__init__("elkapod_comm_server")

        # Setup elkapod simulation model
        self._robot = robot
        servo11 = self._robot.getDevice("Servo11")
        servo12 = self._robot.getDevice("Servo12")
        servo13 = self._robot.getDevice("Servo13")
        servo21 = self._robot.getDevice("Servo21")
        servo22 = self._robot.getDevice("Servo22")
        servo23 = self._robot.getDevice("Servo23")
        servo31 = self._robot.getDevice("Servo31")
        servo32 = self._robot.getDevice("Servo32")
        servo33 = self._robot.getDevice("Servo33")
        servo41 = self._robot.getDevice("Servo41")
        servo42 = self._robot.getDevice("Servo42")
        servo43 = self._robot.getDevice("Servo43")
        servo51 = self._robot.getDevice("Servo51")
        servo52 = self._robot.getDevice("Servo52")
        servo53 = self._robot.getDevice("Servo53")
        servo61 = self._robot.getDevice("Servo61")
        servo62 = self._robot.getDevice("Servo62")
        servo63 = self._robot.getDevice("Servo63")

        # Create leg objects
        leg1 = Leg(servo11, servo12, servo13,
                   lambda q: q * np.array([-1., 1., 1.]))
        leg2 = Leg(servo21, servo22, servo23,
                   lambda q: q * np.array([-1., 1., 1.]))
        leg3 = Leg(servo31, servo32, servo33,
                   lambda q: q * np.array([-1., 1., 1.]))
        leg4 = Leg(servo41, servo42, servo43,
                   lambda q: q + np.array([np.pi, 0., 0.]))
        leg5 = Leg(servo51, servo52, servo53,
                   lambda q: q + np.array([np.pi, 0., 0.]))
        leg6 = Leg(servo61, servo62, servo63,
                   lambda q: q + np.array([np.pi, 0., 0.]))

        self._legs = [leg1, leg2, leg3, leg4, leg5, leg6]

        self._elkapod_leg_subscription = self.create_subscription(
            LegFrames,
            "elkapod_comm_server_leg_frames",
            self._leg_frame_callback,
            10
        )

    def _leg_frame_callback(self, msg):
        leg_frames = msg.leg_frames
        for leg, frame in zip(self._legs, leg_frames):
            angles = frame.servo_angles.tolist()
            leg.set_angles(np.array([*angles]))
            self.get_logger().info(f"Setting raw angles {angles} for leg {frame.leg_nb}!")
        self.get_logger().info("All angles has been set successfully!")
