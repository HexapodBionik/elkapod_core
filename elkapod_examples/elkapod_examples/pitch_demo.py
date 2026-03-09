import rclpy
from rclpy.executors import SingleThreadedExecutor

from .orientation_demo import OrientationDemo


def main():
    default_trajectory = [2.0, 6.0, 4.0, 3.0, 0.0, 0.0, -2.0, -6.0, -4.0, -3.0]
    rclpy.init()
    node = OrientationDemo("pitch_demo", "/pitch_setpoint", default_trajectory)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
