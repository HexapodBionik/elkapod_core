import rclpy
from rclpy.executors import SingleThreadedExecutor

from .orientation_demo import OrientationDemo


def main():
    default_trajectory = [1.0, 3.0, 2.0, 1.5, 0.0, 0.0, -1.0, -3.0, -2.0, -1.5]
    rclpy.init()
    node = OrientationDemo("roll_demo", "/roll_setpoint", default_trajectory)
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
