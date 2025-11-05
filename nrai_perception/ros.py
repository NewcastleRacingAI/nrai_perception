import rclpy
from rclpy.node import Node
from .code import process_image

class Perception(Node):

    def __init__(self):
        super().__init__("nrai_perception")
        self.timer = self.create_timer(1, self._timer_callback)

    def _timer_callback(self):
        self.get_logger().info("nrai_perception node is running.")

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = Perception()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()