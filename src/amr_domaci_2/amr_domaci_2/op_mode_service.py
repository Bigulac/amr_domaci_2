import rclpy
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import String

class LifecycleTalker(Node):

    def __init__(self):
        super().__init__('play_button')

        self.pub = None
        self.timer = None
        self._count = 0

    def publish(self):
        """Publish a new message when enabled."""
        msg = String()
        msg.data = "Lifecycle HelloWorld #" + str(self._count)
        self._count += 1

        # Only if the publisher is in an active state, the message transfer is
        # enabled and the message actually published.
        if self.pub is not None:
            self.pub.publish(msg)
            self.get_logger().info(f'Lifecycle publisher is active. Published: [{msg.data}]')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.pub = self.create_lifecycle_publisher(String, "lifecycle_chatter", 10)

        self.get_logger().info("on_configure() is called.")
        return super().on_configure(state)

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Log, only for demo purposes
        self.get_logger().info("on_activate() is called.")
        self.timer = self.create_timer(1.0, self.publish)

        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_deactivate() is called.")
        self.destroy_timer(self.timer)
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.pub)

        self.get_logger().info('on_cleanup() is called.')
        return super().on_cleanup(state)

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.pub)

        self.get_logger().info('on_shutdown() is called.')
        return super().on_shutdown(state)


def main():
    rclpy.init()

    executor = rclpy.executors.SingleThreadedExecutor()
    lc_node = LifecycleTalker()
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()