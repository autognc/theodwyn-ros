#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool
from theo_msgs.msg import TheoCode

ACTIVE_CODE = 4
INACTIVE_CODE = 0


class BrokerBridgeNode(Node):
    """Mirrors scrat2_movement's /scrat2/should_record (std_msgs/Bool,
    published ~4Hz by vel_publisher.cpp) onto theo_msgs/TheoCode so the
    existing recorder_node / measurement_node / filter_node / image_saver_node
    all activate/deactivate exactly as they do today with the real broker."""

    def __init__(self):
        super().__init__('broker_bridge_node')

        self.declare_parameter('record_topic', '/scrat2/should_record')
        # Reuse the exact topic name every existing consumer already defaults
        # to / is already configured with (nav_ros's on_theo yaml,
        # image_saver's config, and recorder_node's own launch file all
        # already resolve to this) — nothing else needs to change.
        self.declare_parameter('broker_topic', '/eowyn/external/broker_robotic_topic')
        # should_record can drop to False for a tick or two during the chase
        # controller's approach/settle phase — require it to stay False for
        # this long before we actually deactivate/split the bag.
        self.declare_parameter('deactivate_debounce_sec', 1.0)

        record_topic = self.get_parameter('record_topic').value
        broker_topic = self.get_parameter('broker_topic').value
        self._debounce_sec = float(self.get_parameter('deactivate_debounce_sec').value)

        qos_out = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(TheoCode, broker_topic, qos_out)
        # vel_publisher publishes should_record with default QoS (reliable,
        # volatile, depth 10) — match that on the subscriber side.
        self.create_subscription(Bool, record_topic, self._on_should_record, 10)

        self._active = False
        self._false_since = None

        # publish a known inactive state immediately so late subscribers
        # (transient_local) always see a valid code before the first message
        self._publish(INACTIVE_CODE)
        self.get_logger().info(
            f'broker_bridge_node up: "{record_topic}" -> "{broker_topic}" '
            f'(code {ACTIVE_CODE}/{INACTIVE_CODE}, debounce {self._debounce_sec}s)'
        )

    def _on_should_record(self, msg: Bool):
        now = self.get_clock().now()
        if msg.data:
            self._false_since = None
            if not self._active:
                self._active = True
                self.get_logger().info('should_record=True, publishing ACTIVE broker code')
                self._publish(ACTIVE_CODE)
        else:
            if not self._active:
                return
            if self._false_since is None:
                self._false_since = now
                return
            elapsed = (now - self._false_since).nanoseconds / 1e9
            if elapsed >= self._debounce_sec:
                self._active = False
                self._false_since = None
                self.get_logger().info('should_record=False (debounced), publishing INACTIVE broker code')
                self._publish(INACTIVE_CODE)

    def _publish(self, code):
        msg = TheoCode()
        msg.code = code
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrokerBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
