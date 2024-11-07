import rclpy
from rclpy.node import Node

from vicon_interfaces.msg import Position


class ViconSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Position,
            '/vicon/sindiso/sindiso',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('subject "%s" with segment %s:' %(msg.subject_name, msg.segment_name))
        self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.x_trans, msg.y_trans, msg.z_trans))
        self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.x_rot, msg.y_rot, msg.z_rot, msg.w))

def main(args=None):
    rclpy.init(args=args)

    vicon_subscriber = ViconSubscriber()

    rclpy.spin(vicon_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
