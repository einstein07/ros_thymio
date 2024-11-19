import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList


class ViconSubscriber(Node):

    def __init__(self):
        super().__init__('vicon_subscriber')
        self.declare_parameter('my_id', rclpy.Parameter.Type.STRING)
        self.subscription = self.create_subscription(
            PositionList,
            '/vicon/default/data',
            self.listener_callback,
            10)


        """self.declare_parameter('list_of_topics', rclpy.Parameter.Type.STRING_ARRAY)
        self.id, self.topic_list = self.configure_params()
        self.subscriptions = []
        for topic in self.topic_list:
            self.subscriptions.append(
                                        Subscriber(
                                        self,
                                        Position,
                                        topic
                                        )
            )
            self.get_logger().info('successfully subscribed to topic "%s"' % topic)
        self.timer = self.create_timer(1, self.TimerCallback)
        queue_size = 10
        self.sync = TimeSynchronizer(self.subscriptions, queue_size)
        self.sync.registerCallback(self.SyncCallback)

    def configure_params(self):
        id = self.get_parameter('my_id')
        topic_list = self.get_parameter('list_of_topics')
        self.get_logger().info("my id: %s, topic-list[]: %s" %
                               (str(id.value),
                                str(topic_list.value),))

        return id, topic_list"""

    #def SyncCallback(self, msg):


    def listener_callback(self, msg):
        for i in range(msg.n):
            self.get_logger().info('subject "%s" with segment %s:' %(msg.positions[i].subject_name, msg.positions[i].segment_name))
            self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.positions[i].x_trans, msg.positions[i].y_trans, msg.positions[i].z_trans))
            self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.positions[i].x_rot, msg.positions[i].y_rot, msg.positions[i].z_rot, msg.positions[i].w))

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
