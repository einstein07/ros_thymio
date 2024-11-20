import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
from thymiodirect import Thymio
from thymiodirect.thymio_serial_ports import ThymioSerialPort
import sys

class ViconSubscriber(Node):

    def __init__(self):
        super().__init__('vicon_subscriber')

        """Thymio connection variable"""
        self.robotConnection = None
        self.declare_parameter('my_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('default_topic', rclpy.Parameter.Type.STRING)
        self.my_id = self.get_parameter('my_id')
        self.default_topic = self.get_parameter('default_topic')
        self.subscription = self.create_subscription(
            PositionList,
            self.default_topic,
            self.listener_callback,
            10)
        print('Sucessfully subscrive to %s' %self.default_topic)

        self.connect_to_thymio()

    def connect_to_thymio(self):
        try:
            port = Connection.serial_default_port()
            th = Thymio(serial_port=port, on_connect=lambda node_id: print(f' Thymio {node_id} is connected'))
            th.connect()
            robot = th[th.first_node()]
            time.sleep(1)
            self.robotConnection = robot

        except Exception as err:
            print(err)
            sys.exit("Could not connect to Thymio! Exiting...")



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
