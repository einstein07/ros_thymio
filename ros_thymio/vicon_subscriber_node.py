import math

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
from thymiodirect import Connection
from thymiodirect import Thymio
import time
import pygame
import sys

class ViconSubscriber(Node):
    """Constants"""
    HARD_TURN = 'HARD_TURN'
    SOFT_TURN = 'SOFT_TURN'
    NO_TURN = 'NO_TURN'
    def __init__(self):
        super().__init__('vicon_subscriber')

        self.declare_parameter('my_id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('default_topic', rclpy.Parameter.Type.STRING)
        self.declare_parameter('target_distance', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_speed', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('gain', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('exponent', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('hard_turn_on_angle_threshold', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('soft_turn_on_angle_threshold', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('no_turn_angle_threshold', rclpy.Parameter.Type.DOUBLE)

        """Thymio connection variable"""
        self.robotConnection = None

        self.my_position = None
        """This id must match the name used in vicon for the specific subject"""
        self.my_id = self.get_parameter('my_id').get_parameter_value().string_value
        print('my id: %s' % self.my_id)
        """This is the topic that publishes a list of positions for all objects in the arena"""
        self.default_topic = self.get_parameter('default_topic').get_parameter_value().string_value
        print('default topic: %s' %self.default_topic)
        """ The following variables are used as parameters for
            flocking interaction. You can set their value
            in the configuration file """
        #Target robot - robot distance in cm
        self.target_distance = self.get_parameter('target_distance').get_parameter_value().double_value
        print('target distance: %f' % self.target_distance)
        # Maximum robot speed
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        print('max speed: %f' % self.max_speed)
        #Gain of the Lennard - Jones potential
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        print('target distance: %f' % self.gain)
        #Exponent of the Lennard - Jones potential
        self.exponent = self.get_parameter('exponent').get_parameter_value().double_value
        print('exponent: %f' % self.exponent)
        self.hard_turn_on_angle_threshold = self.get_parameter('hard_turn_on_angle_threshold').get_parameter_value().double_value
        print('hard tun on threshold: %f' % self.hard_turn_on_angle_threshold)
        self.soft_turn_on_angle_threshold = self.get_parameter('soft_turn_on_angle_threshold').get_parameter_value().double_value
        print('soft turn on threshold: %f' % self.soft_turn_on_angle_threshold)
        self.no_turn_angle_threshold = self.get_parameter('no_turn_angle_threshold').get_parameter_value().double_value
        print('no turn on threshold: %f' % self.no_turn_angle_threshold)
        self.subscription = self.create_subscription(
            PositionList,
            self.default_topic,
            self.listener_callback,
            10)
        print('Subscription created to %s' %self.default_topic)

        self.connect_to_thymio()
        self.turning_mechanism = ViconSubscriber.NO_TURN

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
            if msg.positions[i].subject_name == self.my_id:
                self.my_position = msg.positions[i]
            self.get_logger().info('subject "%s" with segment %s:' %(msg.positions[i].subject_name, msg.positions[i].segment_name))
            self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.positions[i].x_trans, msg.positions[i].y_trans, msg.positions[i].z_trans))
            self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.positions[i].x_rot, msg.positions[i].y_rot, msg.positions[i].z_rot, msg.positions[i].w))
        self.set_wheel_speed_from_vectora(self.flocking_vector(msg))

    def generalized_lennard_jones(self, f_distance):
        f_normal_distance_exp = pow(self.target_distance/f_distance, self.exponent)
        return -self.gain / f_distance * (f_normal_distance_exp * f_normal_distance_exp - f_normal_distance_exp)

    def flocking_vector(self, msg):

        if msg.n > 1:
            c_accum = pygame.math.Vector2()
            for i in range(msg.n):
                if msg.positions[i].subject_name != self.my_id:
                    """ Take the blob distance and angle
                        With the distance, calculate the Lennard-Jones interaction force
                        Form a 2D vector with the interaction force and the angle
                        Sum such vector to the accumulator"""
                    # Calculate LJ
                    LJ_force = self.generalized_lennard_jones(math.dist(
                        [self.my_position.x_trans, self.my_position.y_trans],
                        [msg.positions[i].x_trans, msg.positions[i].y_trans]
                    ))

                    # sum to accumulator
                    angle_to_neighbor = self.angle_between_points_in_degrees(
                            (self.my_position.x_trans, self.my_position.y_trans),
                            (msg.positions[i].x_trans, msg.positions[i].y_trans)
                        )
                    print('LJ Force: %f' %LJ_force)
                    print('Angle to neighbor: %f' % angle_to_neighbor)
                    c_accum += pygame.math.Vector2.from_polar((LJ_force, angle_to_neighbor))
            if msg.n - 1 > 0:
                # Divide the accumulator by the number of neighboring agents
                c_accum /= (msg.n-1)
                #Clamp the length of the vector to the max speed
                if c_accum.length() > self.max_speed:
                    c_accum.normalize()
                    c_accum *= self.max_speed;
                return c_accum;
            else:
                return pygame.math.Vector2()
        else:
            return pygame.math.Vector2()

    def set_wheel_speed_from_vectora(self, c_heading):
        # Get the heading angle
        heading_angle = self.signed_normalize_angle(c_heading)
        # Get the length of the heading vector
        heading_length = c_heading.length()
        # Clamp the speed so that it's not greater than MaxSpeed
        base_angular_wheel_speed = min(heading_length, self.max_speed)

        """State transition logic"""
        if self.turning_mechanism == ViconSubscriber.HARD_TURN:
            if math.fabs(heading_angle) <= self.soft_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN
        if self.turning_mechanism == ViconSubscriber.SOFT_TURN:
            if math.fabs(heading_angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
            elif math.fabs(heading_angle) <= self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.NO_TURN
        if self.turning_mechanism == ViconSubscriber.NO_TURN:
            if math.fabs(heading_angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
            elif math.fabs(heading_angle) > self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN

        """Wheel speeds based on current turning states"""
        speed1 = 0
        speed2 = 0
        if self.turning_mechanism == ViconSubscriber.NO_TURN:
            """Go straight"""
            speed1 = base_angular_wheel_speed
            speed2 = base_angular_wheel_speed
        elif self.turning_mechanism == ViconSubscriber.SOFT_TURN:
            """Both wheels go straight, but one is faster than the other"""
            speed_factor = self.hard_turn_on_angle_threshold - math.fabs(heading_angle) / self.hard_turn_on_angle_threshold
            speed1 = base_angular_wheel_speed - base_angular_wheel_speed * (1 - speed_factor)
            speed2 = base_angular_wheel_speed + base_angular_wheel_speed * (1 - speed_factor)
        elif self.turning_mechanism == ViconSubscriber.HARD_TURN:
            speed1 = - self.max_speed
            speed2 = self.max_speed

        """ Apply the calculated speeds to the appropriate wheels"""
        left_wheel_speed = 0
        right_wheel_speed = 0
        if heading_angle > 0:
            # Turn left
            left_wheel_speed = speed1
            right_wheel_speed = speed2
        else:
            left_wheel_speed = speed2
            right_wheel_speed = speed1
        self.robotConnection['motor.left.target'] = left_wheel_speed
        self.robotConnection['motor.right.target'] = right_wheel_speed

    def angle_between_points_in_degrees(self, point1, point2):
        # Create vectors from the points
        vector1 = pygame.math.Vector2(point1)
        vector2 = pygame.math.Vector2(point2)

        # Calculate the angle in degrees
        angle_degrees = vector1.angle_to(vector2)

        # Convert degrees to radians
        #angle_radians = math.radians(angle_degrees)
        return angle_degrees #angle_radians

    def signed_normalize_angle(self, vector):
        # Use the angle_to method to get the angle between this vector and the positive x-axis (in degrees)
        angle_degrees = vector.angle_to(pygame.math.Vector2(1, 0))

        # Convert the angle from degrees to radians
        angle_radians = math.radians(angle_degrees)

        return angle_radians

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
