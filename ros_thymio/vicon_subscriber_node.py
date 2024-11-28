import math

import rclpy
from rclpy.node import Node
from message_filters import Subscriber, TimeSynchronizer
from vicon_receiver.msg import Position
from vicon_receiver.msg import PositionList
from thymiodirect import Connection
from thymiodirect import Thymio
import numpy as np
import datetime
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
        self.declare_parameter('target_x', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('target_y', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('max_speed', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('gain', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('exponent', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('hard_turn_on_angle_threshold', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('soft_turn_on_angle_threshold', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('no_turn_angle_threshold', rclpy.Parameter.Type.DOUBLE)

        """Thymio connection variable"""
        self.robotConnection = None

        self.my_position = None
        self.current_yaw = 0
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
        # Target final position-x
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        print('target x: %f' % self.target_x)
        # Target final position-y
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        print('target y: %f' % self.target_y)
        # Maximum robot speed
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        print('max speed: %f' % self.max_speed)
        #Gain of the Lennard - Jones potential
        self.gain = self.get_parameter('gain').get_parameter_value().double_value
        print('target distance: %f' % self.gain)
        #Exponent of the Lennard - Jones potential
        self.exponent = self.get_parameter('exponent').get_parameter_value().double_value
        print('exponent: %f' % self.exponent)
        self.hard_turn_on_angle_threshold = math.radians(self.get_parameter('hard_turn_on_angle_threshold').get_parameter_value().double_value)
        print('hard tun on threshold: %f' % self.hard_turn_on_angle_threshold)
        self.soft_turn_on_angle_threshold = math.radians(self.get_parameter('soft_turn_on_angle_threshold').get_parameter_value().double_value)
        print('soft turn on threshold: %f' % self.soft_turn_on_angle_threshold)
        self.no_turn_angle_threshold = math.radians(self.get_parameter('no_turn_angle_threshold').get_parameter_value().double_value)
        print('no turn on threshold: %f' % self.no_turn_angle_threshold)
        self.subscription = self.create_subscription(
            PositionList,
            self.default_topic,
            self.listener_callback,
            10)
        print('Subscription created to %s' %self.default_topic)

        self.connect_to_thymio()
        self.turning_mechanism = ViconSubscriber.NO_TURN
        self.timer_ = 0

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
        if self.timer_ % 1 == 0:
            print(datetime.datetime.now())
            for i in range(msg.n):
                if msg.positions[i].subject_name == self.my_id:
                    print('my id %s' % self.my_id)
                    self.my_position = msg.positions[i]
                    self.current_yaw = self.quaternion_to_yaw()
                    break
            self.my_position.x_trans = self.mm_to_m(self.my_position.x_trans)
            self.my_position.y_trans = self.mm_to_m(self.my_position.y_trans)
                #self.get_logger().info('subject "%s" with segment %s:' %(msg.positions[i].subject_name, msg.positions[i].segment_name))
                #self.get_logger().info('I heard translation in x, y, z: "%f", "%f", "%f"' % (msg.positions[i].x_trans, msg.positions[i].y_trans, msg.positions[i].z_trans))
                #self.get_logger().info('I heard rotation in x, y, z, w: "%f", "%f", "%f", "%f": ' % (msg.positions[i].x_rot, msg.positions[i].y_rot, msg.positions[i].z_rot, msg.positions[i].w))
            mag, angle = self.navigate_to()
            self.set_wheel_speed_from_vectora(mag, angle)#self.vector_to_target())#self.flocking_vector(msg) + self.vector_to_target())
            #self.headToPosition(self.target_x, self.target_y)
            self.timer_ = 0
            print('done')
        else:
            self.timer_ = self.timer_ + 1

    def generalized_lennard_jones(self, f_distance):
        f_normal_distance_exp = pow(self.target_distance/f_distance, self.exponent)
        return -self.gain / f_distance * (f_normal_distance_exp * f_normal_distance_exp - f_normal_distance_exp)

    def vector_to_target(self):
        c_accum = pygame.math.Vector2()
        # sum to accumulator
        dist_to_target = math.dist(
                        [self.mm_to_cm(self.my_position.x_trans), self.mm_to_cm(self.my_position.y_trans)],
                        [self.mm_to_cm(self.target_x), self.mm_to_cm(self.target_y)]
                    )
        angle_to_target = self.angle_between_points_in_degrees(
            (self.mm_to_cm(self.my_position.x_trans), self.mm_to_cm(self.my_position.y_trans)),
            (self.mm_to_cm(self.target_x), self.mm_to_cm(self.target_y))
        )
        print('my current position x: %f y: %f' %(self.mm_to_cm(self.my_position.x_trans), self.mm_to_cm(self.my_position.y_trans)))
        print('Distance to target: %f' %dist_to_target)
        print('Angle to target: %f' % angle_to_target)
        c_accum += pygame.math.Vector2.from_polar((dist_to_target, angle_to_target))

        if c_accum.length() > 0:
            c_accum.normalize()
            c_accum *= 0.25 * self.max_speed

        print('vector to target position: %f' %c_accum.length())
        return c_accum

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
                        [self.mm_to_cm(self.my_position.x_trans), self.mm_to_cm(self.my_position.y_trans)],
                        [self.mm_to_cm(msg.positions[i].x_trans), self.mm_to_cm(msg.positions[i].y_trans)]
                    ))

                    # sum to accumulator
                    angle_to_neighbor = self.angle_between_points_in_degrees(
                            (self.mm_to_cm(self.my_position.x_trans), self.mm_to_cm(self.my_position.y_trans)),
                            (self.mm_to_cm(msg.positions[i].x_trans), self.mm_to_cm(msg.positions[i].y_trans))
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
                    c_accum *= self.max_speed
                print('flocking vector magnitude: %f' %c_accum.length())
                return c_accum
            else:
                return pygame.math.Vector2()
        else:
            return pygame.math.Vector2()

    def navigate_to(self):
        # Calculate distance and desired angle
        distance = math.dist(
            [self.my_position.x_trans, self.my_position.y_trans],
            [self.target_x, self.target_y])
        desired_angle = np.arctan2(self.target_y - self.my_position.y_trans, self.target_x - self.my_position.x_trans)
        angle_diff = (desired_angle - self.current_yaw + np.pi) % (2 * np.pi) - np.pi
        print('current position x %f y %f current yaw in radians %f in degrees: %f' %(self.my_position.x_trans, self.my_position.y_trans, self.current_yaw, math.degrees(self.current_yaw)))
        print('desired angle: %f angle diff degrees: %f' % (math.degrees(desired_angle), math.degrees(angle_diff)))
        #return pygame.math.Vector2.from_polar((distance*100, desired_angle))
        return (distance*100), angle_diff

    def set_wheel_speed_from_vector1(self, distance, angle_diff):
        """
        Set the wheel speeds based on the distance to the target and the angle difference.

        Parameters:
        - distance: Distance to the target (in millimeters).
        - angle_diff: Difference between the desired angle and the current yaw (in radians).
        """
        # Parameters to tune for navigation behavior
        Kp_linear = 1.0  # Proportional control for linear speed (forward movement)
        Kp_angular = 10.0  # Proportional control for angular speed (turning)
        min_speed = 10  # Minimum speed to avoid robot stalling
        distance_threshold = 20  # Distance threshold (mm) to consider the target "reached"

        # Calculate the base forward speed (linear velocity)
        # Decrease speed as the robot gets closer to the target
        base_speed = Kp_linear * distance
        if distance < distance_threshold*0.001:
            base_speed = 0  # Stop if close enough to the target

        # Limit the forward speed to the maximum allowable speed
        base_speed = np.clip(base_speed, min_speed, self.max_speed)

        # Calculate the turning speed (angular velocity)
        turn_speed = Kp_angular * angle_diff

        # Limit the turning speed to the maximum allowable speed
        turn_speed = np.clip(turn_speed, -self.max_speed, self.max_speed)

        # Calculate the wheel speeds based on linear and angular velocities
        left_wheel_speed = base_speed - turn_speed
        right_wheel_speed = base_speed + turn_speed

        # Clamp the wheel speeds to avoid exceeding maximum limits
        left_wheel_speed = np.clip(left_wheel_speed, -self.max_speed, self.max_speed)
        right_wheel_speed = np.clip(right_wheel_speed, -self.max_speed, self.max_speed)

        # Debug information for monitoring
        print(f"Distance to target: {distance:.2f} mm")
        print(f"Angle difference: {math.degrees(angle_diff):.2f} degrees")
        print(f"Left wheel speed: {left_wheel_speed:.2f}")
        print(f"Right wheel speed: {right_wheel_speed:.2f}")

        # Apply wheel speeds to the robot
        self.robotConnection['motor.left.target'] = int(left_wheel_speed)
        self.robotConnection['motor.right.target'] = int(right_wheel_speed)

    def set_wheel_speed_from_vector2(self, mag, angle):
        """
        Set the wheel speeds based on the distance to the target and the angle difference,
        while respecting the defined turning thresholds.

        Parameters:
        - mag: Distance to the target (in millimeters).
        - angle: Difference between the desired angle and the current yaw (in radians).
        """

        # Print current turning mechanism and thresholds
        print('Current turning mechanism: %s' % self.turning_mechanism)
        print('No turn angle: %f° | Soft-turn-on angle: %f° | Hard-turn-on angle: %f°' % (
            math.degrees(self.no_turn_angle_threshold),
            math.degrees(self.soft_turn_on_angle_threshold),
            math.degrees(self.hard_turn_on_angle_threshold)
        ))

        # Get the length of the heading vector (magnitude) and clamp the speed
        heading_length = mag
        base_angular_wheel_speed = min(heading_length, self.max_speed)

        # State transition logic based on the angle difference (turning state machine)
        if self.turning_mechanism == ViconSubscriber.HARD_TURN:
            if abs(angle) <= self.soft_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN
                print('Switching to SOFT TURN')

        if self.turning_mechanism == ViconSubscriber.SOFT_TURN:
            if abs(angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
                print('Switching to HARD TURN')
            elif abs(angle) <= self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.NO_TURN
                print('Switching to NO TURN')

        if self.turning_mechanism == ViconSubscriber.NO_TURN:
            if abs(angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
                print('Switching to HARD TURN')
            elif abs(angle) > self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN
                print('Switching to SOFT TURN')

        # Wheel speeds based on the current turning mechanism
        if self.turning_mechanism == ViconSubscriber.NO_TURN:
            # Go straight
            left_wheel_speed = base_angular_wheel_speed
            right_wheel_speed = base_angular_wheel_speed

        elif self.turning_mechanism == ViconSubscriber.SOFT_TURN:
            # Adjust one wheel speed more than the other for a gentle turn
            turn_factor = abs(angle) / self.hard_turn_on_angle_threshold
            left_wheel_speed = base_angular_wheel_speed * (1 - turn_factor) if angle > 0 else base_angular_wheel_speed
            right_wheel_speed = base_angular_wheel_speed * (
                        1 + turn_factor) if angle > 0 else base_angular_wheel_speed * (1 - turn_factor)

        elif self.turning_mechanism == ViconSubscriber.HARD_TURN:
            # Perform a hard turn (one wheel goes forward, the other backward)
            turn_speed = base_angular_wheel_speed / 2
            if angle > 0:
                left_wheel_speed = -turn_speed
                right_wheel_speed = turn_speed
            else:
                left_wheel_speed = turn_speed
                right_wheel_speed = -turn_speed

        # Clamp wheel speeds within the allowed range
        left_wheel_speed = np.clip(left_wheel_speed, -self.max_speed, self.max_speed)
        right_wheel_speed = np.clip(right_wheel_speed, -self.max_speed, self.max_speed)

        # Debug: Print wheel speed and angle information
        print(f"New turning mechanism: {self.turning_mechanism} | Heading angle: {math.degrees(angle):.2f}°")
        print(f"Setting left wheel to: {left_wheel_speed:.2f} | Right wheel to: {right_wheel_speed:.2f}")

        # Apply the calculated speeds to the appropriate wheels
        self.robotConnection['motor.left.target'] = int(left_wheel_speed)
        self.robotConnection['motor.right.target'] = int(right_wheel_speed)

    def set_wheel_speed_from_vectora(self, mag, angle):
        print('current turning mechanism: %s' %self.turning_mechanism)
        print('no turn angle %f soft-turn-on angle %f hard-turn-on angle %f' %(math.degrees(self.no_turn_angle_threshold), math.degrees(self.soft_turn_on_angle_threshold), math.degrees(self.hard_turn_on_angle_threshold)))

        # Get the heading angle
        #heading_angle = self.signed_normalize_angle(c_heading)
        heading_angle = angle
        # Get the length of the heading vector
        #heading_length = c_heading.length()
        heading_length = mag
        # Clamp the speed so that it's not greater than MaxSpeed
        base_angular_wheel_speed = min(heading_length, self.max_speed)

        """State transition logic"""
        if self.turning_mechanism == ViconSubscriber.HARD_TURN:
            if math.fabs(heading_angle) <= self.soft_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN
                print('entering soft turn')
        if self.turning_mechanism == ViconSubscriber.SOFT_TURN:
            if math.fabs(heading_angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
                print('entering hard turn')
            elif math.fabs(heading_angle) <= self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.NO_TURN
                print('entering no turn')
        if self.turning_mechanism == ViconSubscriber.NO_TURN:
            if math.fabs(heading_angle) > self.hard_turn_on_angle_threshold:
                self.turning_mechanism = ViconSubscriber.HARD_TURN
                print('entering hard turn')
            elif math.fabs(heading_angle) > self.no_turn_angle_threshold:
                self.turning_mechanism = ViconSubscriber.SOFT_TURN
                print('entering soft turn')

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
        print('new turning mechanism: %s with heading angle of %f' % (self.turning_mechanism, angle))
        print('setting left wheel to: %s speed actual value: %f' %(int(left_wheel_speed), left_wheel_speed) )
        print('setting right wheel to: %s speed actual value: %f' %(int(right_wheel_speed), right_wheel_speed))
        self.robotConnection['motor.left.target'] = int(left_wheel_speed)
        self.robotConnection['motor.right.target'] = int(right_wheel_speed)

    def angle_between_points_in_radians(self, point1, point2):
        # Create vectors from the points
        vector1 = pygame.math.Vector2(point1)
        vector2 = pygame.math.Vector2(point2)

        # Calculate the angle in degrees
        angle_degrees = vector1.angle_to(vector2)
        print('angle in %f degrees' % angle_degrees)
        # Convert degrees to radians
        angle_radians = math.radians(angle_degrees)
        print('angle in %f radians' % angle_radians)
        return angle_radians

    def signed_normalize_angle(self, vector):
        # Use the angle_to method to get the angle between this vector and the positive x-axis (in degrees)
        angle_degrees = vector.angle_to(pygame.math.Vector2(1, 0))

        # Convert the angle from degrees to radians
        angle_radians = math.radians(angle_degrees)

        return angle_radians

    def mm_to_m(self, dist):
        return dist * 0.001

    def quaternion_to_yaw(self):
        """Convert a quaternion (x, y, z, w) to a yaw angle (in radians)."""
        yaw = np.arctan2(2 * (self.my_position.w * self.my_position.z_rot + self.my_position.x_rot * self.my_position.y_rot), 1 - 2 * (self.my_position.y_rot ** 2 + self.my_position.z_rot ** 2))
        yaw_degrees = math.degrees(yaw)
        print('calculated yaw: %f' %yaw_degrees)
        return yaw

    def headToPosition(self, x, y):
        arrived = False
        if (not arrived):
            """response = self.getPosition()
            if (self.my_position.x_trans == -1 or self.my_position.y_trans == -1):
                self.timesMissed = self.timesMissed + 1
                continue"""

            if (x - 5 <= self.my_position.x_trans <= x + 5 and y - 5 <= self.my_position.y_trans <= y + 5):
                arrived = True
                """continue"""

            setHeading = self.calcHeading(self.my_position.x_trans, self.my_position.y_trans, x, y)
            if (setHeading - 5 < self.current_yaw < setHeading + 5):
                correctHeading = True
            else:
                correctHeading = False

            if (not correctHeading):

                """if (response.xcoord == -1):
                    response = self.getPosition()
                    continue"""

                if (setHeading > self.current_yaw):
                    counterClockwise = (360 - setHeading) + self.current_yaw
                    clockwise = setHeading - self.current_yaw
                else:
                    counterClockwise = self.current_yaw - setHeading
                    clockwise = (360 - self.current_yaw) + setHeading

                if (counterClockwise < clockwise):
                    print('heading: %f current: %f' %(setHeading, self.current_yaw))
                    self.robotConnection['motor.left.target'] = -75
                    self.robotConnection['motor.right.target'] = 75
                else:
                    print('heading: %f current: %f' % (setHeading, self.current_yaw))
                    self.robotConnection['motor.left.target'] = 75
                    self.robotConnection['motor.right.target'] = -75
                print('turning')
                """response = self.getPosition()

                if (response.xcoord == -1 or response.ycoord == -1):
                    self.timesMissed = self.timesMissed + 1
                    continue"""

                if (setHeading < 2):
                    setHeading = 2
                if (setHeading - 2 < self.current_yaw < setHeading + 2):
                    correctHeading = True
            else:
                self.robotConnection['motor.left.target'] = 300
                self.robotConnection['motor.right.target'] = 300
                print('going straight')
        else:
            self.robotConnection['motor.left.target'] = 0
            self.robotConnection['motor.right.target'] = 0
            print('arrived')

    def calcHeading(self, currentX, currentY, goalX, goalY):
        xToGoal = goalX - currentX
        yToGoal = goalY - currentY

        angle = math.degrees(np.arctan2(yToGoal, xToGoal))
        if(angle == 0):
            return 90
        if(0 < angle <= 90):
            return 90 - angle
        if(90 < angle <= 180):
            return 270 + (180 - angle)
        if(angle < 0):
            return 90 + (abs(angle))

def main(args=None):
    rclpy.init(args=args)

    vicon_subscriber = ViconSubscriber()

    rclpy.spin(vicon_subscriber)
    #vicon_subscriber.headToPosition(vicon_subscriber.target_x, vicon_subscriber.target_y)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vicon_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
