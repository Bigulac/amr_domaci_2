import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class Drive(Node):
    def __init__(self):
        super().__init__('drive_to_target_point')

        # Publisher: nas cvor salje komande brzine robotu
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber: nas cvor cita odometriju robota
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Stanje odometrije
        self.odom_received = False
        self.previous_x = None
        self.previous_y = None
        self.distance_travelled = 0.0

        # Parametri zadatka
        self.speed = 0.2
        self.target_distance = 2.0

        # Stanje izvrsavanja
        self.stopped = False

        # Kontrolna petlja na 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Krecem pravo koristeci /odom: v={self.speed} m/s, cilj={self.target_distance} m'
        )