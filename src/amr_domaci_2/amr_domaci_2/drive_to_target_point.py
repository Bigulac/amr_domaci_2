import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile

from tf_transformations import euler_from_quaternion



class DriveRobot(Node):
    def __init__(self):
        super().__init__('drive_to_target')

        # Publisher: nas cvor salje komande brzine robotu
        ROS_DISTRO = os.environ.get('ROS_DISTRO')
        qos = QoSProfile(depth=10)
        if ROS_DISTRO == 'humble':
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        else:
            self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        

        # Subscriber: nas cvor cita odometriju robota
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            Bool,
            'set_op_mode/active_mode',
            self.active_mode_callback,
            10
        )

        # Izabran režim rada
        self.active = False
        self.active_prev = False
        self.state = "OFF"

        # Stanje odometrije
        self.odom_received = False
        self.previous_x = None
        self.previous_y = None
        self.previous_beta = None
        self.distance_travelled = 0.0

        # Parametri zadatka
        self.linear_velocity = 0.5
        self.angular_velocity = 1.0
        self.target_x = None
        self.target_y = None
        self.target_beta = None
        self.target_distance = None

        # Kontrolna petlja na 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        """
        self.get_logger().info(
            f'Krecem pravo koristeci /odom: v={self.speed} m/s, cilj={self.target_distance} m'
        )
        """

    def _signum(self, var):
        return abs(var)/var if var != 0 else 0

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        beta = euler_from_quaternion([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            ])[2]
        while beta > math.pi or beta < -math.pi:
            beta = beta - self._signum(beta) * 2 * math.pi

        if not self.odom_received:
            self.previous_x = x
            self.previous_y = y
            self.odom_received = True
            self.get_logger().info('Primljena prva /odom poruka. Pocetak merenja puta.')
            return

        dx = x - self.previous_x
        dy = y - self.previous_y

        step_distance = math.sqrt(dx * dx + dy * dy)
        self.distance_travelled += step_distance

        self.previous_x = x
        self.previous_y = y
        self.previous_beta = beta

        if self.state == 'WAITING':
            self.target_x = x
            self.target_y = y
            self.target_beta = beta

    def active_mode_callback(self, msg: Bool):
        if msg.data:
            self.active = True
            self.state = 'WAITING' if self.state == 'OFF' else self.state
        else:
            self.active = False
            self.state = 'OFF'

    def timer_callback(self):
        
        # print(self.state)

        if not self.active:
            return
        
        ROS_DISTRO = os.environ.get('ROS_DISTRO')
        msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()

        alpha = math.atan2(self.target_y - self.previous_y, self.target_x - self.previous_x)

        if self.state == 'WAITING':
            print(init_msg)
            target_x = float(input('Koordinata x(format x.x):'))
            target_y = float(input('Koordinata y(format y.y):'))
            target_beta = float(input('Orijentacija u odnosu na x-osu  u stepenima (format b.b):'))
            self.set_target(target_x, target_y, target_beta)
        elif self.state == 'ALIGN':
            if abs(self.previous_beta - alpha) < math.pi/100 or abs(self.previous_beta - alpha) > 99 * math.pi / 100:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                self.state = 'MOVE'
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = (-self._signum(self.previous_beta - alpha) \
                                     if abs(self.previous_beta - alpha) < math.pi / 2 \
                                     else self._signum(self.previous_beta - alpha)) * self.angular_velocity
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = (-self._signum(self.previous_beta - alpha) \
                                           if abs(self.previous_beta - alpha) < math.pi / 2 \
                                           else self._signum(self.previous_beta - alpha)) * self.angular_velocity

        elif self.state == 'MOVE':
            if math.sqrt(
                (self.target_x - self.previous_x) * (self.target_x - self.previous_x) +
                (self.target_y - self.previous_y) * (self.target_y - self.previous_y)
                ) < 0.01:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                self.state = 'ORIENT'
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = self.linear_velocity if abs(self.previous_beta - alpha) < math.pi/2 else -self.linear_velocity
                    msg.angular.z = (-self._signum(self.previous_beta - alpha) \
                                     if abs(self.previous_beta - alpha) < math.pi / 2 \
                                     else self._signum(self.previous_beta - alpha)) * self.angular_velocity * 0.2
                else:
                    msg.twist.linear.x = self.linear_velocity if abs(self.previous_beta - alpha) < math.pi/2 else -self.linear_velocity
                    msg.twist.angular.z = (-self._signum(self.previous_beta - alpha) \
                                     if abs(self.previous_beta - alpha) < math.pi / 2 \
                                     else self._signum(self.previous_beta - alpha)) * self.angular_velocity * 0.2

        elif self.state == 'ORIENT':
            if abs(self.previous_beta - self.target_beta) < math.pi/100:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                self.state = 'WAITING'
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = -self._signum(self.previous_beta - self.target_beta) * self.angular_velocity
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = -self._signum(self.previous_beta - self.target_beta) * self.angular_velocity

        self.cmd_vel_pub.publish(msg)
        
        # if not self.odom_received:
        #     msg.linear.x = 0.0
        #     msg.angular.z = 0.0
        #     self.cmd_vel_pub.publish(msg)
        #     self.get_logger().warn('Cekam /odom poruku...')
        #     return

        # if self.distance_travelled < self.target_distance:
        #     if ROS_DISTRO == 'humble':
        #         msg.linear.x = self.speed
        #         msg.angular.z = 0.0
        #     else:
        #         msg.twist.linear.x = self.speed
        #         msg.twist.angular.z = 0.0
        #     self.cmd_vel_pub.publish(msg)

        #     self.get_logger().info(
        #         f'Predjeni put: {self.distance_travelled:.2f} / {self.target_distance:.2f} m'
        #     )
        # else:
        #     self.stop_robot()

    def set_target(self, target_x, target_y, target_beta):
        self.target_x = target_x
        self.target_y = target_y
        self.target_beta = (target_beta * math.pi / 180) % (2 * math.pi)
        self.target_beta = self.target_beta - (0 if self.target_beta <= math.pi else 2 * math.pi)
        self.state = 'ALIGN'

    def stop_robot(self):
        ROS_DISTRO = os.environ.get('ROS_DISTRO')

        msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()
        if ROS_DISTRO == 'humble':
            msg.linear.x = 0.0
            msg.angular.z = 0.0
        else:
            msg.twist.linear.x = 0.0
            msg.twist.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

        if not self.stopped:
            self.get_logger().info(
                f'Cilj dostignut. Predjeni put: {self.distance_travelled:.2f} m. Robot zaustavljen.'
            )
            self.stopped = True

init_msg = """
Ručni režim - zadavanje cilja
---------------------------
Uneti koordinate ciljne tačke

"""

def main(args=None):
    rclpy.init(args=args)

    node = DriveRobot()

    try:
        rclpy.spin(node)
        # while rclpy.ok():
        # if node.state == 'WAITING':
        #     print(init_msg)
        #     target_x = float(input('Koordinata x(format x.x):'))
        #     target_y = float(input('Koordinata y(format y.y):'))
        #     target_beta = float(input('Orijentacija u odnosu na x-osu  u stepenima (format b.b):'))
        #     node.set_target(target_x, target_y, target_beta)
    except KeyboardInterrupt:
        node.get_logger().info('Prekid programa. Zaustavljam robota.')
        ROS_DISTRO = os.environ.get('ROS_DISTRO')
        stop_msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
