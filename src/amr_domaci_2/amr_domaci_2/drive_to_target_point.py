import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile

from tf_transformations import euler_from_quaternion
from amr_domaci_2_interfejsi.srv import SendDesiredCoords



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

        self.srv = self.create_service(SendDesiredCoords, 'send_desired_coords', self.set_target)

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
        self.linear_velocity = 0.2
        self.angular_velocity = 0.3
        self.target_x = None
        self.target_y = None
        self.target_beta = None
        self.target_distance = None

        # Kontrolna petlja na 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

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
        beta = beta - (0 if beta <= math.pi else 2 * math.pi)

        if not self.odom_received:
            self.previous_x = x
            self.previous_y = y
            self.odom_received = True
            # self.get_logger().info('Primljena prva /odom poruka.')
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
            if self.state == "WAITING":
                self.set_target()
        else:
            self.active = False
            if self.state != 'OFF':
                self.stop_robot()
            self.state = 'OFF'

    def timer_callback(self):
        
        if not self.active:
            return
        
        ROS_DISTRO = os.environ.get('ROS_DISTRO')
        msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()

        if self.state == 'ALIGN':
            alpha = math.atan2(self.target_y - self.previous_y, self.target_x - self.previous_x)
            delta = (self.previous_beta - alpha) % (2 * math.pi)
            delta = delta - (0 if delta <= math.pi else 2* math.pi)
            if abs(delta) < math.pi/100 or abs(delta) > 99 * math.pi / 100:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                self.state = 'MOVE'
                self.get_logger().info('Robot je poravnat sa zeljenim pravcem. Prelazak na stanje kretanja.')
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = (-self._signum(delta) \
                                     if abs(delta) < math.pi / 2 \
                                     else self._signum(delta)) * self.angular_velocity
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = (-self._signum(delta) \
                                           if abs(delta) < math.pi / 2 \
                                           else self._signum(delta)) * self.angular_velocity

        elif self.state == 'MOVE':
            alpha = math.atan2(self.target_y - self.previous_y, self.target_x - self.previous_x)
            delta = (self.previous_beta - alpha) % (2 * math.pi)
            delta = delta - (0 if delta <= math.pi else 2* math.pi)
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
                self.get_logger().info('Robot dostigao zeljenu poziciju. Sledi postizanje zeljene orijentacije.')
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = self.linear_velocity if abs(delta) < math.pi/2 else -self.linear_velocity
                    msg.angular.z = (-self._signum(delta) \
                                     if abs(delta) < math.pi / 2 \
                                     else self._signum(delta)) * self.angular_velocity
                else:
                    msg.twist.linear.x = self.linear_velocity if abs(delta) < math.pi/2 else -self.linear_velocity
                    msg.twist.angular.z = (-self._signum(delta) \
                                     if abs(delta) < math.pi / 2 \
                                     else self._signum(delta)) * self.angular_velocity

        elif self.state == 'ORIENT':
            delta = (self.previous_beta - self.target_beta) % (2 * math.pi)
            delta = delta - (0 if delta <= math.pi else 2* math.pi)
            if abs(delta) < math.pi/100:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = 0.0
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                self.state = 'WAITING'
                self.get_logger().info('Robot postigao zeljenu poziciju i orijentaciju. Robot zaustavljen.')
            else:
                if ROS_DISTRO == 'humble':
                    msg.linear.x = 0.0
                    msg.angular.z = -self._signum(delta) * self.angular_velocity
                else:
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = -self._signum(delta) * self.angular_velocity

        self.cmd_vel_pub.publish(msg)

    """def set_target(self, request, response):
        self.target_x = request.x
        self.target_y = request.y
        self.target_beta = (request.beta * math.pi / 180) % (2 * math.pi)
        self.target_beta = self.target_beta - (0 if self.target_beta <= math.pi else 2 * math.pi)
        self.state = 'ALIGN'
        response.success = True
        self.get_logger().info('Primljena ciljna pozicija i orijentacija. Robot se usmerava ka zeljenoj poziciji.')
        return response
    """

    def set_target(self):
        print(init_msg)
        self.target_x = float(input("Unesi x: ")) if self.active else 0
        self.target_y = float(input("Unesi y: ")) if self.active else 0
        self.target_beta = float(input("Unesi beta: ")) if self.active else 0
        self.target_beta = (self.target_beta * math.pi / 180) % (2 * math.pi)
        self.target_beta = self.target_beta - (0 if self.target_beta <= math.pi else 2 * math.pi)
        self.state = 'ALIGN'
        
    
    def stop_robot(self):
        ROS_DISTRO = os.environ.get('ROS_DISTRO')

        msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()
        self.cmd_vel_pub.publish(msg)

        self.get_logger().info(
                'Promenjen rezim rada iz automatskog u manuelni. Robot zaustavljen.'
            )


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
    except KeyboardInterrupt:
        node.get_logger().info('Prekid programa. Zaustavljam robota.')
        ROS_DISTRO = os.environ.get('ROS_DISTRO')
        stop_msg = Twist() if ROS_DISTRO == 'humble' else TwistStamped()
        node.cmd_vel_pub.publish(stop_msg)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
