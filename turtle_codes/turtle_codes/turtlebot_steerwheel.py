#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class TurtlebotSteerWheel(Node):
    def __init__(self):
        super().__init__('turtlebot_steerwheel')
        # Velocidades máximas
        self.max_linear = 0.22    # m/s
        self.max_angular = 2.84   # rad/s
        # Publicador de cmd_vel
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        # Suscripción al tópico /joy
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info('Nodo turtlebot_steerwheel iniciado')

    def joy_callback(self, msg: Joy):
        # Eje 0: timón → velocidad angular
        axis_angular = msg.axes[0]
        # Eje 1: pedales → velocidad lineal
        axis_linear = msg.axes[1]
        # Mapeo lineal
        vel_ang = axis_angular * self.max_angular
        vel_lin = axis_linear * self.max_linear
        # Construir y publicar Twist
        twist = Twist()
        twist.linear.x = vel_lin
        twist.angular.z = vel_ang
        self.pub.publish(twist)
        self.get_logger().debug(f'Publicado cmd_vel: lin={vel_lin:.2f}, ang={vel_ang:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotSteerWheel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
