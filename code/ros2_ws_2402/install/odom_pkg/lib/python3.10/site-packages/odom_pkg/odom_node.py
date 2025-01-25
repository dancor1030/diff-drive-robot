import numpy as np
from scipy.integrate import odeint
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from math import pi


dt = 0.1
RPM_2_rads = 2*pi*1/60 # from RPM to rad/s

class Odom(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Suscriptor para recibir w1 y w2
        self.subscription = self.create_subscription(
            Twist,
            'debug_pub',
            # 'debug_pub',
            self.listener_callback,
            10)

        # Publicador para enviar eta_pose
        self.publisher_ = self.create_publisher(
            Pose2D,
            'eta_pose',
            10)


        # Inicializar variables
        self.w1 = 0.0
        self.w2 = 0.0
        self.eta = np.array([[0.0], [0.0], [0.0]])  # [x, y, theta]
        self.last_time = self.get_clock().now()

        # Parámetros del robot
        # self.a = 1.0  # Parámetro 'a' del robot
        # self.b = 0.5  # Parámetro 'b' del robot


        self.Q = np.array([[47.619047619,0,-3.976190476],[47.619047619,0,3.976190476]])

        self.W = np.transpose(self.Q) @ np.linalg.inv(self.Q @ np.transpose(self.Q))

        # Timer para actualizar la pose
        self.timer = self.create_timer(dt, self.timer_callback)

    def listener_callback(self, msg):
        self.w1 = RPM_2_rads*msg.linear.y # rad/s
        self.w2 = RPM_2_rads*msg.angular.y # rad/s

    def timer_callback(self):
        # Calcular el intervalo de tiempo desde la última actualización
        # current_time = self.get_clock().now()
        # dt = (current_time - self.last_time).nanoseconds / 1e9  # Convertir a segundos
        # self.last_time = current_time

        # Calcular ξ
        xi = self.calculate_xi(self.w1, self.w2)
        

        # Calcular η̇
        eta_dot = self.calculate_eta_dot(xi, float(self.eta[2]))

        # Integrar η̇ para obtener η
        self.eta = np.array(self.eta, dtype=np.float64)
        eta_dot = np.array(eta_dot, dtype=np.float64)

        self.eta += eta_dot * dt
        
        # Publicar eta_pose
        pose_msg = Pose2D()
        pose_msg.x = float(self.eta[0])
        pose_msg.y = float(self.eta[1])
        pose_msg.theta = float(self.eta[2])
        self.publisher_.publish(pose_msg)


        # self.get_logger().info(f'Publicando eta_pose: x={pose_msg.x}, y={pose_msg.y}, theta={pose_msg.theta}')

    def calculate_xi(self, w1, w2):
        # Calcular ξ = W * [w1, w2]
        omega = np.array([[w1], 
                          [w2]])
        xi = self.W @ omega
        # print(omega)
        return xi

    def calculate_eta_dot(self, xi, theta):
        J = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta),  np.cos(theta), 0],
                      [0,              0,             1]])        
        eta_dot = J @ xi
        # eta_dot = J_inv @ xi.reshape(3, 1)
        return eta_dot

    def calculate_J_inv(self, theta):
        # Calcular la inversa de la matriz Jacobiana J
        J = np.array([[np.cos(theta), -np.sin(theta), 0],
                      [np.sin(theta),  np.cos(theta), 0],
                      [0,              0,             1]])
        J_inv = np.linalg.inv(J)
        return J_inv

def main(args=None):
    rclpy.init(args=args)

    robot_kinematics = Odom()

    rclpy.spin(robot_kinematics)

    robot_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()