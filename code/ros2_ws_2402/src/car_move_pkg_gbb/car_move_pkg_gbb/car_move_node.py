import rclpy
from rclpy.node import Node
# Libreria de msgs tipo Twist
from geometry_msgs.msg import Twist
import pygame
import math
import sys

# Librerias para operaciones matematicas
import numpy as np
from numpy import cos, sin

class CarMoveNode(Node):
    def __init__(self):
        # Rename the node to 'car_move_node'
        super().__init__('car_move_node')
        # Create a publisher for Twist messages on the '/cmd_vel' topic (ang x, ang y)
        self.publisher_ = self.create_publisher(Twist, '/cmd', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_twist)
        self.u = 0
        self.r = 0
        self.Q = np.array([[47.619047619,0,-3.976190476],[47.619047619,0,3.976190476]])
        self.w = np.array([[0],[0]])
        
        self.u_inc = 0.01
        self.r_inc = 1.57/5

        self.u_thres = 0.1
        self.r_thres = 1.57*3

        # Initialize pygame and set up key detection
        pygame.init()
        self.screen = pygame.display.set_mode((100, 100))  # Small window to capture events
        pygame.display.set_caption("Car Control")

    def publish_twist(self):
        # Create a Twist message and set linear and angular velocities
        twist = Twist()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Check which keys are pressed
        keys = pygame.key.get_pressed()

        if keys[pygame.K_UP]:
            self.u += self.u_inc
            self.r = 0
        elif keys[pygame.K_DOWN]:
            self.u -= self.u_inc
            self.r = 0
        elif keys[pygame.K_LEFT]:
            self.u = 0
            self.r += self.r_inc
        elif keys[pygame.K_RIGHT]:
            self.u = 0
            self.r -= self.r_inc
        else :
            self.u = 0
            self.r = 0

        # Saturation
        if self.u >= self.u_thres :
            self.u = self.u_thres
        elif self.u <= -self.u_thres:
            self.u = -self.u_thres

        if self.r >= self.r_thres :
            self.r = self.r_thres
        elif self.r <= -self.r_thres:
            self.r = -self.r_thres

        zeta = np.array([[self.u],[0],[self.r]])
        self.w = self.Q@zeta

        conv_factor = 1/(2*math.pi)*60
        # convert w to RPM:
        self.w *= conv_factor
        
        twist.angular.x = self.w[0,0]
        twist.angular.y = self.w[1,0] 

        # Publish the Twist message
        self.publisher_.publish(twist)
        self.get_logger().info(f'Wheels Speed SP: w1 = {self.w[0,0]}, w2 = {self.w[1,0]}')

def main(args=None):
    rclpy.init(args=args)
    node = CarMoveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
