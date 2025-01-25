import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import time
# Libreria de msgs tipo Twist
from geometry_msgs.msg import Twist, Pose2D

# Librerias para operaciones matematicas
import math
import numpy as np
from numpy import cos, sin

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 10 Hz
        self.joint_state = JointState()
        # self.joint_state.name = ['chassis_lwheel_joint','chassis_rwheel_joint','world_base_joint',]
        # self.joint_state.name = ['world_worldx_joint','worldx_worldy_joint','worldy_worldth_joint','chassis_lwheel_joint','chassis_rwheel_joint']
        # self.joint_state.position = [x_value, y_value, theta_value, 0.0, 0.0]
        self.joint_state.name = ['world_worldx_joint','worldx_worldy_joint','worldy_worldth_joint','chassis_lwheel_joint','chassis_rwheel_joint']
        self.joint_state.position = [0.0,0.0,0.0,0.0,0.0]
        self.start_time = time.time()

        self.pos1 = 0.0
        self.vel1 = 0.0
        self.cmd1 = 0.0
        self.pos2 = 0.0
        self.vel2 = 0.0
        self.cmd2 = 0.0

        self.pose_x = 0
        self.pose_y = 0
        self.pose_theta = 0

        # Subscripcion a msgs tipo Twists que seran procesados mediante un callback
        # callback function on each message
        self.subscription_twist = self.create_subscription(Twist,'debug_pub',self.subs_twist_callback,1)
        self.subscription_pose = self.create_subscription(Pose2D,'eta_pose',self.subs_pose_callback,1)

    def publish_joint_states(self):
        current_time = time.time() - self.start_time

        # Update the joint positions (e.g., a sine wave for joint1)
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        
        self.joint_state.position[0] = self.pose_x  # joint1
        self.joint_state.position[1] = self.pose_y  # joint1
        self.joint_state.position[2] = self.pose_theta # joint2
        self.joint_state.position[3] = self.pos1  # joint1
        self.joint_state.position[4] = self.pos2 # joint2

        # Publish the message
        self.publisher_.publish(self.joint_state)
        self.get_logger().info(f'Publishing: {self.joint_state.position}')

    def subs_twist_callback(self, msg):
        # read the msg and extract the content
        # Captura de mensajes del publicador en MicroROS por el nodo
        # Los valores se mapean desde (0,255) a (-5,5)
        self.pos1 = msg.linear.x
        self.vel1 = msg.linear.y
        self.cmd1 = msg.linear.z

        self.pos2 = msg.angular.x
        self.vel2 = msg.angular.y
        self.cmd2 = msg.angular.z
        
        # Se imprimen los valores mapeados
        print("pos1: {:.4f}, vel1: {:.4f}, cmd1: {:.4f} \n".format(self.pos1,self.vel1,self.cmd1))
        print("pos2: {:.4f}, vel2: {:.4f}, cmd2: {:.4f} \n".format(self.pos2,self.vel2,self.cmd2))

    def subs_pose_callback(self, msg):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
