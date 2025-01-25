#!/usr/bin/env python3
# HACE EXPLICITO EL INTERPRETE A UTILIZAR, EN ESTE CASO VER. Python3.

import rclpy # importa ROS CLIENT LIBRARY FOR PYTHON o rclpy for short.
from rclpy.node import Node # importa clase 'Node' del script 'node' dentro de rclpy
from geometry_msgs.msg import Twist # importa clase Twist (es un tipo de mensaje) 
#dentro del script 'geometry_msgs.msg' utilizada de manera general para cualquier nodo de ROS2.
from turtlesim.msg import Pose # importa clase Pose (tipo de msj) dentro del script 'turtlesim.msg', este 
# tipo de msj es exclusivo del paquete turtlesim.

# Nodo customizado y configurado, hereda de la clase 'Node' encontrada en el script rclpy.node
class TurtleMoveNodeGBB(Node):
    # definicion del constructor
    def __init__(self):
        # renombra al nodo para aparecer en la terminal con el nombre correcto cuando
        # usemos el comando 'ros2 node list'
        super().__init__("turtle_move_node_gbb") 
        # mensaje de verificacion de que el nodo fue instanciado
        print('hello from my new node')

        # ------------------- PUBLISHER -----------------------------

        # crea el obj. publicador con nombre 'cmd_pub' con mensajes tipo 'Twist', publicar'a
        # en el t'opico '/turtle1/cmd_vel' (que ya existe dentro de ros2, aparece cuando 
        # ejecutamos el nodo 'turtlesim_node dentro del pkg 'turtlesim'), 
        # y el buffer o "memoria" de la cola ser'a de 10 mensajes, 
        # esto quiere decir que solo pueden existir 10 mensajes en cola al mismo tiempo.
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # crea un periodo para publicar mensajes en el topico, en segundos.
        timer_period = 0.5 # [s]
        # crea un obj. llamado 'timer', utiliza el metodo heredado 'create_timer' para este fin.
        # utiliza el timer_period como periodo del temporizador y ejecuta la funcion 
        # llamada timer_callback (a definir mas adelante) cada 'timer_period' segundos.
        self.timer = self.create_timer(timer_period, self.timer_callback)        

        # ------------------- SUBSCRIBER -----------------------------

        # crea un obj. subscriptor llamado 'pose_sub' utilizando el metodo heredado 'create_subscription'
        # el cual utiliza el tipo de mensaje 'Pose', recibe mensajes del topico '/turtle1/pose', y 
        # cada que recibe un msj por este topico, ejecuta el metodo 'pose_callback' definido mas adelante.
        # finalmente, el maximo de mensajes en cola (o sea el buffer) es 1.
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1)


    # definicion de funcion que ser'a llamada en el timer.
    def timer_callback(self):
        # creacion de obj. de clase Twist.
        cmd_vel = Twist()

        # modificaci'on de atributos 'linear.x' y 'angular.z' del obj 'cmd_vel'
        cmd_vel.linear.x = 0.5
        cmd_vel.angular.z = 1.0 # [rad/s]

        # utiliza el obj. publicador 'cmd_pub' creado anteriormente y el metodo 'publish' heredado 
        # para publicar la informacion que se encuentra en el objeto mensaje 'cmd_vel' tipo 'Twist'
        self.cmd_pub.publish(cmd_vel)

    # definicion de funcion que ser'a llamada en el publisher
    def pose_callback(self, data): # recibe en data el mensaje tipo Pose del topico indicado en 'create_subscription'
        # en este caso, el mensaje tipo Pose presenta los atributos 'x', 'y', y 'theta'.
        # entonces generamos un string llamado 'msg' utilizando string formatting de python.
        # el '\n' sirve para hacer un salto de linea y obtener la informaci'on de una manera ordenada.
        msg = '\nX: {:.3f} \nY: {:.3f}\nTHETA:{:.3f} \n---\n'.format(data.x, data.y, data.theta)
        
        # get_logger es un metodo heredado de la clase 'Node' y el objeto que devuelve en su return (seguramente es
        # un tipo Logger si es que as'i se llama dentro de la estructura de RCLPY) a su vez tiene un metodo llamado 'info', 
        # este metodo recibe un mensaje como argumento, y lo printea en la terminal como un mensaje LOG (es decir, incluye informaci'on
        # relevante como el tiempo y el tipo de mensaje e.g. [INFO] adem'as del mensaje a printear). en este caso, imprimimos 
        # el mensaje 'msg' en el logger.
        self.get_logger().info(msg)

# ------------------------ FUNCION MAIN ----------------------        
def main(args=None): # recibe 'NULO' como argumento, o sea 'None'
    # inicializa el paquete rclpy (desconocemos el codigo interno del metodo init de 'rclpy')
    rclpy.init(args=args)
    # Crea el objeto 'node' instanciando la clase 'TurtleMoveNodeGBB' en este caso.
    node = TurtleMoveNodeGBB()

    # utiliza el metodo 'spin' de rclpyl, que funciona como un ciclo while(true) pero con algunas funcionalidades internas adicionales.
    rclpy.spin(node)

    # destruye el nodo creado una vez se sale del ciclo while generado en el metodo 'spin'
    # esta linea es *OPCIONAL* debido a que de igual manera el objeto es destruido automaticamente por el 'garbage collector' 
    # segun la documentacion oficial de ROS2.
    node.destroy_node()

    # "apaga" la libreria rclpy.
    rclpy.shutdown()

# ------------------------ MAGIC VARIABLE __name__ -------------------------------------

# ejecuta la funcion main (es decir, en este caso, crea el objeto) SOLO SI este archivo es ejecutado directamente, 
# en caso de ser importado por otro archivo dentro del paquete, la funcion MAIN *NO* sera ejecutada.
if __name__ == "__main__":
    main()