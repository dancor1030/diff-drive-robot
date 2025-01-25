#!/usr/bin/env python3
# HACE EXPLICITO EL INTERPRETE A UTILIZAR, EN ESTE CASO VER. Python3.

import rclpy # importa ROS CLIENT LIBRARY FOR PYTHON o rclpy for short.

from rclpy.node import Node # importa clase 'Node' del script 'node' dentro de rclpy

from turtlesim.msg import Pose # importa clase Pose (tipo de msj) dentro del script 'turtlesim.msg', este 
# tipo de msj es exclusivo del paquete turtlesim.

from geometry_msgs.msg import Twist # importa clase Twist (es un tipo de mensaje) 
#dentro del script 'geometry_msgs.msg' utilizada de manera general para cualquier nodo de ROS2.

from math import atan2, sqrt, pi # importamos atan2() para el calculo de theta referencia (setpoint variable angular, para poder llegar 
# al punto), sqrt() lo usamos para el calculo de la distancia mas corta entre los el punto de referencia y el punto actual de la tortuga, 
# y pi es una constante importante para la conversion de unidades, queremos ver el angulo en grados.

class TurtleCMD(Node): # Creamos nodo TurtleCMD y heredamos de la clase Node
    def __init__(self):
        self.vel = Twist() # Generamos una variable llamada 'velocidad' o simplemente '.vel' y es tipo Twist
        self.ask = 1 # generamos una variable booleana para conocer si el nodo debe preguntar o no por la posici'on final deseada
        self.angerror = 1 # error angular, lo inicializamos en 1 pero luego redefinimos su valor.
        self.disterror = 1 # error de distancia, lo inicializamos en 1 pero luego redefinimos su valor
        self.angKp = 2.5 # constante proporcional para el control de posici'on angular
        self.posKp = 1 # constante proporcional para el control de posicion cartesiana
        self.x0 = 0 # posicion inicial en x, la inicializamos en 0 pero luego la redefinimos
        self.y0 = 0 # posicion inicial en y, la inicializamos en 0 pero luego la redefinimos
        self.lin=1 # variable booleana para definir cuando la tortuga puede comenzar a navegar de manera lineal y angular al 
        # mismo tiempo, esto es un 'quick fix' que se desarroll'o para cuando el error angular de la tortuga es mayor a 150 grados
        # ya que en estos errores, el sistema de control se inestabiliza, entonces con esta variable logramos que la tortuga 
        # gire hasta que su error angular sea menor a 150 grados y luego comienza a navegar de manera angular y cartesiana en simultaneo.
        self.reached_xy=0 # booleano para saber si alcanzo la posicion cartesiana deseada

        super().__init__("turtle_cmd_node") # cambia el nombre del nodo para la terminal (en node list)

        self.sub_pose = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 1) # crea subscriptor que 
        # recibir'a mensajes tipo Pose, del topico '/turtle1/pose', con un buffer de 1 y cada que reciba un dato llamara 
        # la funci'on pose_callback

        self.pub_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 1) # crea publicador en el topico '/turtle1/cmd_vel' 
        # donde publica mensjes tipo Twist, y publica con un buffer de 1.

    def pose_callback(self, data): 
        self.curr_x = data.x # guarda el dato de x en 'curr_x' que significa 'CURRENT X'
        self.curr_y = data.y # guarda el dato de y en 'curr_y' que significa 'CURRENT Y'
        self.curr_theta = data.theta # guarda el dato de theta en 'curr_theta' que significa 'CURRENT THETA'

        self.move() # llama la funcion move

        # crea un string con la informacion que deseamos presentar en el logger de ROS2
        msg = f'\nCURRENT X = {self.curr_x}\nCURRENT Y = {self.curr_y}\nCURRENT THETA [deg] = {self.curr_theta*180/pi}\n----------------------\n'
        # print(self.distdes)
        # print(self.curr_dist)
        # print(self.disterror)
        
        self.get_logger().info(msg) # presenta 'msg' en la terminal por medio del logger
        
        # print(f'theta goal = {self.theta_goal*180/pi}')
        # print(f'theta = {self.curr_theta*180/pi}')
        # print(f'ang vel = {self.vel.angular.z}')
        # print(self.lin)

    def move(self): 
        if self.ask ==1 : # si debe preguntar, entonces preguntar'a
            self.xdes = float(input('ENTER DESIRED X:')) # ingresa en 'xdes' o sea 'DESIRED X' el valor deseado para x
            self.ydes = float(input('ENTER DESIRED Y:')) # ingresa en 'ydes' o sea 'DESIRED Y' el valor deseado para y
            self.thetades = float(input('ENTER DESIRED THETA [deg]:'))*pi/180 # ingresa en 'thetades' o sea 'DESIRED THETA' el valor deseado para theta

        if self.reached_xy == 0: # si no ha alcanzado la posicion cartesiana aun entonces:

            # calculamos la distancia h entre el punto deseado y el punto actual de la tortuga 
            # este sera el error para el control de posicion cartesiana, observe que 
            # es variable en todo momento
            self.h = sqrt((self.xdes-self.curr_x)**2 + (self.ydes-self.curr_y)**2) 
            
            # define el 'theta goal' que es el angulo el cual debe tener la tortuga para moverse de manera lineal y llegar 
            # al punto deseado de manera cartesiana, ser'a el setpoint para el control angular, 
            # NOTESE que es distinto de theta deseado o 'THETADES', y que este 
            # TAMBI'EN es variable, al igual que h.
            self.theta_goal = atan2(self.ydes - self.curr_y, self.xdes - self.curr_x) # transforms it to {0} frame then computes theta
            
            # recalcula el error angular utilizando el theta goal  y el theta actual (set point variable).
            self.angerror = (self.theta_goal - self.curr_theta)
            
            # por temas de estabilidad de los sistemas de control (angular y cartesiano) los cuales est'an conectados, 
            # un gran error angular inestabiliza el sistema de control angular para theta goal y en consecuencia no logra 
            # llegar al punto deseado, para corregir el error se desarrolla la siguiente condicion inicial: 
            if abs(self.angerror*180/pi) > 150: # si el error angular es mayor a 150 grados entonces:
                self.vel.angular.z = self.angKp*abs(self.angerror) # asigna una velocidad angular como ley de control
                self.vel.linear.x = 0.5*self.posKp*self.h # asigna una velocidad lineal reducida como ley de control
                self.lin = 0 # setea el booleano .lin como falso (para que la tortuga se mueva mas lento linealmente de lo que se 
                # mueve angularmennte y logre reducir el error angular rapidamente)
                self.ask = 0 # no preguntar'a por nuevas coordenadas hasta no llegar a las coordenadas deseadas
            elif (abs(self.angerror) > 1e-03): # mientras el error angular sea mayor a 0.001, implementar'a control de pos. angular
                # para alcanzar el theta goal (y poder estar en la direccion correcta hacia el punto deseado)
                self.vel.angular.z = self.angKp*self.angerror # aplica la ley de control
                self.ask=0 # no preguntar'a
                self.lin=1 # ahora como el error angular es < 150 deg. podemos activar la ley de control 'comun' o 'normal' para 
                # el control lineal, es decir, no reducir su magnitud, ya no se necesita pq el error angular en conjunto con el 
                # lineal ya ser'ia manejable para los sistemas de control.
            else:
                self.vel.angular.z = 0.0 # si el error angular es menor a 0.001 apaga el control angular 

            # condicion para control lineal (solo se activa cuando el booleano .lin == True)
            if (abs(self.h) > 1e-03) and self.lin==1: # utiliza el mismo threshold o umbral de 0.001 para el error
                self.vel.linear.x = self.posKp*self.h # aplica la ley de control de posicion cartesiana
                self.ask=0 # no preguntar'a
            else:
                self.vel.linear.x = 0.0 # si el error lineal (o cartesiano) es menor a 0.001, apagar'a el control cartesiano.

            if abs(self.angerror) < 1e-03 and abs(self.h) < 1e-03: # si ambos errores tanto lineal como angular son menores a 0.001
                # activar'a el booleano reached_xy indicando que alcanz'o la posicion deseada en (x, y).
                self.reached_xy=1

        # SI REACHED_XY == TRUE significa que alcanz'o las coordenadas cartesianas deseadas.                        
        elif (abs(self.thetades - self.curr_theta) > 1e-04):  # calcula el error angular, le coloca un umbral de 0.0001
            self.angerror = (self.thetades - self.curr_theta) # aplica una ley de control angular pero en esta ocasi'on es para alcanzar 
            # el setpoint theta deseado y no 'theta goal', recordar lo mencionado anteriormente.
            self.vel.angular.z = self.angKp*self.angerror  # aplica ley de control
        else: # SI alcanz'o el setpoint angular (theta deseado) entonces:
            self.vel.linear.x = 0.0 # apaga velocidad lineal
            self.vel.angular.z = 0.0 # apaga velocidad angular
            self.ask=1 # preguntar'a la proxima vez que move() sea llamada (debido a que alcanzo posicion angular y posicion cartesiana)
            self.reached_xy=0 # resetea la variable que indica si si alcanz'o la coordenada cartesiana

        self.pub_cmdvel.publish(self.vel) # publica la velocidad como mensaje Twist


def main(args=None):
    rclpy.init(args=args) # inicializa el nodo
    node = TurtleCMD() # instancia la clase y crea el nodo 'node'

    rclpy.spin(node) # pone a 'correr' el nodo
    rclpy.shutdown() # apaga el nodo cuando sale del programa.

if __name__ == "__main__": # ejecuta la funcion main() S'OLO SI se ejecuta el nodo, pero si el nodo es importado desde otro script
    # esta funcion main() no es ejecutada.
    main()