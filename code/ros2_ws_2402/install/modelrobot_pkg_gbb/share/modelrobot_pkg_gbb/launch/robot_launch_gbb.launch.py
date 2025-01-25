# Permite tomar informacion del sistema operativo (direcciones)
import os # importa libreria de os para unir al path el archivo urdf.

from ament_index_python.packages import get_package_share_path 

# Se necesitan para habilitar los nodos
from launch import LaunchDescription
from launch_ros.actions import Node # importa clase Nodo de paquete launch_ros.actions

# Clases necesarias para la ejecucion de los nodos en relacion al URDF
# permiten cargar nodos con configuraciones especiales
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro # importa lib xacro para leer el archivo .urdf.xacro (normalmente solo se podrian leer archivos URDF)

# almacenamos el nombre del paquete y del archivo .urdf.xacro
pkg_folder = 'modelrobot_pkg_gbb'
robot_file = 'robot_car_g02.urdf.xacro'

def generate_launch_description():
    # Se determina la direccion que el compilador le asigno al paquete en el sistema    
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    
    # Se concatena la direccion del paquete con /model/ y con el nombre del archivo de robot    
    default_model_path = os.path.join(pkg_path + '/model/' + robot_file)

    # Esta declaracion permite habilitar o deshabilitar la interfaz empleada mas adelante
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file
    # Esta linea permite usar la libreria xacro, para procesar el archivo y convertirlo en URDF 
    robot_description_config = xacro.process_file(default_model_path)

    # Guardamos la descripcion del robot en la variable params con el siguiente formato
    params = {'robot_description': robot_description_config.toxml()}

    # Este publica el estado actual del robot, esta diseñado para leer archivos URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        # Aqui se guarda toda la informacion necesaria sobre la configuracion del robot        
        parameters=[params]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # Este ayuda a publicar los joints o articulaciones y su estado
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        # Quiere decir que este actuara a menos que se inicialice el GUI        
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    # Ventana que permite modificar el estado de las articulaciones (valores de las ruedas)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        # Condicion que permite que ocurre el nodo cuando deseemos que este         
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # Launch!
    # Se cargan cada uno de los objetos    
    return LaunchDescription([
        gui_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
])