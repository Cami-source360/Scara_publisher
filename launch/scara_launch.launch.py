import os

from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

import xacro

pkg_folder = 'scara'
robot_file = 'scara_digital_twin.urdf.xacro'

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_path(pkg_folder))
    default_model_path = os.path.join(pkg_path + '/models/' + robot_file)

    #habilita o deshabilita el GUI para modificar las articulaciones
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'], description='Flag to enable joint_state_publisher_gui')

    # Process the URDF file. convierte xacro to URDF
    robot_description_config = xacro.process_file(default_model_path)

    #almacena el URDF procesado en un diccionario
    # Este diccionario se utiliza para configurar el nodo robot_state_publisher
    # que es responsable de publicar el estado del robot en el topic /robot_state
    params = {'robot_description': robot_description_config.toxml()}

    # diseñado para leer archivos URDF y publicar el estado del robot
    # Este nodo toma el URDF y lo convierte en un formato que ROS puede entender    
    # y luego publica el estado del robot en el topic /robot_state
    # Este nodo es esencial para que otros nodos puedan interactuar con el robot
    # Por ejemplo, el nodo de control del robot necesita este estado para saber cómo mover las articulaciones
    # y el nodo de visualización necesita este estado para mostrar el robot en RV
    # Este nodo es el corazón del sistema de control del robot
    twist_mux_params = os.path.join(get_package_share_path(pkg_folder),'config','mux_config.yaml')
    
    twist_mux_node = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params],
                    remappings=[('/cmd_vel_out','/twist_mux_out')]
    )

    twist_mux_params2 = os.path.join(get_package_share_path(pkg_folder),'config','mux_config2.yaml')
    
    twist_mux_node2 = Node(package='twist_mux', 
                    executable='twist_mux',
                    parameters=[twist_mux_params2],
                    remappings=[('/cmd_vel_out','/inv_kin_mux')]
    )


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher', #publica articulaciones
        executable='joint_state_publisher', #publica estado de articulaciones
        condition=UnlessCondition(LaunchConfiguration('gui')) # si no se inicializa el GUI, se lanza este nodo
    )

    # permite modificar las articulaciones de forma manual
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration('gui')) # en caso de que no se inicalice el GUI, no se lanza este nodo
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )


    inverse_kinematics = Node(
        package='scara',
        executable='inverse_kinematics',
        name='inv_kinematic',
        output='screen',
    )

    direct_kinematics = Node(
        package='scara',
        executable='direct_kinematics',
        name='dir_kinematic',
        output='screen',
    )



    # Launch!
    return LaunchDescription([
        gui_arg,
        # joint_state_publisher_node,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        inverse_kinematics,
        twist_mux_node,
        twist_mux_node2,
        direct_kinematics
    ])