from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch the real car system with a joystick node for teleop-driving.
    """
    
    # declare arguments
    joy_teleop_config_arg = DeclareLaunchArgument(
        'joy_teleop_config',
        default_value=os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'joy_teleop.yaml'),
        description='path to the joy_teleop config file.'
    )
    vesc_config_arg = DeclareLaunchArgument(
        'vesc_config',
        default_value=os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'vesc.yaml'),
        description='path to the vesc config file.'
    )
    sensors_config_arg = DeclareLaunchArgument(
        'sensors_config',
        default_value=os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'sensors.yaml'),
        description='path to the sensors config file.'
    )
    mux_config_arg = DeclareLaunchArgument(
        'mux_config',
        default_value=os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'mux.yaml'),
        description='path to the mux config file.'
    )
    joy_teleop_config = LaunchConfiguration('joy_teleop_config')
    vesc_config = LaunchConfiguration('vesc_config')
    sensors_config = LaunchConfiguration('sensors_config')
    mux_config = LaunchConfiguration('mux_config')
    
    
    # joystick nodes for teleop-driving
    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_linux_node',
        parameters=[joy_teleop_config]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[joy_teleop_config]
    )
    
    
    # include bringup launch
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('f1tenth_stack'), 'launch', 'bringup_launch.py')),
        launch_arguments={
            'vesc_config': vesc_config,
            'sensors_config': sensors_config,
            'mux_config': mux_config
        }.items()
    )  


    # launch description
    ld = LaunchDescription()
    ld.add_action(joy_teleop_config_arg)
    ld.add_action(vesc_config_arg)
    ld.add_action(sensors_config_arg)
    ld.add_action(mux_config_arg)
    ld.add_action(joy_teleop_node)
    ld.add_action(bringup_launch)
    ld.add_action(joy_linux_node)    
    return ld
