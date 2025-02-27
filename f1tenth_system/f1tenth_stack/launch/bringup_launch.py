from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch the real car system.
    """
    
    # declare arguments
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
    vesc_config = LaunchConfiguration('vesc_config')
    sensors_config = LaunchConfiguration('sensors_config')
    mux_config = LaunchConfiguration('mux_config')
    
    
    # electronic speed controller    
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[vesc_config]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[vesc_config]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[vesc_config]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[mux_config],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[vesc_config]
    )
    
    
    # laser scanner 
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[sensors_config]
    )
    
    
    # static transformations of the real car
    static_tf_node_bl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_to_laser',
        arguments=['0.27', '0.0', '0.13', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )
    
    static_tf_node_mo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom']
    )
    
    static_tf_node_bi = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_to_imu',
        arguments=['0.08', '0.0', '0.08', '0.0', '0.0', '0.7071068', '0.7071068', 'base_link', 'imu']
    )


    # launch description
    ld = LaunchDescription()
    ld.add_action(vesc_config_arg)
    ld.add_action(sensors_config_arg)
    ld.add_action(mux_config_arg)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    # ld.add_action(throttle_interpolator_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node_bl)
    ld.add_action(static_tf_node_mo)
    ld.add_action(static_tf_node_bi)

    return ld
