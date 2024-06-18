import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('mobile_manipulator_bringup'),
        'config',
        'params.yaml'
    )

    ur_driver_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_robot_driver'),
                         'launch/ur5.launch.py')
        ),
        launch_arguments={
            'launch_rviz':'false',
            'ur_type':'ur5',
            'use_fake_hardware':'true',
            'robot_ip':'192.168.1.180',
            # 'initial_joint_controller':'scaled_joint_trajectory_controller'
        }.items()
    )
    ld.add_action(ur_driver_launch_file)

    ur_moveit_launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ur_moveit_config'),
                         'launch/ur_moveit.launch.py')
        ),
        launch_arguments={
            'launch_rviz':'true',
            'ur_type':'ur5',
            'use_fake_hardware':'false',
            'robot_ip':'192.168.1.180',
            # 'initial_joint_controller':'scaled_joint_trajectory_controller'
        }.items()
    )
    ld.add_action(ur_moveit_launch_file)

    target_position_node = Node(
        package='target_position',
        executable='target_position_node',
        parameters=[config]
    )
    ld.add_action(target_position_node)
    
    collision_objects_node = Node(
        package='collision_objects',
        executable='collision_objects_node',
        parameters=[config]
    )
    ld.add_action(collision_objects_node)

    base_movement_node = Node(
        package='base_movement',
        executable='base_movement_node',
        parameters=[config]
    )
    ld.add_action(base_movement_node)

    path_planner_node = Node(
        package='path_planning',
        executable='dynamic_path_planner_node'
    )
    ld.add_action(path_planner_node)

    tool0_tf_node = Node(
        package='logger',
        executable='tool0_tf_node'
    )
    ld.add_action(tool0_tf_node)

    tf_logger_node = Node(
        package='logger',
        executable='tf_logger_node',
        parameters=[config]
    )
    ld.add_action(tf_logger_node)

    return ld
    
