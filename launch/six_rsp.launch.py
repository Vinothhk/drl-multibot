import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_description = "turtlebot"
    print("Fetching URDF ==>")
    robot_desc_path_1 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_1.urdf')
    robot_desc_path_2 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_2.urdf')
    robot_desc_path_3 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_3.urdf')
    robot_desc_path_4 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_4.urdf')
    robot_desc_path_5 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_5.urdf')
    robot_desc_path_6 = os.path.join(get_package_share_directory(package_description), 'urdf/burger_6.urdf')
    
    rps_1_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='bot_1',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_1/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_1, ' robot_name:=','bot_1'])
                    }],
        output="screen"
    )
    rps_2_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='bot_2',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_2/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_2,' robot_name:=','bot_2'])
                    }],
        output="screen"
    )
    
    rps_3_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='bot_3',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_3/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_3,' robot_name:=','bot_3'])
                    }],
        output="screen"
    )
    rps_4_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='bot_4',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_4/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_4, ' robot_name:=','bot_4'])
                    }],
        output="screen"
    )
    rps_5_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='bot_5',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_5/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_5,' robot_name:=','bot_5'])
                    }],
        output="screen"
    )
    
    rps_6_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='bot_6',
        emulate_tty=True,
        parameters=[{
                    'frame_prefix':'bot_6/',
                    'use_sim_time': True,
                    'robot_description': Command(['xacro ',robot_desc_path_6,' robot_name:=','bot_6'])
                    }],
        output="screen"
    )
    
    
    
    
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )
# create and return launch description object
    return LaunchDescription([
        rps_1_node,
        rps_2_node,
        rps_3_node,
        rps_4_node,
        rps_5_node,
        rps_6_node,
        rviz_node
    ])