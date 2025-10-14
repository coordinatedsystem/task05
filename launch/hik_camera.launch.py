from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    params_file_path = os.path.join(
        get_package_share_directory('hik_camera'),
        'config',
        'camera_params.yaml'
    )

    # 获取 rviz 配置文件路径
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('hik_camera'),
        'config',
        'rviz_config.rviz'
    ])

    return LaunchDescription([
        # 启动海康相机节点
        Node(
            package='hik_camera',
            executable='hik_camera_node',
            name='hik_camera',
            output='screen',
            parameters=[params_file_path]
        ),

        # 启动灯条检测节点
        Node(
            package='hik_camera',
            executable='light_strip_detector',
            name='light_strip_detector',
            output='screen'
            # 如果需要传递参数，可以在这里添加 parameters 参数
            # parameters=[...]
        ),
        
        # 启动 RViz2 并加载配置文件
        ExecuteProcess(
            cmd=[
                'rviz2',  
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('hik_camera'),
                    'config',
                    'rviz_config.rviz'
                ])
            ],
            name='rviz2',
            output='screen'
        )
    ])