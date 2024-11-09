import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径：获取 `fishbot_navigation2` 和 `nav2_bringup` 包的共享目录
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 指定 RViz 配置文件的路径，默认为 `nav2_bringup` 包中的默认配置
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    
    # 设置 Launch 配置的默认值
    # `use_sim_time`：决定是否使用仿真时钟（例如 Gazebo）
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='true')
    
    # `map_yaml_path`：加载地图文件的路径，默认为 `fishbot_navigation2` 包中的 `room.yaml`
    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map', default=os.path.join(fishbot_navigation2_dir, 'maps', 'room.yaml'))
    
    # `nav2_param_path`：加载 Nav2 参数文件的路径，默认为 `fishbot_navigation2` 包中的 `nav2_params.yaml`
    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file', default=os.path.join(fishbot_navigation2_dir, 'config', 'nav2_params.yaml'))

    return launch.LaunchDescription([
        # 声明新的 Launch 参数，用于命令行传参时的配置
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),  # 是否使用仿真时钟
        launch.actions.DeclareLaunchArgument(
            'map', default_value=map_yaml_path,
            description='Full path to map file to load'),  # 地图文件路径
        launch.actions.DeclareLaunchArgument(
            'params_file', default_value=nav2_param_path,
            description='Full path to param file to load'),  # Nav2 参数文件路径

        # 启动 Nav2 bringup 配置，包含导航的核心功能和配置文件
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        ),
        
        # 启动 RViz 2 用于可视化，展示导航相关的内容
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],  # 使用指定的 RViz 配置文件
            parameters=[{'use_sim_time': use_sim_time}],  # 将 `use_sim_time` 参数传递给 RViz
            output='screen'),  # 输出到屏幕上
    ])
