import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 获取默认路径
    fishbot_node_path = get_package_share_directory('fishbot')  # share_path = install/fishbot/share/fishbot
    default_model_path = fishbot_node_path + '/urdf/fishbot.xacro'
    default_world_path = fishbot_node_path + '/world/custom_room.world'

    # urdf的绝对路径
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径')
    
    # 在launch的时候用xacro命令把xacro编译成urdf
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str)
    
  	# robot_state_publisher_node（在gazebo仿真环境中，不需要joint_state_publisher_node）
    # robot_state_publisher 可以通过话题 /robot_description 把urdf的内容发布出来
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    # 通过 IncludeLaunchDescription 来启动另外一个 launch 文件（这里启动gazebo里自带的launch）
    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
      	# 传递参数
        launch_arguments=[('world', default_world_path),('verbose','true')]
    )
    
    # 请求 Gazebo 加载机器人
    robot_name_in_model = "fishbot"
    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',  # 这个参数指定了 Gazebo 从 ROS 的 /robot_description 话题中获取机器人的模型信息
                   '-entity', robot_name_in_model, ]
    )
    
    # 加载并激活 fishbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'fishbot_joint_state_broadcaster'],
        output='screen')

    # 差速控制器和力控制器都是可以控制机器人运动，两个有一个就行
    # 加载并激活 fishbot_effort_controller 力控制器
    load_fishbot_effort_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_effort_controller'], 
        output='screen')

    # 加载并激活 fishbot_effort_controller 差速控制器
    load_fishbot_diff_drive_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','fishbot_diff_drive_controller'], 
        output='screen')


    return launch.LaunchDescription([
        action_declare_arg_mode_path,
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node,
        # 事件动作，当加载机器人结束后执行    
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller]
            )
        ),
        # 事件动作，load_fishbot_diff_drive_controller
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_fishbot_diff_drive_controller]
            )
        )
    ])
