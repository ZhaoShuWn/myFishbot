import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory # 用于获取指定包的共享目录路径


# 这个函数是固定要写的，函数名也得保持一致
def generate_launch_description():
    share_path = get_package_share_directory('fishbot')
    
    default_model_path = share_path + '/urdf/first_robot.urdf'
    
    # 构建 RViz 配置文件的默认路径，`urdf_tutorial_path` 后加上 `/config/rviz/display_model.rviz`
    default_rviz_config_path = urdf_tutorial_path + '/config/rviz/display_model.rviz'
    
    # 创建 `DeclareLaunchArgument` 动作，用于声明一个名为 `model` 的 launch 参数
    # 参数 `name` 是参数名称，`default_value` 是默认值，`description` 提供参数的描述信息
    action_declare_arg_mode_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(default_model_path),
        description='URDF 的绝对路径'
    )
    
    # 创建 `ParameterValue` 对象，`robot_description` 是机器人模型描述的参数
    # 通过 `Command` 读取 xacro 文件生成参数值，使用 `LaunchConfiguration` 获取 `model` 参数值
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),  # `Command` 类用于在运行时执行命令行命令
        value_type=str  # `value_type` 指定参数类型为字符串
    )
    
    # 创建 `Node` 动作，用于启动 `robot_state_publisher` 节点，该节点用于发布机器人状态信息
    # `package` 是节点所属的包名，`executable` 是要执行的可执行文件，`parameters` 是节点参数列表
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]  # 提供包含 `robot_description` 参数的字典
    )
    
    # 创建 `Node` 动作，用于启动 `joint_state_publisher` 节点，该节点用于发布机器人关节状态
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',  # 所属包名
        executable='joint_state_publisher',  # 要执行的可执行文件
    )
    
    # 创建 `Node` 动作，用于启动 RViz 节点，用于可视化机器人模型
    # `arguments` 用于传递启动时的命令行参数，这里是 `-d` 和 RViz 配置文件路径
    rviz_node = launch_ros.actions.Node(
        package='rviz2',  # 所属包名
        executable='rviz2',  # 要执行的可执行文件
        arguments=['-d', default_rviz_config_path]  # 启动时传递 RViz 配置文件路径作为参数
    )
    
    # 返回一个 `LaunchDescription` 对象，其中包含所有要启动的动作
    # `LaunchDescription` 类将多个动作对象打包在一起，作为 launch 文件的输出
    return launch.LaunchDescription([
        action_declare_arg_mode_path,  # 声明参数动作
        joint_state_publisher_node,    # 启动关节状态发布节点
        robot_state_publisher_node,    # 启动机器人状态发布节点
        rviz_node                      # 启动 RViz 节点
    ])
