# ROS2学习-fishbot

fishbot是一个扫地机器人样式的小车，带有激光雷达、摄像机什么的。

### 1. **各个文件为了完成的任务**

1. demo01的目的是为了建模一个小车，能够在gazebo中运动（不带任何传感器）
1. demo02的目的是在demo01基础上再加上fishbot的各种传感器
1. demo03的目的是实现用demo02中的小车来进行navigation路径规划

### 2. 在实现过程中需要记录的一些内容

### 2.1 该项目的话题结构解析

1. 写好urdf文件，然后通过robot_state_publisher功能包来发布关节的静态结构。通过joint_state_publisher来发布关节的结构。

```python
# robot_description实际上是一个字符串，这个字符串就是urdf的全部内容
robot_description = launch_ros.parameter_descriptions.ParameterValue(
    launch.substitutions.Command(
    ['xacro ', launch.substitutions.LaunchConfiguration('model')]),  # Command类在运行时执行命令行命令
    value_type=str  # `value_type` 指定参数类型为字符串
)

# robot_state_publisher_node 订阅 /joint_states 主题，从中获取关节的动态状态（如位置、速度），然后将这些关节状态信息转化为机器人的姿态（坐标变换）。这些坐标信息以 TF 坐标系的形式发布在 ROS 网络中。
robot_state_publisher_node = launch_ros.actions.Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description}]  # 将参数传递给节点 `robot_description`
)
```

2. 在gazebo中加载机器人

```
# 请求 Gazebo 加载机器人
robot_name_in_model = "fishbot"
spawn_entity_node = launch_ros.actions.Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=['-topic', '/robot_description',  # 这个参数指定了 Gazebo 从/robot_description话题中获取模型信息
	'-entity', robot_name_in_model, ]
)
```


3. 我们现在需要发布关节信息了

```
# 加载并激活 fishbot_joint_state_broadcaster 控制器
# 我们在这里发布关节信息，只要发布里关节信息，robot_state_publisher_node 会自动来订阅
load_joint_state_controller = launch.actions.ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'fishbot_joint_state_broadcaster'], 	output='screen'
)
```



### 2.2 一些可能需要用到的指令

```bash
# 用键盘来控制机器人
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# 启动slam-toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
