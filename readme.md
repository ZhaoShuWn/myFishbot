# ROS 2 fishbot

### 1. **工作空间结构**

一个标准的 ROS2 工作空间通常有以下结构：

```
ros2_ws/                   # 工作空间根目录
├── src/                   # 包含所有 ROS2 包的源代码目录
├── build/                 # 编译输出文件夹，由 colcon 自动生成
├── install/               # 安装文件夹，包含各包的构建结果，供运行时使用
├── log/                   # 运行和构建日志文件
└── colcon.meta            # colcon 的配置文件（可选）
```

### 2. **ROS2 包的基本结构**

在 `src` 目录下，每个子文件夹通常代表一个 ROS2 包，每个包的标准结构如下：

```
my_robot_package/           # ROS2 包的名称
├── package.xml             # 包的描述文件，定义依赖项、作者信息等
├── CMakeLists.txt          # 构建系统配置文件，用于指定构建指令
├── setup.py                # 如果是 Python 包，setup.py 用于定义构建和安装过程
├── src/                    # 包含源代码的文件夹（Python 或 C++ 文件）
│   ├── my_robot_node.cpp   # 示例源代码文件（C++）
│   └── my_robot_node.py    # 示例源代码文件（Python）
├── launch/                 # 启动文件目录，包含用于启动系统的文件
│   └── my_launch_file.launch.py
├── urdf/                   # 机器人的 URDF 文件，用于描述机器人模型
│   └── my_robot.urdf
├── config/                 # 配置文件夹，包含 YAML 等配置文件
│   └── my_config.yaml
└── rviz/                   # RViz 配置文件目录，用于可视化的设置
    └── my_robot.rviz
```

### 3. **示例：运行一个节点**

如果你有一个 `launch` 文件 `my_launch_file.launch.py`，可以通过以下命令运行：

```
ros2 launch my_robot_package my_launch_file.launch.py
```

