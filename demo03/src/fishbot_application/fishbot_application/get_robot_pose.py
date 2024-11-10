import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer  # 导入 tf2_ros 中的 TransformListener 和 Buffer 类，分别用于监听变换和缓存变换数据
from tf_transformations import euler_from_quaternion  # 导入 tf_transformations 中的 euler_from_quaternion 函数，用于将四元数转换为欧拉角

# 定义一个 TFListener 类，继承自 rclpy.node.Node，表示一个 ROS 2 节点
class TFListener(Node):

    # 构造函数：初始化节点，设置缓冲区和变换监听器，设置定时器来定期获取变换
    def __init__(self):
        # 调用父类 Node 的构造函数，传入节点名称 'tf2_listener'（就算给这个代码的节点命名）
        super().__init__('tf2_listener')
        
        # 创建一个 Transform Buffer（缓冲区），用于存储变换信息
        self.buffer = Buffer()
        
        # 创建一个 TransformListener 对象，负责监听变换数据，传入缓冲区和当前节点（self）作为参数
        self.listener = TransformListener(self.buffer, self)
        
        # 创建一个定时器，每 1 秒调用一次 get_transform 方法
        # 这里的定时器周期为 1 秒（1000 毫秒），回调函数是 get_transform
        self.timer = self.create_timer(1, self.get_transform)

    # 获取坐标变换的函数
    def get_transform(self):
        try:
            # 获取从 'map' 坐标系到 'base_footprint' 坐标系的变换
            # 使用 lookup_transform 方法，传入目标坐标系、源坐标系、时间戳（Time(seconds=0) 表示当前时间），和超时（Duration(seconds=1) 设置为 1 秒）
            tf = self.buffer.lookup_transform(
                'map',  # 目标坐标系
                'base_footprint',  # 源坐标系
                rclpy.time.Time(seconds=0),  # 当前时间戳，表示获取最新的变换
                rclpy.time.Duration(seconds=1)  # 变换查询超时时间，设置为 1 秒
            )

            # 从变换中提取平移（translation）和旋转（rotation）信息
            transform = tf.transform  # transform 包含平移（translation）和旋转（rotation）

            # 将旋转的四元数转换为欧拉角
            # 四元数格式为 [x, y, z, w]，其中 w 为标量部分，x, y, z 为向量部分
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,  # x 分量
                transform.rotation.y,  # y 分量
                transform.rotation.z,  # z 分量
                transform.rotation.w   # w 分量
            ])

            # 打印平移、旋转四元数和旋转欧拉角
            # 使用 info 级别日志记录，显示平移（translation），旋转四元数（rotation）和旋转欧拉角（rotation_euler）
            self.get_logger().info(
                f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}'
            )
        except Exception as e:
            # 如果获取变换失败，打印警告信息，显示错误原因
            self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')

# main 函数：ROS 2 节点的入口，初始化节点并开始运行
def main():
    # 初始化 rclpy，准备开始 ROS 2 通信
    rclpy.init()

    # 创建 TFListener 节点实例
    node = TFListener()

    # 通过 rclpy.spin() 让节点开始监听消息和回调
    # 这将使程序持续运行，直到节点关闭
    rclpy.spin(node)

    # 关闭节点并清理资源
    rclpy.shutdown()

# 程序入口：如果这个脚本是直接运行的（而不是作为模块导入），则调用 main 函数
if __name__ == '__main__':
    main()
