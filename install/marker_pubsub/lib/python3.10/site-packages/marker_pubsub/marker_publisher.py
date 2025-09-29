import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from rclpy.duration import Duration

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒发布一次
        self.i = 0
        self.side_length = 2.0  # 正方形边长，可自由修改
        self.speed = 0.1  # 每次移动的距离
        self.current_edge = 0  # 当前边编号（0~3）
        self.edge_progress = 0.0  # 当前边已走距离
        self.x = 1.0  # 起点x
        self.y = 0.0  # 起点y

    def timer_callback(self):
        # 计算当前位置
        if self.current_edge == 0:
            self.x += self.speed
        elif self.current_edge == 1:
            self.y += self.speed
        elif self.current_edge == 2:
            self.x -= self.speed
        elif self.current_edge == 3:
            self.y -= self.speed

        self.edge_progress += self.speed
        if self.edge_progress >= self.side_length:
            self.current_edge = (self.current_edge + 1) % 4
            self.edge_progress = 0.0

        marker = Marker()
        marker.header.frame_id = "map"  # 参考坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        marker.color.r = 1.0  # 红色
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # 不透明

        marker.lifetime = Duration(seconds=0).to_msg()  # 永久存在

        self.publisher_.publish(marker)
        self.get_logger().info(f'Publishing Marker: edge={self.current_edge}, x={self.x:.2f}, y={self.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    publisher = MarkerPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
