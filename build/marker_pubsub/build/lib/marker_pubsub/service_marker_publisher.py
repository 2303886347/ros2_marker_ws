import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from rclpy.duration import Duration


class ServiceMarkerPublisher(Node):
    def __init__(self):
        super().__init__('service_marker_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        self.srv = self.create_service(Trigger, 'add_marker', self.add_marker_callback)

    def add_marker_callback(self, request, response):
        marker = Marker()
        marker.header.frame_id = "map"  # 使用 map 坐标系
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # 设置标记的位置
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
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

        response.success = True
        response.message = "Marker added!"
        self.get_logger().info('Marker added!')
        return response

def main(args=None):
    rclpy.init(args=args)
    service_marker_publisher = ServiceMarkerPublisher()
    rclpy.spin(service_marker_publisher)
    service_marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
