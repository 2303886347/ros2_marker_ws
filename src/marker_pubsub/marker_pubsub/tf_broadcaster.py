import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration

class TransformBroadcasterNode(Node):

    def __init__(self):
        super().__init__('transform_broadcaster')
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'marker_frame'  # 这是子框架的名字，你可以把它设为任何你想要的
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.w = 1.0  # 没有旋转

        self.broadcaster.sendTransform(t)
        self.get_logger().info('Broadcasting transformation from map to marker_frame')

def main(args=None):
    rclpy.init(args=args)
    broadcaster = TransformBroadcasterNode()
    rclpy.spin(broadcaster)
    broadcaster.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
