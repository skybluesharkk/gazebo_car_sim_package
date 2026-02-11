#!/usr/bin/env python3
"""
Odometry to TF Publisher
odom 메시지를 받아서 odom -> car TF를 발행하는 노드
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_publisher')
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/car/odometry',
            self.odom_callback,
            10
        )
        
        self.get_logger().info('Odom to TF Publisher started')
    
    def odom_callback(self, msg):
        """Odometry 메시지를 받아 TF로 변환"""
        t = TransformStamped()
        
        # Header 설정
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'car/chassis/chassis_link'
        
        # Translation (위치)
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Rotation (회전)
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # TF 발행
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
