import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # [MODIFIED] Use standard Trigger service
from geometry_msgs.msg import Twist
import sys
import subprocess
import os
import random
import math

# [ADDED] Class to generate random spawn poses
class respawn_pose_node:
    def __init__(self, map_w=30.0, map_h=30.0): # Default values from env_node
        self.MAP_WIDTH = map_w
        self.MAP_HEIGHT = map_h
        self.RADIUS = 2.5
        self.MAP_WALL_SIZE = {'width': 2, 'height': 28}

        self.info_of_obstacles = [
            {'x': 7.5,'y': 7.5, 'dx': 1.0 + self.RADIUS, 'dy':1.0 + self.RADIUS }, # box1
            {'x': 7.0,'y': -10.0, 'dx': 1.0 + self.RADIUS, 'dy':1.0 + self.RADIUS }, # box2
            {'x': -4.0,'y': 0, 'dx': 1.0 + self.RADIUS, 'dy':1.0 + self.RADIUS }, # box3
            {'x': -4.0,'y': -7.0, 'dx': 1.0 + self.RADIUS, 'dy':1.0 + self.RADIUS }, # box4
            {'x': -8.0,'y': 8.0, 'dx': 1.0 + self.RADIUS, 'dy':1.0 + self.RADIUS }, # box5
            {'x': 5.0,'y': -3.0, 'dx': 0.5 + self.RADIUS, 'dy':4.0 + self.RADIUS }, # wall1
            {'x': 0.0,'y': 3.0, 'dx': 0.5 + self.RADIUS, 'dy':4.0 + self.RADIUS }, # wall2
        ]

    def get_safe_pose(self, use_random=True):
        d_map_wall_size = self.MAP_WALL_SIZE['width'] / 2.0
        width_limit = self.MAP_WIDTH / 2.0 - d_map_wall_size - self.RADIUS
        height_limit = self.MAP_HEIGHT / 2.0 - d_map_wall_size - self.RADIUS

        if use_random == False:
            return (-10.0, 0.0, 0.0)
        
        while True:
            x = random.uniform(-width_limit, +width_limit)
            y = random.uniform(-height_limit, +height_limit)

            conflict = False 
            for obstacle in self.info_of_obstacles:
                if abs(x - obstacle['x']) < obstacle['dx'] and abs(y - obstacle['y']) < obstacle['dy'] :
                    conflict = True
                    break
            
            if not conflict:
                if math.hypot(x - 10.0, y - 0.0) < 3.0:
                    continue

                return (x, y, 0.0)


class ServiceNode(Node):

    def __init__(self):
        super().__init__('ServiceNode')
        # [MODIFIED] Use Trigger service type
        self.srv = self.create_service(Trigger, 'reset', self.reset_callback)
        self.vel_pub = self.create_publisher(Twist,'/cmd_vel', 10)
        
        # [ADDED] Initialize pose generator
        self.respawn_generator = respawn_pose_node()
        self.get_logger().info('서비스 노드 시작됨 (Standard Trigger Interface).')

    def reset_callback(self, request, response):
        # [MODIFIED] Generate pose internally
        x, y, z = self.respawn_generator.get_safe_pose(use_random=False)
        self.get_logger().info(f'랜덤 이동할 위치 생성: x={x:.2f}, y={y:.2f}, z={z:.2f}')
        
        # 속도 초기화
        move_msg = Twist()
        move_msg.linear.x = 0.0
        move_msg.angular.z = 0.0
        self.vel_pub.publish(move_msg)

        # Gazebo 서비스를 통해 차량 위치 설정
        env = os.environ.copy()
        env['IGN_PARTITION'] = 'david_sim'
        
        # set_pose 서비스 호출
        pose_cmd = f'''ign service -s /world/my_car_world/set_pose \
            --reqtype ignition.msgs.Pose \
            --reptype ignition.msgs.Boolean \
            --timeout 1000 \
            --req "name: 'car', position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{x: 0, y: 0, z: 0, w: 1}}"'''
        
        try:
            result = subprocess.run(pose_cmd, shell=True, env=env, capture_output=True, text=True, timeout=3)
            if result.returncode == 0:
                self.get_logger().info('Gazebo set_pose 서비스 호출 성공')
                # [MODIFIED] Set success message in standard response
                response.success = True
                response.message = f"Reset done to ({x:.2f}, {y:.2f})"
            else:
                self.get_logger().warn(f'Gazebo set_pose 실패: {result.stderr}')
                response.success = False
                response.message = f"Reset failed: {result.stderr}"
        except subprocess.TimeoutExpired:
            self.get_logger().warn('Gazebo set_pose 타임아웃')
            response.success = False
            response.message = "Reset timed out"
        except Exception as e:
            self.get_logger().error(f'Gazebo set_pose 오류: {e}')
            response.success = False
            response.message = f"Reset error: {e}"

        self.get_logger().info('가제보에 reset명령 전달 완료함.')

        return response

def main(args=None):
    rclpy.init(args=args)
    service_node = ServiceNode()
    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()