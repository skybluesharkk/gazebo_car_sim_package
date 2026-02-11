from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Twist
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rclpy
from rclpy.node import Node

from std_srvs.srv import Trigger  # 표준 Trigger 서비스 사용
from std_msgs.msg import String 
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.msg import Log
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from ros_gz_interfaces.msg import Contacts

import math
import time
import cv2
import numpy as np
import random


class EnvNode(Node):

    def __init__(self):
        super().__init__('EnvNode')
        
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('img_width', 64)
        self.declare_parameter('img_height', 64)
        self.declare_parameter('map_width', 30.0)
        self.declare_parameter('map_height', 30.0)
        self.declare_parameter('max_step', 1000)
        self.declare_parameter('max_episode', 1000)
        self.MAX_RANGE = self.get_parameter('max_range').value
        self.IMG_WIDTH = self.get_parameter('img_width').value
        self.IMG_HEIGHT = self.get_parameter('img_height').value
        self.MAP_WIDTH = self.get_parameter('map_width').value
        self.MAP_HEIGHT = self.get_parameter('map_height').value
        self.MAX_STEP = self.get_parameter('max_step').value
        self.MAX_EPISODE = self.get_parameter('max_episode').value

        self.cli_env_reset = self.create_client(Trigger, 'reset')

        # 구독자 생성
        self.sub_lidar = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 1)
        self.sub_camera = self.create_subscription(Image,'/camera',self.camera_callback,1)
        self.sub_imu = self.create_subscription(Imu,'/imu',self.imu_callback,1)
        self.sub_collision_chassis = self.create_subscription(Contacts, '/collision_chassis', self.collision_chassis_callback, 10)
        self.sub_collision_camera = self.create_subscription(Contacts, '/collision_camera', self.collision_camera_callback, 10)
        self.sub_collision_tire_lf = self.create_subscription(Contacts, '/collision_tire_lf', self.collision_tire_lf_callback, 10)
        self.sub_collision_tire_rf = self.create_subscription(Contacts, '/collision_tire_rf', self.collision_tire_rf_callback, 10)
        self.sub_collision_tire_lr = self.create_subscription(Contacts, '/collision_tire_lr', self.collision_tire_lr_callback, 10)
        self.sub_collision_tire_rr = self.create_subscription(Contacts, '/collision_tire_rr', self.collision_tire_rr_callback, 10)
        self.sub_model_car_odometry = self.create_subscription(Odometry,'/model/car/odometry',self.model_car_odometry_callback,1)
        self.sub_world_pose = self.create_subscription(TFMessage,'/world/my_car_world/pose/info',self.world_car_pose_callback,1)

        # 발행자
        self.pub_cmd_vel = self.create_publisher(Twist,'/cmd_vel',10)

        self.req = Trigger.Request()
        self.goal_position = [10,0,0]
        self.latest_lidar = None
        self.latest_lidar_raw = None
        self.latest_image = None
        self.latest_imu   = None
        self.collision_flag = 0.0
        self.collision_sensor = None
        self.is_arrived_at_goal = 0.0
        
        # Stuck 감지 변수
        self.stuck_count = 0
        self.STUCK_THRESHOLD = 50  # 50스텝(5초) 연속 미이동 시 Stuck 판정
        self.is_stuck = False
        
        self.latest_odometry_info = None
        self.world_car_pose = None
        self.prev_pose = None
        self.prev_dist = None
        self.steps_done = 0

    def _spin_all(self, total_sec=0.1, each_timeout=0.01):
        """
        주어진 시간(total_sec) 동안 대기 중인 모든 콜백을 처리합니다.
        기존 spin_once 1회는 콜백 1개만 처리하므로, 12개 이상의 구독자가 있을 때
        world_car_pose 등 중요한 콜백이 누락되는 문제를 해결합니다.
        """
        deadline = time.time() + total_sec
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=each_timeout)

    def camera_callback(self,msg):
        raw_img_data = np.frombuffer(msg.data, dtype=np.uint8)
        reshaped_img_data = raw_img_data.reshape(msg.height, msg.width, 3)
        resized_image_data = cv2.resize(reshaped_img_data,(self.IMG_WIDTH,self.IMG_HEIGHT))
        
        # [0, 255] → [-1, 1] 정규화
        resized_image_data = resized_image_data.astype(np.float32) / 255.0
        resized_image_data = (resized_image_data - 0.5) / 0.5
        self.latest_image = np.nan_to_num(resized_image_data, nan=0.0)

    def imu_callback(self, msg):
        raw_x = np.nan_to_num(msg.linear_acceleration.x, nan=0.0, posinf=0.0, neginf=0.0)
        raw_y = np.nan_to_num(msg.linear_acceleration.y, nan=0.0, posinf=0.0, neginf=0.0)
        raw_ang_z = np.nan_to_num(msg.angular_velocity.z, nan=0.0, posinf=0.0, neginf=0.0)

        if abs(raw_x) > 50.0 or abs(raw_y) > 50.0:
            raw_x = np.clip(raw_x, -20.0, 20.0)
            raw_y = np.clip(raw_y, -20.0, 20.0)

        if abs(raw_x) > 20.0 or abs(raw_y) > 20.0:
            self.collision_flag = 1.0
            self.collision_sensor = f'imu (acc_x={raw_x:.1f}, acc_y={raw_y:.1f})'

        lin_acc_x = (np.clip(raw_x / 20.0, -1.0, 1.0) + 1.0) / 2.0
        lin_acc_y = (np.clip(raw_y / 20.0, -1.0, 1.0) + 1.0) / 2.0
        ang_vel_z = (np.clip(raw_ang_z / 1.0, -1.0, 1.0) + 1.0) / 2.0
        
        self.latest_imu = [ang_vel_z, lin_acc_x, lin_acc_y]

    def lidar_callback(self,msg):
        self.latest_lidar_raw = msg.ranges
        all_lidar_data = msg.ranges
        num_of_lidar = len(all_lidar_data)
        preprocessed_lidar_data = [0.0] * num_of_lidar

        for i in range(num_of_lidar): 
            if math.isinf(all_lidar_data[i]) or math.isnan(all_lidar_data[i]):
                preprocessed_lidar_data[i] = self.MAX_RANGE
            else:
                preprocessed_lidar_data[i] = all_lidar_data[i]

        compressed_num_of_lidars = int(num_of_lidar / 3)
        compressed_lidars_data = [0.0] * compressed_num_of_lidars

        for i in range(compressed_num_of_lidars):
            normalized_val = min(preprocessed_lidar_data[i*3:(i+1)*3]) / self.MAX_RANGE
            compressed_lidars_data[i] = normalized_val 
        
        self.latest_lidar = np.nan_to_num(compressed_lidars_data, nan=1.0)

    def collision_chassis_callback(self, msg):
        if msg.contacts:
            self.collision_flag = 1.0
            self.collision_sensor = 'chassis'

    def collision_camera_callback(self, msg):
        if msg.contacts:
            self.collision_flag = 1.0
            self.collision_sensor = 'camera'

    def collision_tire_lf_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_lf'
            break

    def collision_tire_rf_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_rf'
            break

    def collision_tire_lr_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_lr'
            break

    def collision_tire_rr_callback(self, msg):
        for contact in msg.contacts:
            c1 = contact.collision1.name
            c2 = contact.collision2.name
            if 'ground_plane' in c1 or 'ground_plane' in c2:
                continue
            self.collision_flag = 1.0
            self.collision_sensor = 'tire_rr'
            break

    def model_car_odometry_callback(self, msg):
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y

        if np.isnan(odom_x) or np.isnan(odom_y):
            self.get_logger().error(f"⚠️ Raw Odometry is NaN! x={odom_x}, y={odom_y}")
        
        norm_x = (np.clip(odom_x / 15.0, -1.0, 1.0) + 1.0) / 2.0
        norm_y = (np.clip(odom_y / 15.0, -1.0, 1.0) + 1.0) / 2.0

        ori_x = msg.pose.pose.orientation.x
        ori_y = msg.pose.pose.orientation.y
        ori_z = msg.pose.pose.orientation.z
        ori_w = msg.pose.pose.orientation.w

        siny_cosp = 2 * (ori_w * ori_z + ori_x * ori_y)
        cosy_cosp = 1 - 2 * (ori_y * ori_y + ori_z * ori_z)
        odom_yaw = math.atan2(siny_cosp, cosy_cosp)
        normalized_odom_yaw = (odom_yaw / math.pi + 1.0) / 2.0 

        odom_linear_vel = np.nan_to_num(msg.twist.twist.linear.x, nan=0.0)
        odom_angular_vel = np.nan_to_num(msg.twist.twist.angular.z, nan=0.0)

        norm_linear_vel = (np.clip(odom_linear_vel / 2.0, -1.0, 1.0) + 1.0) / 2.0
        norm_angular_vel = (np.clip(odom_angular_vel / 1.0, -1.0, 1.0) + 1.0) / 2.0
        self.latest_odometry_info = [norm_x, norm_y, normalized_odom_yaw, norm_linear_vel, norm_angular_vel]

    def world_car_pose_callback(self, msg):
        for transform in msg.transforms:
            if transform.child_frame_id == 'car': 
                world_car_pose_x = transform.transform.translation.x
                world_car_pose_y = transform.transform.translation.y
                world_car_pose_z = transform.transform.translation.z

                if np.isnan([world_car_pose_x, world_car_pose_y, world_car_pose_z]).any():
                    self.get_logger().error(f"⚠️ World Pose is NaN!")
                    self.world_car_pose = None
                else:
                    self.world_car_pose = [world_car_pose_x, world_car_pose_y, world_car_pose_z]
                break

    def send_request(self):
        while not self.cli_env_reset.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('reset 서비스를 기다리는 중')
        self.get_logger().info('리셋 요청 전송 (좌표는 서버에서 자동 생성)')
        return self.cli_env_reset.call_async(self.req)

    def get_observation(self):
        missing_sensors = []
        if self.latest_image is None: missing_sensors.append('image')
        if self.latest_imu is None: missing_sensors.append('imu')
        if self.latest_lidar is None: missing_sensors.append('lidar')
        if self.latest_odometry_info is None: missing_sensors.append('odometry')
        
        if missing_sensors:
            self.get_logger().error(f'None 센서 : {", ".join(missing_sensors)} (Step {self.steps_done})')
        
        lidar_data = np.array(self.latest_lidar, dtype=np.float32) if self.latest_lidar is not None else np.zeros((120,), dtype=np.float32)
        imu_data = np.array(self.latest_imu, dtype=np.float32) if self.latest_imu is not None else np.zeros((3,), dtype=np.float32)
        odom_data = np.array(self.latest_odometry_info, dtype=np.float32) if self.latest_odometry_info is not None else np.zeros((5,), dtype=np.float32)

        sensor_data = np.concatenate([lidar_data, imu_data, odom_data])

        if sensor_data.max() > 1.0 or sensor_data.min() < 0.0:
            sensor_data = np.clip(sensor_data, 0.0, 1.0)

        nan_count = np.isnan(sensor_data).sum()
        inf_count = np.isinf(sensor_data).sum()
        if nan_count > 0 or inf_count > 0:
            sensor_data = np.nan_to_num(sensor_data, nan=0.0, posinf=0.0, neginf=0.0)
        
        if self.latest_image is not None:
            img_nan = np.isnan(self.latest_image).sum()
            if img_nan > 0:
                self.latest_image = np.nan_to_num(self.latest_image, nan=0.0)
        
        return {
            'image': self.latest_image,
            'sensors': sensor_data
        }
    
    def compute_dist(self):
        if self.world_car_pose is None:
            return 23.0
        now_car_pose = self.world_car_pose
        pose_x = (now_car_pose[0] - self.goal_position[0])
        pose_y = (now_car_pose[1] - self.goal_position[1])
        pose_z = (now_car_pose[2] - self.goal_position[2])
        return math.sqrt(pose_x*pose_x + pose_y*pose_y + pose_z*pose_z)
    
    def compute_reward(self):
        if self.collision_flag:
            return -2.0
        
        if self.is_stuck:
            return -1.0

        if self.world_car_pose is None or np.isnan(self.world_car_pose).any():
            return -2.0

        dist = self.compute_dist()
        
        if np.isnan(dist) or np.isinf(dist):
            return -2.0

        if dist < 0.5:
            self.is_arrived_at_goal = True
            return 2.0

        if self.prev_dist is not None:
            dist_diff = self.prev_dist - dist
            
            if abs(dist_diff) > 2.0:
                dist_diff = np.clip(dist_diff, -0.5, 0.5)
            
            dist_reward = np.clip(dist_diff * 2.0, -0.5, 0.5)
        else:
            dist_reward = 0.0

        self.prev_dist = dist

        time_penalty = -0.01
        dist_shaping = -0.005 * dist

        step_reward = dist_reward + time_penalty + dist_shaping
        return np.clip(step_reward, -1.0, 1.0)

    def check_done(self):
        # 잔여 콜백 flush
        self._spin_all(total_sec=0.02, each_timeout=0.005)
        
        if self.collision_flag:
            self.get_logger().error(f'[종료] 충돌 센서={self.collision_sensor}, step={self.steps_done}')
            return 1.0
        
        if self.latest_lidar_raw is not None:
            valid_ranges = [r for r in self.latest_lidar_raw if not math.isinf(r) and not math.isnan(r) and r > 0]
            if valid_ranges:
                min_dist = min(valid_ranges)
                if min_dist < 1.35:
                    self.collision_flag = 1.0  
                    self.collision_sensor = f'lidar (min={min_dist:.2f}m)'
                    self.get_logger().error(f'[종료] 라이다 근접 충돌 min_dist={min_dist:.2f}m, step={self.steps_done}')
                    return 1.0

        if self.is_arrived_at_goal:
            self.get_logger().info(f'[종료] 목표 도달 step={self.steps_done}')
            return 1.0
        
        if self.stuck_count >= self.STUCK_THRESHOLD:
            self.is_stuck = True
            self.get_logger().warn(f'[종료] Stuck 감지 ({self.STUCK_THRESHOLD}스텝 연속 미이동), step={self.steps_done}')
            return 1.0
        
        if self.steps_done == self.MAX_STEP:
            self.get_logger().warn(f'[종료] 최대 스텝({self.MAX_STEP}) 도달')
            return 1.0
        
        return 0.0
    
    def step(self, action): 
        steering_angle = float(action[0])
        linear_velocity = float(action[1])

        move_msg = Twist()
        move_msg.linear.x = linear_velocity
        move_msg.angular.z = steering_angle
        self.pub_cmd_vel.publish(move_msg)

        # 0.1초 동안 대기 중인 모든 콜백 처리
        self._spin_all(total_sec=0.1, each_timeout=0.01)
        self.steps_done += 1

        next_state = self.get_observation()
        
        # 이동량 기반 Stuck 감지
        current_pose = self.world_car_pose
        
        if self.prev_pose is not None and current_pose is not None:
            dx = current_pose[0] - self.prev_pose[0]
            dy = current_pose[1] - self.prev_pose[1]
            actual_move = math.hypot(dx, dy)
            
            if actual_move > 1.5:
                self.get_logger().error(f"‼️ 글리치 감지 (이동거리: {actual_move:.2f}m)")
                return next_state, 0.0, True 

            if actual_move < 0.01:
                self.stuck_count += 1
            else:
                self.stuck_count = 0
        
        self.prev_pose = current_pose

        done = self.check_done()
        reward = self.compute_reward()

        return next_state, reward, done
        
    def reset(self):
        self.pub_cmd_vel.publish(Twist())
        
        # 기존 데이터 초기화
        self.world_car_pose = None
        self.latest_odometry_info = None
        self.latest_image = None
        self.latest_lidar = None
        
        # 낡은 메시지 비우기
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.01)

        # 리셋 요청
        reset_future = self.send_request()
        rclpy.spin_until_future_complete(self, reset_future, timeout_sec=5.0)
        self.get_logger().info('✅ 리셋 서비스 응답 완료, 동기화 시작...')

        if reset_future.result() is None or not reset_future.result().success:
            self.get_logger().error('Reset service failed')
            return self.reset()

        import re
        msg = reset_future.result().message
        match = re.search(r"Reset done to \((-?\d+\.\d+), (-?\d+\.\d+)\)", msg)
        if match:
            target_x = float(match.group(1))
            target_y = float(match.group(2))
        else:
            target_x = -10.0
            target_y = 0.0

        start_time = self.get_clock().now()
        sync_success = False

        while True:
            rclpy.spin_once(self, timeout_sec=0.05) 

            cur_x, cur_y = None, None
            if self.world_car_pose is not None:
                cur_x, cur_y = self.world_car_pose[0], self.world_car_pose[1]
            elif self.latest_odometry_info is not None:
                cur_x, cur_y = self.latest_odometry_info[0], self.latest_odometry_info[1]
            
            if cur_x is not None:
                dist_diff = math.hypot(cur_x - target_x, cur_y - target_y)
                self.get_logger().info(f"Sync check: Current({cur_x:.2f}, {cur_y:.2f}) vs Target({target_x:.2f}, {target_y:.2f}) -> Diff: {dist_diff:.2f}")

                if dist_diff < 0.5:
                    self.get_logger().info(f'🎯 동기화 성공! 차이: {dist_diff:.2f}m')
                    sync_success = True
                    break

            elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9
            if elapsed > 10.0:
                self.get_logger().error('🚨 동기화 타임아웃!')
                break
        
        if not sync_success:
            return self.reset()

        # 센서 대기
        sensor_wait_start = self.get_clock().now()
        while True:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            missing = []
            if self.latest_odometry_info is None: missing.append('odom')
            if self.latest_lidar is None: missing.append('lidar')
            if self.latest_imu is None: missing.append('imu')
            
            if not missing:
                self.get_logger().info("✅ All sensors ready.")
                break
            
            elapsed_sensor = (self.get_clock().now() - sensor_wait_start).nanoseconds / 1e9
            if elapsed_sensor > 5.0:
                self.get_logger().warn(f"⚠️ Sensor wait timeout! Missing: {missing}")
                break
        
        # 변수 초기화
        self.steps_done = 0
        self.collision_flag = 0.0
        self.is_arrived_at_goal = 0.0
        self.is_stuck = False
        self.stuck_count = 0
        self.prev_pose = self.world_car_pose
        self.prev_dist = self.compute_dist()
        
        return self.get_observation()

    def publish_zero_action(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.pub_cmd_vel.publish(stop_msg)
