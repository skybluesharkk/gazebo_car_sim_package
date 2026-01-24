from geometry_msgs.msg import PointStamped # 타임스탬프가 포함된 3D 좌표 메시지 형식을 가져옴
from geometry_msgs.msg import Twist # 선속도와 각속도를 제어하기 위한 메시지 형식을 가져옴

import rclpy # ROS2의 파이썬 클라이언트 라이브러리를 가져옴
from rclpy.node import Node # Node작성을 위한 기본 클래스를 가져옴

from custom_interfaces import reset_interface # TODO: custom interface에 내가 구현해야함
# ros2 topic info <topic_name> 을 통해 확인한 타입을 활용
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
from ros_gz_inferfaces.msg import Contacts

#TODO 각 topic에 대해 몇개씩 큐에 담아둘지는 일단 10개로 통일해서 적고 업데이트 빈도에 맞춰서 수정해야 함.
class EnvNode(Node):

    def __init__(self):
        # 노드 이름 정의하기
        super().__init__('EnvNode')

        self.cli_env_reset = self.create_client( reset_interface, 'reset') # reset 해주는 서비스 들어가야함 reset_interface라는 양식으로 reset에게 주문해야함.
        # 아래는 파이썬 환경이 구독해와야 할 토픽 목록임. 각각에 대해 구독자를 생성하면 될 듯
        # LiDAR
        self.sub_lidar = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10) # 어떤 형식으로,어떤이름으로 받아서, 뭐를 실행할거고, 몇개까지 큐에 저장할건지
        # camera
        self.sub_camera = self.create_subscription(Image,'/camera',self.camera_callback,10)
        # camera_info
        self.sub_camera_info = self.create_subscription(CameraInfo,'/camera_info',self.camera_info_callback,10)
        # imu
        self.sub_imu = self.create_subscription(Imu,'/imu',self.imu_callback,10)
        # collision
        self.sub_collison = self.create_subscription(Contacts,'/collision',self.collision_callback,10)
        # model/car/odometry
        self.sub_model_car_odometry = self.create_subscription(Odometry,'/model/car/odometry',self.model_car_odometry_callback,10)
        # model/car/tf
        self.sub_model_car_tf = self.create_subscription(TFMessage,'/model/car/tf',self.model_car_tf_callback,10)
        # model/car/pose
        self.sub_model_car_pose = self.create_subscription(PoseStamped,'/model/car/pose',self.model_car_pose_callback,10)

        # 발행자 만들기
        # /cmd/vel
        self.pub_cmd_vel = self.create_publisher(Twist,'/cmd/vel',10)

        self.req = reset_interface.Request() # service server에 보낼 리퀘스트를 미리 만들어둠

    # TODO 각 토픽을 수신했을 떄 어떻게 할 지 콜백함수를 작성함. 일단은 pass로 모두 채우고 바꾸기
    def lidar_callback(self,msg):
        pass
    def camera_callback(self,msg):
        pass
    def camera_info_callback(self,msg):
        pass
    def imu_callback(self,msg):
        pass
    def collision_callback(self,msg):
        pass # TODO 지면과 맏닿은 것은 충돌이 아님. 따라서 충돌의 대상이 ground_plane이 아닐때만 특정 행동을 수행해야함.
    def model_car_odometry_callback(self,msg): # TODO tf인지 잘 모르겠지만 좌표를 받으면 그 좌표랑 골인 지점 사이 거리 계산해서 작으면 리셋 리퀘스트 보내야함
        pass
    def model_car_tf_callback(self,msg):
        pass
    def model_car_pose_callback(self,msg):
        pass
    def send_request(self,msg): # 요청 보내는 함수. 환경 리셋해달라고.
        # TODO 환경의 특정 좌표를 던져서 자동차를 출발점으로 이동시키거나, 다르게 해야 할 거 같은 데 아직 모르겠다. 일단 처음 출발좌표 적기
        self.req.x = -10
        self.req.y = 0
        self.req.z = 0

        return self.cli_env_reset.call_async(self.req)

def main():
    rclpy.init()
    node = EnvNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown() 
