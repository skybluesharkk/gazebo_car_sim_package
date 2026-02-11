import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 패키지 소스 경로
    pkg_src_path = '/home/david/ros2_car_ws/src/gazebo_car_sim_package'
    
    # 0. Launch Argument 선언 (seed)
    seed_arg = DeclareLaunchArgument(
        'seed',
        default_value='0',
        description='Random seed for training'
    )
    seed = LaunchConfiguration('seed')
    
    # 1. 환경 변수 설정
    # 1. 환경 변수 설정
    conda_base = '/home/david/miniconda3/envs/ros_humble'
    
    env_vars = [
        SetEnvironmentVariable(name='IGN_PARTITION', value='david_sim'),
        SetEnvironmentVariable(name='IGN_IP', value='127.0.0.1'),
        SetEnvironmentVariable(name='PKG_PATH', value=pkg_src_path),
        # [수정] 콘다 가상 환경 내부의 실제 미디어/리소스 경로 추가
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', 
            value=f"{pkg_src_path}/models:"
                  f"{pkg_src_path}/models/obstacles:"
                  f"{pkg_src_path}/worlds/my_car_world:"
                  f"{conda_base}/share/ignition/ignition-gazebo6:" # 가제보 모델/월드 리소스
                  f"{conda_base}/share/ignition/ignition-rendering6/media:" # 👈 핵심: 텍스처/스크립트 경로
                  f"{os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')}"),
        
        SetEnvironmentVariable(name='IGN_GAZEBO_RENDER_ENGINE_GUI', value='ogre2'),
        SetEnvironmentVariable(name='__GLX_VENDOR_LIBRARY_NAME', value='nvidia'),
        SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='4.5'),
        SetEnvironmentVariable(name='PYTHONUNBUFFERED', value='1'),
    ]

    # 2. Gazebo 시뮬레이션 실행 (Headless)
    world_file = os.path.join(pkg_src_path, 'worlds', 'my_car_world', 'my_car_world.sdf')
    
    gz_sim = ExecuteProcess(
        cmd=[
            'xvfb-run',
            '--server-num=99',
            '--server-args=-screen 0 1024x768x24 +extension GLX',
            'ign', 'gazebo', '-r', '-s', '-v', '4',
            world_file
        ],
        output='screen'
    )

    # 3. ROS GZ Bridge 설정 (충돌 토픽 각각 별도로 브릿지)
    # Gazebo에서 생성되는 실제 토픽 경로
    chassis_contact_gz = '/world/my_car_world/model/car/model/chassis/link/chassis_link/sensor/contact_sensor_chassis/contact'
    camera_contact_gz = '/world/my_car_world/model/car/model/chassis/link/chassis_link/sensor/contact_sensor_camera/contact'
    
    # 타이어 4개의 contact 토픽 경로
    tire_lf_contact_gz = '/world/my_car_world/model/car/model/left_front_wheel/link/tire_link/sensor/contact_sensor_tire/contact'
    tire_rf_contact_gz = '/world/my_car_world/model/car/model/right_front_wheel/link/tire_link/sensor/contact_sensor_tire/contact'
    tire_lr_contact_gz = '/world/my_car_world/model/car/model/left_rear_wheel/link/tire_link/sensor/contact_sensor_tire/contact'
    tire_rr_contact_gz = '/world/my_car_world/model/car/model/right_rear_wheel/link/tire_link/sensor/contact_sensor_tire/contact'

    bridge_args = [
        '/world/my_car_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/model/car/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
        '/camera@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/model/car/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/world/my_car_world/pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
        '/model/car/pose@geometry_msgs/msg/Pose]ignition.msgs.Pose',
        
        # 충돌 토픽 브리지 설정 (차체, 카메라, 타이어 4개)
        f'{chassis_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'{camera_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'{tire_lf_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'{tire_rf_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'{tire_lr_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        f'{tire_rr_contact_gz}@ros_gz_interfaces/msg/Contacts[ignition.msgs.Contacts',
        
        '--ros-args',
        '-r', '/world/my_car_world/clock:=/clock',
        '-r', '/model/car/tf:=/tf',
        
        # 각 충돌 토픽을 별도의 ROS 토픽으로 리매핑
        '-r', f'{chassis_contact_gz}:=/collision_chassis',
        '-r', f'{camera_contact_gz}:=/collision_camera',
        '-r', f'{tire_lf_contact_gz}:=/collision_tire_lf',
        '-r', f'{tire_rf_contact_gz}:=/collision_tire_rf',
        '-r', f'{tire_lr_contact_gz}:=/collision_tire_lr',
        '-r', f'{tire_rr_contact_gz}:=/collision_tire_rr'
    ]

    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen'
    )

    tf_lidar = Node(
    package='tf2_ros', executable='static_transform_publisher',
    arguments=['0', '0', '0.4', '0', '0', '0', 
               'car/chassis/chassis_link', 
               'car/chassis/chassis_link/gpu_lidar'],  
    parameters=[{'use_sim_time': True}],
    output='screen'
    )

    tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['1.0', '0', '0.3', '-1.5708', '0', '-1.5708', 
                'car/chassis/chassis_link', 
                'car/chassis/chassis_link/camera'], 
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5. Odom to TF Publisher (Foxglove 시각화용)
    odom_tf_publisher = ExecuteProcess(
        cmd=['python3', os.path.join(pkg_src_path, 'ddpg_algorithm', 'odom_to_tf_publisher.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        cwd=os.path.join(pkg_src_path, 'ddpg_algorithm')
    )

    # 6. DDPG 노드 실행
    script_path = os.path.join(pkg_src_path, 'ddpg_algorithm')

    service_node = ExecuteProcess(
        cmd=['python3', os.path.join(script_path, 'service_node.py'), '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        cwd=script_path
    )

    # train.py에 --seed 인자 전달
    training_node = ExecuteProcess(
        cmd=['python3', os.path.join(script_path, 'train.py'), '--seed', seed, '--ros-args', '-p', 'use_sim_time:=true'],
        output='screen',
        cwd=script_path
    )

    return LaunchDescription(env_vars + [
        seed_arg,  # Argument 등록
        gz_sim,
        bridge_node,
        tf_lidar,
        tf_camera,
        TimerAction(period=3.0, actions=[odom_tf_publisher]),
        TimerAction(period=5.0, actions=[service_node]),
        TimerAction(period=7.0, actions=[training_node]),
    ])