# 🏎️ Autonomous_car with GAZEBO, ROS2, and DDPG

## 개요

- 공식문서를 참고한 기본적인 차량 구현
- 경로 안내와 충돌 회피 기능을 위한 world구현
- ROS2를 활용한 통신 구현
- Pytorch를 활용한 DDPG 알고리즘 구현
- Gazebo Harmonic + ROS2 Jazzy

## 센서들

- LiDAR (range scan or ray-cast distances)
- RGB camera image (front-facing camera)
- IMU (angular velocity and linear acceleration)
- Odometry (estimated robot pose and velocity)

## dependencies

````bash
conda activate hanyang_robot
conda env export > environment.yml```
````

- ros를 통해 가제보를 실행시키기 위해서는 ros_gz_sim을 설치해야 한다.

````bash
conda install -c robostack-staging ros-humble-ros-gz-sim```
````

- 버전 문제로 인해 ros로 서버 실행하려면 아래 명령어로 해야함

```bash
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="-s my_car_world.sdf"
```
