#!/bin/bash

# 1. 환경 변수 설정
export PATH="$CONDA_PREFIX/bin:/usr/bin:/bin"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# 2. 리소스 및 플러그인 경로 최적화 (수정된 부분)
export PKG_PATH="/home/david/ros2_car_ws/src/gazebo_car_sim_package"

# 모델이 들어있는 모든 부모 폴더를 경로에 넣어야 합니다.
# 특히 obstacles 폴더를 추가하여 box와 wall을 찾을 수 있게 합니다.
export MODELS_PATH="$PKG_PATH/models"
export OBSTACLES_PATH="$PKG_PATH/models/obstacles"
export WORLDS_PATH="$PKG_PATH/worlds/my_car_world"

export IGN_GAZEBO_RESOURCE_PATH="$MODELS_PATH:$OBSTACLES_PATH:$WORLDS_PATH:$IGN_GAZEBO_RESOURCE_PATH"
# 최신 버전 호환성을 위해 GZ_SIM_RESOURCE_PATH도 함께 설정해주는 것이 안전합니다.
export GZ_SIM_RESOURCE_PATH="$IGN_GAZEBO_RESOURCE_PATH"

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"
export IGN_RENDERING_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-rendering-6/engine-plugins"

# 3. NVIDIA GPU 가속 설정 (기존 유지)
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MESA_GL_VERSION_OVERRIDE=4.5
# export QT_QPA_PLATFORM=offscreen

# 4. 실행할 월드 파일 경로
WORLD_FILE="$WORLDS_PATH/my_car_world.sdf"

echo "------------------------------------------"
echo "영찬님의 RTX 4060 GPU 가속으로 가제보 서버를 가동합니다."
echo "------------------------------------------"

# 5. xvfb-run 실행
# xvfb를 번호 99번으로 띄웁니다.
xvfb-run --server-num=99 --server-args="-screen 0 1024x768x24 +extension GLX" \
ign gazebo -s -r "/home/david/ros2_car_ws/src/gazebo_car_sim_package/worlds/my_car_world/my_car_world.sdf" -v 4