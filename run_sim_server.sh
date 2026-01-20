#!/bin/bash

# 1. 환경 변수 설정
export PATH="$CONDA_PREFIX/bin:/usr/bin:/bin"
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"

# 2. 리소스 및 플러그인 경로 (Fortress 기준)
export IGN_GAZEBO_RESOURCE_PATH="/home/david/Autonomous_car_gz_ros_ddpg/models:$IGN_GAZEBO_RESOURCE_PATH"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-gazebo-6/plugins"
export IGN_RENDERING_PLUGIN_PATH="$CONDA_PREFIX/lib/ign-rendering-6/engine-plugins"

# 3. [핵심] NVIDIA GPU 가속 설정 (Headless)
# 소프트웨어 렌더링(LIBGL_ALWAYS_SOFTWARE=1)은 절대 쓰지 마세요.
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export MESA_GL_VERSION_OVERRIDE=4.5
export QT_QPA_PLATFORM=offscreen

echo "------------------------------------------"
echo "NVIDIA RTX 4060을 사용하여 가제보 서버를 시작합니다."
echo "------------------------------------------"

# 4. xvfb-run을 사용하여 가상 디스플레이 생성 후 실행
# -s 옵션은 서버 모드, -r은 즉시 실행입니다.
xvfb-run --auto-servernum --server-args="-screen 0 1024x768x24 +extension GLX" \
ign gazebo -s -r "/home/david/Autonomous_car_gz_ros_ddpg/worlds/my_car_world/my_car_world.sdf" -v 4