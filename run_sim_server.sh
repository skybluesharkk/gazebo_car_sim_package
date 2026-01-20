#!/bin/bash

# 1. PATH 설정 (hanyang_jazzy 환경 우선)
export PATH="$CONDA_PREFIX/bin:/usr/bin:/bin:/usr/sbin:/sbin"

# 2. 리소스 경로 설정
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/models/obstacles:$PWD/worlds/my_car_world:$GZ_SIM_RESOURCE_PATH"

# 3. [핵심] 렌더링 엔진 경로 지정
# Harmonic(Sim 8)은 Ogre2를 기본으로 사용합니다 ㅋ
export GZ_RENDERING_PLUGIN_PATH="$CONDA_PREFIX/lib/gz-rendering-8/engine-plugins"
export OGRE2_CONFIG_PATH="$CONDA_PREFIX/share/gz-rendering8/ogre2"

# 4. 라이브러리 경로
export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib:$DYLD_LIBRARY_PATH"

echo "------------------------------------------"
echo "ROS 2 Jazzy + Gazebo Harmonic 서버를 시작합니다."
echo "gz 위치: $(which gz)"
echo "------------------------------------------"

# 5. 서버 실행 (물리 엔진 즉시 시작 -r) ㅋ
# 만약 또 터진다면 뒤에 --render-engine ogre 를 붙여보세요!
gz sim -s -r "$PWD/worlds/my_car_world/my_car_world.sdf" -v 4