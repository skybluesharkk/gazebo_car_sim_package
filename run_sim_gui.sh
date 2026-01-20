#!/bin/bash

# 1. PATH 설정
export PATH="$CONDA_PREFIX/bin:/usr/bin:/bin:/usr/sbin:/sbin"

# 2. 리소스 경로 설정 
# models 폴더와 car 부품들이 있는 폴더를 모두 포함시킵니다.
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/models/obstacles:$PWD/worlds/my_car_world:$GZ_SIM_RESOURCE_PATH"

# 3. 맥북 M2 그래픽 엔진 안정화 설정 
export GZ_RENDERING_PLUGIN_PATH="$CONDA_PREFIX/lib/gz-rendering-8/engine-plugins"
export OGRE2_CONFIG_PATH="$CONDA_PREFIX/share/gz-rendering8/ogre2"
export OGRE_PLUGIN_DIR="$CONDA_PREFIX/lib/OGRE-Next"

# 4. 라이브러리 경로
export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib:$DYLD_LIBRARY_PATH"

echo "------------------------------------------"
echo "Gazebo Harmonic GUI를 시작합니다."
echo "대상 월드: $PWD/worlds/my_car_world/my_car_world.sdf"
echo "------------------------------------------"

# 5. GUI 모드(-g)로 서버에 접속
# 서버가 이미 실행 중이므로, 같은 월드 파일을 지정하면 해당 세션에 붙습니다 
gz sim -g "$PWD/worlds/my_car_world/my_car_world.sdf"