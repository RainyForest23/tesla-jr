#!/bin/bash
xhost +local:docker

docker run --name rainy_isaac \
  --gpus '"device=0"' \
  -it --rm \
  --network=host \
  --shm-size=8g \
  --privileged \
  -e "ACCEPT_EULA=Y" \
  -e "PRIVACY_CONSENT=Y" \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e NVIDIA_VISIBLE_DEVICES=0 \
  -e DISPLAY=:1 \
  -e VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
  -e "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  -e "LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble/lib" \
  -e "AMENT_PREFIX_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/humble" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /usr/share/vulkan/icd.d:/usr/share/vulkan/icd.d:ro \
  -v ~/Desktop/git_teslaJR:/workspace \
  -v /opt/isaac_cache/ov:/root/.local/share/ov/data \
  -v /opt/isaac_cache/glcache:/root/.cache/nvidia/GLCache \
  -v /opt/isaac_cache/compute:/root/.nv/ComputeCache \
  nvcr.io/nvidia/isaac-sim:4.2.0 \
  /isaac-sim/isaac-sim.sh --allow-root
