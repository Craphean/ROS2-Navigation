#!/usr/bin/env bash
# ==============================================================================
# download_fuel_models.sh
# 
# 预先从 Gazebo Fuel (fuel.gazebosim.org) 官方云端模型库批量下载并本地缓存
# 医院 (Hospital) 与家庭生活 (House) 以及大型仓储/中转站 (Warehouse/Depot) 所需的模型。
# 
# 一旦下载完成，模型将缓存在 ~/.gz/fuel/fuel.gazebosim.org/ 目录下，
# 后续无论在线或离线启动仿真，Gazebo 都能瞬间秒开加载！
# ==============================================================================
set -eo pipefail

source /opt/ros/lyrical/setup.bash

echo "======================================================================"
echo "[download_fuel_models] 开始检查并下载 Gazebo Fuel 官方开源世界模型库..."
echo "======================================================================"

MODELS=(
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/CGMClassic"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/MetalCabinet"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/WhiteCabinet"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Desk"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/OfficeChairGrey"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/SquareShelf"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Chair"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/foldable_chair"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Warehouse"
  "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Depot"
)

TOTAL="${#MODELS[@]}"
CURRENT=0

for url in "${MODELS[@]}"; do
  CURRENT=$(( CURRENT + 1 ))
  model_name="${url##*/}"
  echo "[${CURRENT}/${TOTAL}] 正在同步模型: ${model_name} ..."
  if gz fuel download -u "${url}" >/dev/null 2>&1; then
    echo "  -> [OK] 缓存成功: ${model_name}"
  else
    echo "  -> [WARN] 下载失败或网络波动: ${model_name}，将在后台重试一次..."
    gz fuel download -u "${url}" || echo "  -> [ERROR] 重试仍然失败: ${model_name}，请检查网络连接。"
  fi
done

echo "======================================================================"
echo "[download_fuel_models] 所有 Fuel 模型检测与同步流程完成！"
echo "本地缓存路径: ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models/"
echo "现可使用以下命令直接启动全新地图世界:"
echo "  - 医院病房走廊环境: WORLD=hospital ./scripts/tb3_explore_slam.sh"
echo "  - 医院双车协同探索: WORLD=hospital ./scripts/tb3_multi_explore_slam.sh"
echo "  - 家庭室内生活环境: WORLD=house ./scripts/tb3_explore_slam.sh"
echo "  - 家庭双车协同探索: WORLD=house ./scripts/tb3_multi_explore_slam.sh"
echo "======================================================================"

