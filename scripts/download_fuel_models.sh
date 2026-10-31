#!/usr/bin/env bash
# ==============================================================================
# download_fuel_models.sh
#
# 预先把各个世界依赖的 Gazebo Fuel 模型缓存到本地，之后无论在线还是离线，
# 仿真都能直接加载。
#
# 为什么不用 gz fuel download 做主力：这台机器到 fuel.gazebosim.org 的
# 单连接吞吐只有 ~14 KB/s，Depot 82 MB 要一个多小时，而且 CLI 在几分钟后
# 就自己 abort 了。 fuel_prefetch.py 用 12 路 Range 请求把同样的文件拉下来
# （实测 14 KB/s -> ~170 KB/s），并按 gz 的缓存布局落盘。
#
# 用法：
#   ./scripts/download_fuel_models.sh              # 全部
#   ./scripts/download_fuel_models.sh warehouse    # 只下某个世界需要的
# ==============================================================================
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CACHE="${GZ_FUEL_CACHE:-${HOME}/.gz/fuel}"

# 各世界依赖的模型（名字必须和 world .sdf 里的 URI 一致，空格要保留）。
declare -A WORLD_MODELS=(
  [hospital]="OpenRobotics/CGMClassic,OpenRobotics/MetalCabinet,OpenRobotics/WhiteCabinet,OpenRobotics/Desk,OpenRobotics/OfficeChairGrey,OpenRobotics/SquareShelf,OpenRobotics/Table,OpenRobotics/Chair,OpenRobotics/foldable_chair"
  [house]="OpenRobotics/CGMClassic,OpenRobotics/MetalCabinet,OpenRobotics/WhiteCabinet,OpenRobotics/Desk,OpenRobotics/OfficeChairGrey,OpenRobotics/SquareShelf,OpenRobotics/Table,OpenRobotics/Chair,OpenRobotics/foldable_chair"
  # warehouse：Warehouse 本体 + 8 个 MovAi 货架 + 货架托盘 + 人物 + 椅子
  [warehouse]="OpenRobotics/Warehouse,MovAi/shelf,MovAi/shelf_big,MovAi/pallet_box_mobile,plateau/Casual female,OpenRobotics/Jersey Barrier,OpenRobotics/Chair,OpenRobotics/foldable_chair,OpenRobotics/Table,OpenRobotics/MaleVisitorOnPhone"
  # depot：整个 Depot 是 Fuel 上一个 82 MB 的大模型
  [depot]="OpenRobotics/Depot"
)

WORLDS=("${@:-}")
if [[ -z "${1:-}" ]]; then
  WORLDS=(hospital house warehouse depot)
fi

declare -A SEEN=()
URLS=()
for world in "${WORLDS[@]}"; do
  models="${WORLD_MODELS[${world}]:-}"
  if [[ -z "${models}" ]]; then
    echo "[download_fuel_models] 未知世界: ${world}（可选: hospital house warehouse depot）" >&2
    exit 2
  fi
  # 逗号分隔，模型名里可以有空格（"Casual female"）。
  IFS=',' read -ra list <<< "${models}"
  for model in "${list[@]}"; do
    if [[ -n "${SEEN[${model}]:-}" ]]; then continue; fi
    SEEN["${model}"]=1
    URLS+=("https://fuel.gazebosim.org/1.0/${model%%/*}/models/${model#*/}")
  done
done

echo "======================================================================"
echo "[download_fuel_models] 缓存目录: ${CACHE}"
echo "[download_fuel_models] 模型数: ${#URLS[@]}（世界: ${WORLDS[*]}）"
echo "======================================================================"

# 并行 Range 下载；已经缓存过的模型会直接跳过。
python3 "${SCRIPT_DIR}/fuel_prefetch.py" --cache "${CACHE}" "${URLS[@]}"

echo "======================================================================"
echo "[download_fuel_models] 完成。缓存布局: ${CACHE}/fuel.gazebosim.org/<owner>/models/<name>/<version>/"
echo "  WORLD=warehouse ./scripts/tb3_multi_explore_slam.sh"
echo "  WORLD=depot     ./scripts/tb3_multi_explore_slam.sh"
echo "======================================================================"
