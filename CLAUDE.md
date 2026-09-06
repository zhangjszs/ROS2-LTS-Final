# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

HUAT FSAC 2026 autonomous driving system — a ROS 2 + Modern C++20 codebase for Formula Student driverless competition. Migrated from ROS 1 (catkin) to ROS 2 (colcon/ament). Handles perception (LiDAR + camera fusion), multi-event planning, vehicle control, and safety monitoring.

## Build Commands

```bash
# Full workspace build (uses colcon_defaults.yaml: mold + ccache + parallel-workers:4)
colcon build --symlink-install

# Build a single package
colcon build --packages-select <package_name>

# Build with tests enabled (tests are OFF by default)
colcon build --cmake-args -DBUILD_TESTING=ON
colcon test

# Check ccache hit rate
ccache -s

# Source the workspace after building
source install/setup.bash
```

**Important**: `colcon_defaults.yaml` at the workspace root auto-applies mold linker, ccache, and limits parallel workers to 4. Do NOT override `parallel-workers` to a high value — the 24-thread machine has only 19GB RAM and excessive concurrency causes swap thrashing deadlock.

## Launch Commands

```bash
# Full system (sensors + all planners + control + safety + RViz)
ros2 launch huat_launch fsac.launch.py

# Select competition event via road_type (1=skidpad, 2=straight, 3=track)
ros2 launch huat_launch fsac.launch.py road_type:=1

# Switch planner at runtime
ros2 service call /planner_mux/select topic_tools/srv/MuxSelect "{topic: '/planning/track/pathlimits'}"
```

## Architecture

### Data Flow

```
/velodyne_points → lidar_cluster → /sensors/cones/raw
  → cone_fusion + vehicle_state → /sensors/cones/transformed
  → cone_tracker → /sensors/cones/fused
  → planners → /planning/<event>/pathlimits
  → pure_pursuit → /control/vehicle_command
```

Safety path: `/planning/track/stop_request → safety_monitor → /system/stop → pure_pursuit`

### Package Roles

| Package | Role |
|---|---|
| `lidar_cluster` | PCL point cloud preprocessing, ground segmentation (SVD/RANSAC), Euclidean clustering, cone classification by size |
| `cone_fusion` | LiDAR + camera spatial fusion using TF, cone color classification |
| `cone_tracker` | KD-Tree nearest-neighbor + Kalman filtering + sliding-window dedup |
| `vision_ros` | ONNX (YOLOv8) inference with HSV fallback, temporal IoU tracker, image quality state machine |
| `urinay` | **Core track planner**: Delaunay triangulation (Bowyer-Watson) → midpoint filtering → heuristic tree search → midline + track limits. Color-blind algorithm. |
| `skidpad_planner` | Figure-8 event planner (ICP + APF) |
| `straight_line_planner` | Straight-line acceleration planner |
| `pure_pursuit` | Adaptive look-ahead lateral controller |
| `safety_monitor` | Planning heartbeat watchdog + dual-redundant emergency stop (AS/EBS) |
| `fsac_viz` | RViz2 visualization (cones, trajectory, vehicle state) |
| `huat_launch` | Centralized launch management with topic remapping args |
| `common_msgs` | Shared message definitions (HuatCone, HuatConeCluster, HuatPathLimits, HuatControlCommand, etc.) |
| `autodrive_msgs` | Unified autodrive status/control protocol (HuatVisionDetections) |

### Key Design Details

**Urinay planner** (`planning/urinay/`): The primary path planner. Uses Delaunay triangulation over cone positions, filters triangles by edge length/angle, then runs a height-limited heuristic tree search to find the midline. Computes track limits from triangle edges adjacent to midline points. Implements fail-safe (widens search params when path ends too close) and loop closure detection. Self-contained C++ library with `structures/` (Point, Edge, Triangle, etc.), `modules/` (Delaunay, WayComputer), and `utils/` (KDTree, Failsafe, params).

**Cone color/size types**: Defined in `common_msgs/include/common_msgs/cone_types.h` as `huat_cone::Color` (BLUE=0, YELLOW=1, YELLOW_BIG=2, RED=3, NONE=4) and `huat_cone::Size` (UNKNOWN=0, LARGE=1, SMALL=2). Vision emits unified YELLOW; LiDAR size splits it via `MergeVisionColorWithLidarSize()`. `HuatConeCluster.type` is SIZE-only — never copy into `HuatCone.type`.

**vision_ros**: Python node with threaded inference pipeline. ONNX Runtime backend with CPU/CUDA providers, HSV color fallback when model unavailable. Image quality state machine (NORMAL→DEGRADED→FALLBACK→VISION_LOST) drives model vs fallback selection. Publishes `HuatVisionDetections` to `/perception/vision/detections`.

**Topic naming convention**: `/sensors/<source-or-object>/<stage>`, `/localization/<state>`, `/planning/<event>/<artifact>`, `/control/<command>`, `/system/stop`, `/debug/<node>/<artifact>`, `/fsd/viz/<object>`.

## CMake Conventions

All packages use **modern target-based CMake** (required — see `docs/BUILD_OPTIMIZATION.md` for why):

```cmake
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Correct: target namespaces (auto-deduplicates 8-12 ROS 2 micro-libraries)
target_link_libraries(my_node
  common_msgs::common_msgs
  rclcpp::rclcpp
  Eigen3::Eigen
)

# Tests: link existing library, do NOT recompile .cpp sources
ament_add_gtest(test_algo test/test.cpp)
target_link_libraries(test_algo my_package_lib)
```

**Forbidden** (causes 350KB link lines / 5900+ params / 9-min link times): `${rclcpp_LIBRARIES}`, `${pkg}_INCLUDE_DIRS`, or recompiling business sources in tests.

## C++20 Adoption

Ongoing incremental modernization (tracked in `docs/CPP20_REFACTORING_ROADMAP.md`). Completed: `<numbers>`, Designated Initializers, `std::format`. In progress: `std::span` (zero-copy), `std::ranges` (lazy pipelines), Concepts, `<=>`, `std::jthread`. When modifying code, prefer these modern constructs over legacy equivalents.

## Language & Documentation

Code comments and documentation are in **Chinese (Simplified)**. Message field comments are bilingual. Maintain this convention when adding comments.
