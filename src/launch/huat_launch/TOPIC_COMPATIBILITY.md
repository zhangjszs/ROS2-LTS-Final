# Topic compatibility layer

This launch package keeps the historical HUAT-FSAC topic names as defaults so existing
`rosbag play` validation continues to work, while exposing the main data-flow topics as
launch arguments for future cleanup or remapping.

## Default data flow

```text
/velodyne_points
  -> lidar_cluster
  -> /sensors/cones/raw
  -> cone_fusion + /localization/vehicle_state
  -> /sensors/cones/transformed
  -> cone_tracker
  -> /sensors/cones/fused
  -> planners
  -> /planning/<event>/pathlimits
  -> pure_pursuit
  -> /control/vehicle_command
```

Track-drive stop requests are now separated from the final system stop:

```text
/planning/track/stop_request -> safety_monitor -> /system/stop -> pure_pursuit
```

## Common launch arguments

| Argument | Default topic | Meaning |
| --- | --- | --- |
| `pointcloud_topic` | `/velodyne_points` | LiDAR point cloud input from rosbag/driver |
| `raw_ins_topic` | `/INS/ASENSING_INS` | Legacy INS source used by vehicle_state |
| `vehicle_state_topic` | `/localization/vehicle_state` | Standard vehicle state topic |
| `raw_cones_topic` | `/sensors/cones/raw` | Cone detections in sensor/base frame |
| `transformed_cones_topic` | `/sensors/cones/transformed` | Cone detections transformed to global/map frame |
| `fused_cones_topic` | `/sensors/cones/fused` | Tracked/deduplicated cone map |
| `path_topic` | event-specific `/planning/.../pathlimits` | Controller path input |
| `stop_topic` | `/system/stop` | Final safety stop consumed by control |
| `stop_request_topic` | `/planning/track/stop_request` | Planner stop request before safety arbitration |
| `vehicle_command_topic` | `/control/vehicle_command` | Control command output |

Example without changing rosbag contents:

```bash
roslaunch huat_launch track_run.launch \
  pointcloud_topic:=/velodyne_points \
  raw_ins_topic:=/INS/ASENSING_INS \
  fused_cones_topic:=/sensors/cones/fused
```

## Naming rule for future topics

- Sensor inputs/outputs: `/sensors/<source-or-object>/<stage>`
- Localization state: `/localization/<state>`
- Planning output: `/planning/<event>/<artifact>`
- Control output: `/control/<command>`
- Safety arbitration: `/system/stop` for final stop, `/planning/<event>/stop_request` for planner requests
- Debug-only streams: `/debug/<node>/<artifact>`
- RViz markers: `/fsd/viz/<object>`
