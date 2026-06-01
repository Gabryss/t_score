# t_score

`t_score` is a ROS 2 Jazzy package for generating a 2D traversability costmap from a 3D point-cloud map.

The package was built around RTAB-Map cloud-map outputs from mine-like environments. It focuses on estimating the traversability of the lower ground layer while avoiding common false positives from roof, wall, and vertical-column points projected into the same XY cell.

## Outputs

The main node publishes:

- `/traversability_costmap`: global `nav_msgs/msg/OccupancyGrid`
- `/traversability_costmap_local`: local `nav_msgs/msg/OccupancyGrid`

Costmap values follow standard occupancy semantics:

- `-1`: unknown
- `0`: low traversability risk
- `1..99`: increasing traversability risk
- `100`: lethal/untraversable

The helper image exporter can also save:

- `output/traversability_costmap_final_cloud_risk.png`
- `output/traversability_costmap_final_cloud_risk.png.json`
- `output/traversability_costmap_final_cloud_risk.npz`

In the risk PNG:

- blue: safe / low risk
- red: dangerous / high risk
- gray: unknown
- black: lethal / untraversable

The `.npz` file is the reusable artifact. It stores the exact `OccupancyGrid` cell values plus metadata, so it can be republished later without the original point cloud.

## Build

From the workspace:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select t_score --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

## Run The Node

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch t_score t_score_launch.py
```

Launch arguments are available for common multi-robot overrides:

```bash
ros2 launch t_score t_score_launch.py \
  namespace:=rover_1 \
  robot_frame:=rover_1/base_link \
  map_frame:=map \
  profile:=mine_graph
```

The launch file loads:

```text
ros2_ws/src/t_score/config/params.json
```

By default, the node subscribes to:

```text
/rtabmap/cloud_map
```

and uses `map` as the traversability frame and `base_link` as the robot frame.

## Namespaces

For multi-robot runs, set `ros_namespace` in `params.json`:

```json
"ros_namespace": "rover_1"
```

With this set, configured topics are prefixed:

```text
/rtabmap/cloud_map              -> /rover_1/rtabmap/cloud_map
/traversability_costmap         -> /rover_1/traversability_costmap
/traversability_costmap_local   -> /rover_1/traversability_costmap_local
/tf                             -> /rover_1/tf
/tf_static                      -> /rover_1/tf_static
```

Leave `ros_namespace` empty for the current single-robot behavior:

```json
"ros_namespace": ""
```

The helper scripts also accept `--namespace`:

```bash
ros2 run t_score publish_costmap_from_file.py \
  output/traversability_costmap_final_cloud_risk.npz \
  --namespace rover_1
```

TF has two separate collision concerns:

- TF topics: `/tf` and `/tf_static` can be namespaced or remapped per robot.
- TF frame IDs: frame names inside the TF messages must also be unique when robots share the same TF tree.

For a shared map with multiple rovers, a common pattern is:

```json
"traversability_frame_id": "map",
"robot_frame_id": "rover_1/base_link"
```

and each robot publishes frames such as:

```text
map -> rover_1/odom -> rover_1/base_link
map -> rover_2/odom -> rover_2/base_link
```

The launch file remaps `/tf` and `/tf_static` using `ros_namespace`, `tf_topic`, and `tf_static_topic`. Namespacing the topic alone does not rewrite frame IDs inside TF messages.

The same settings can be overridden from launch:

```bash
ros2 launch t_score t_score_launch.py \
  namespace:=rover_2 \
  robot_frame:=rover_2/base_link
```

## Export A Costmap Image

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run t_score costmap_to_image.py \
  --param-path /home/gabriel/docker/t_score/ros2_ws/src/t_score/config/params.json
```

The output paths, palette, topic, timeout, and save policy are configured in `params.json`.

This also writes a raw reusable costmap snapshot:

```text
output/traversability_costmap_final_cloud_risk.npz
```

Keep this file if the original detailed cloud map will no longer be available.

## Republish A Saved Costmap

After a `.npz` snapshot has been saved, the costmap can be republished later as a normal ROS `OccupancyGrid` topic:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run t_score publish_costmap_from_file.py \
  /home/gabriel/docker/t_score/output/traversability_costmap_final_cloud_risk.npz \
  --topic /traversability_costmap \
  --rate 1.0
```

For a one-shot latched publication:

```bash
ros2 run t_score publish_costmap_from_file.py \
  /home/gabriel/docker/t_score/output/traversability_costmap_final_cloud_risk.npz \
  --topic /traversability_costmap \
  --once
```

The publisher uses reliable transient-local QoS so late subscribers can still receive the saved map.

For a namespaced robot:

```bash
ros2 run t_score publish_costmap_from_file.py \
  /home/gabriel/docker/t_score/output/traversability_costmap_final_cloud_risk.npz \
  --namespace rover_1 \
  --rate 1.0
```

## Run On The Final Cloud From The Bag

For the current RTAB-Map bag:

```text
/home/gabriel/bag_files/rtabmap_outputs/rtabmap_output_gt_detailed_cloud_final
```

the reliable workflow is:

1. Start `t_score`.
2. Start `costmap_to_image.py`.
3. Replay the bag near the last `/rtabmap/cloud_map` message.

Example:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch t_score t_score_launch.py
```

In a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 run t_score costmap_to_image.py \
  --param-path /home/gabriel/docker/t_score/ros2_ws/src/t_score/config/params.json
```

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash

ros2 bag play /home/gabriel/bag_files/rtabmap_outputs/rtabmap_output_gt_detailed_cloud_final \
  --clock \
  --start-offset 2500 \
  --rate 10 \
  --topics /tf /tf_static /rtabmap/cloud_map
```

The bag itself ends after the last `/rtabmap/cloud_map`; using an offset near the cloud-map end avoids replaying a long tail that contains no cloud-map updates.

## Algorithm Summary

For each XY grid cell, the node stores a bounded sample of 3D points and computes terrain metrics:

- local floor layer
- slope from PCA
- roughness from plane residuals
- local height span
- confidence from point count
- obstacle evidence inside the robot clearance band

The cost is a weighted risk score from slope, roughness, height, and confidence. Unknown cells remain `-1`.

### Roof And Wall Handling

Mine clouds often contain roof points projected into the same XY cell as the floor. To reduce roof artifacts, the algorithm uses a vertical-column classifier:

- `Floor`: scored normally
- `FloorWithObstacle`: lethal
- `WallOrVerticalSurface`: configurable, currently not treated as lethal
- `CeilingOnly`: unknown
- `Unknown`: unknown

The classifier is intentionally local and conservative. A neighborhood pass suppresses floating high surfaces when they sit far above nearby valid floor cells.

## Important Parameters

The package still loads `params.json`, but the main node now declares the JSON keys as ROS parameters. This means launch files can override the most important values without editing the JSON. Profile values are applied first, then ROS launch/parameter overrides win.

Profile selection:

```json
"traversability_profile": "mine_graph"
```

Available presets in `params.json`:

```text
mine_graph
navigation_conservative
small_rover
```

Map sizing:

```json
"map_resolution": 0.5,
"global_map_size": 40,
"global_map_growth_margin": 5.0,
"global_map_growth_step": 20.0,
"global_map_max_size": 300.0
```

Scoring thresholds:

```json
"max_traversable_slope": 0.70,
"max_traversable_roughness": 0.20,
"max_traversable_height": 0.25
```

Ground and obstacle filtering:

```json
"enable_ground_layer_filter": true,
"ground_quantile": 0.08,
"ground_band_below": 0.05,
"ground_band_above": 0.25,
"ceiling_ignore_height": 0.60,
"obstacle_min_height": 0.35,
"obstacle_min_points": 12
```

Column classifier:

```json
"enable_column_classifier": true,
"min_floor_points": 8,
"wall_cells_as_obstacles": false,
"require_floor_for_obstacle": true,
"floating_floor_neighbor_radius": 2,
"max_floor_height_jump": 0.80,
"enable_global_floor_support": true,
"global_floor_seed_quantile": 0.10,
"global_floor_seed_height": 0.60,
"global_floor_max_step": 0.45
```

`enable_global_floor_support` grows a supported floor region from low floor seeds through neighboring cells. Floor-like cells that are disconnected from this supported surface are suppressed as unknown; this helps reject roof planes that look locally flat.

Runtime/performance:

```json
"max_points_per_cell": 80,
"cloud_point_stride": 1,
"publish_global_on_update_only": true,
"publish_debug_maps": false,
"enable_footprint_inflation": false
```

For Raspberry Pi deployment, increase `cloud_point_stride`, lower `max_points_per_cell`, and keep debug maps disabled if CPU load is too high.

## Helper Scripts

### `costmap_to_image.py`

Saves a `nav_msgs/msg/OccupancyGrid` as an image, metadata JSON, and raw `.npz` snapshot.

```bash
ros2 run t_score costmap_to_image.py --help
```

It reads defaults from `params.json`, but command-line flags can override them.

### `publish_costmap_from_file.py`

Republishes a saved raw `.npz` costmap snapshot as `nav_msgs/msg/OccupancyGrid`.

```bash
ros2 run t_score publish_costmap_from_file.py output/traversability_costmap_final_cloud_risk.npz
```

### `publish_final_cloud_from_bag.py`

Reads the final cloud from a bag and republishes it with identity TF. This can be useful for one-shot tests, but replaying the original bag near the final cloud is usually more reliable for the large RTAB-Map cloud used here.

```bash
ros2 run t_score publish_final_cloud_from_bag.py BAG_DIR \
  --cloud-topic /rtabmap/cloud_map \
  --namespace rover_1 \
  --seek-back-sec 4000 \
  --publish-seconds 8 \
  --rate 5
```

## Notes

- The global grid starts small and expands as cloud points approach the boundary.
- Unknown space is intentionally preserved as unknown, not free.
- `step_window_radius_cells` is currently `0` because the neighborhood step-height pass was too aggressive on the mine cloud and marked many traversable passages as lethal.
- Enable `publish_debug_maps` only when tuning; it publishes slope, roughness, height, and confidence grids.

## Tests

The package includes a focused C++ classifier test:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select t_score --event-handlers console_direct+
```

The current functional test covers:

- floor-only cells
- floor plus roof points
- floor plus obstacle points
- wall-only cells

The broad style linters are not run by default because this package vendors RapidJSON headers and includes helper scripts that do not follow the default ROS Python quote style.
