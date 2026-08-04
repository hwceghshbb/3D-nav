# ELF3 head RGB-D cuVSLAM

Camera ownership is deliberately separated:

```text
head RGB-D  -> cuVSLAM odometry/relocalization, RTAB mapping, SCAN navigation
body camera -> stair policy active perception only
```

cuVSLAM RGB-D mode supports one color camera with pixel-aligned depth. The body
camera is not subscribed by this package and does not gate `/nav/rig_ready`.
The head joints must remain locked at their calibrated zero position.

## Install cuVSLAM for ROS Python

ROS Humble uses Python 3.10. Install the local cuVSLAM build for that interpreter
once on each deployment machine:

```bash
./scripts/install_cuvslam_ros_python.sh /home/hwc/code/cuVSLAM
```

## Mapping

Start the normal RTAB mapping process and the head cuVSLAM mapping process
against the same live head RGB-D streams. The second process records the visual
landmark database required for later relocalization:

```bash
ros2 launch bxi_cuvslam_localization elf3_head_cuvslam_mapping.launch.py \
  map_directory:=/opt/bxi/maps/site_cuvslam
```

By default the mapping launch reads `/rtabmap/localization_pose` once before
the first tracked frame. That pose anchors cuVSLAM to the same `map` frame as
RTAB/Octo. Do not begin walking until the log reports `Anchored cuVSLAM origin`;
if RTAB publishes an Odometry anchor instead, set `map_anchor_pose_topic:=''`
and `map_anchor_odom_topic:=<topic>`.

Before ending mapping, save the cuVSLAM database explicitly:

```bash
ros2 service call /head_cuvslam_node/save_map std_srvs/srv/Trigger '{}'
ros2 topic echo /nav/relocalization_status --once
```

The map directory contains `data.mdb` and `bxi_metadata.json`. The metadata
records intrinsics, the fixed head-camera extrinsic, map origin and cuVSLAM
version. A useful map requires robot motion and overlapping multi-view images;
a map made while completely stationary is not sufficient for relocalization.

## Localization

```bash
ros2 launch bxi_cuvslam_localization elf3_head_cuvslam_localization.launch.py \
  map_directory:=/opt/bxi/maps/site_cuvslam
```

The node keeps `/nav/localization_valid=false` until cuVSLAM has matched the
current head image against the saved map. For a known approximate start pose,
publish `/initialpose`, then retry with:

```bash
ros2 service call /head_cuvslam_node/relocalize std_srvs/srv/Trigger '{}'
```

Important outputs:

```text
/visual_slam/tracking/odometry  raw head cuVSLAM base pose
/nav/odom                       validated odometry
/nav/base_footprint/pose        navigation pose in map
/nav/head_depth_camera/pose     head camera pose in map
/nav/relocalized                saved-map match result
/nav/localization_valid         motion safety gate
/nav/relocalization_status      readable state/error text
```

## Simulation error recording

The simulation launch compares `/visual_slam/tracking/odometry` with the
timestamp-matched MuJoCo truth on `/simulation/odom`. An event is recorded only
after five consecutive samples exceed either 0.50 m or 20 degrees. Each launch
records at most one event in:

```text
/tmp/cuvslam_large_error_events.jsonl
```

The latched `/nav/cuvslam_large_error` topic is `true` after an event is saved.
The monitor is disabled by default in the common launch and enabled only by the
simulation and mapping-joystick launches.

## Saving a mapping session

The mapping-joystick launch enables the cuVSLAM SLAM backend and creates a new
timestamped directory under the workspace `maps/` directory. A single Ctrl+C
automatically saves `data.mdb` and `bxi_metadata.json`. For an explicit save
before shutdown, call:

```bash
ros2 service call /head_cuvslam_node/save_map std_srvs/srv/Trigger "{}"
```

Wait for `map_saved` on `/nav/relocalization_status` before stopping the launch.

## Simulation validation

```bash
ros2 launch bxi_cuvslam_localization elf3_head_cuvslam_sim.launch.py \
  mode:=mapping map_directory:=/tmp/elf3_head_cuvslam_map
```

The validation launch uses only the existing MuJoCo head and body cameras. Head
RGB and aligned depth run at 320x240 and 30 Hz; the low-resolution body stream
remains available to the stair policy but is absent from navigation and
localization subscriptions.
