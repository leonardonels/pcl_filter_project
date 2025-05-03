<div align="center">
    <h1>Simple Ros2 node to rotate, traslate and dynamically filter rays vertically</h1>
</div>

## :gear: How to build & Run
```commandline
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MMR-Electric-Driverless/pcl-filter.git
```
```commandline
cd ~/ros2_ws
colcon build --symlink-install --packages-select pcl-filter
source install/setup.bash
```

## :abacus: Parameters
```yaml
pointcloud_filter_node:
  ros__parameters:

    input_topic: "/lidar/points"
    output_topic: "/lidar/filtered"

    # If the rotation and all traslation are set to 0. no trasformation are applied, reducing the total callback duration
    y_rotation_angle: -0.1570796 

    x_traslation: 0.
    y_traslation: 0.
    z_traslation: 1.

    gradient: true  # If set to false, the gradint wont be applied, reducing the total callback duration
    
    # The number of vertical zones can change, for this example there are 3
    vertical_zones:
      - "start: 0.0, end: 0.25, downsample: 4"
      - "start: 0.25, end: 0.75, downsample: 1"
      - "start: 0.75, end: 1.0, downsample: 4"

```
