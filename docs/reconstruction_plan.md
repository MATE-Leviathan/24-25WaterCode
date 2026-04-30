# Comprehensive Plan: Underwater 3D Reconstruction with Imaging Sonar and Cameras in HoloOcean

**Created**: 2026-01-07
**Author**: Claude Code
**Project**: Leviathan AUV - 3D Reconstruction in Simulation

---

## Executive Summary

Based on my research and analysis of your repos, you have an excellent foundation for this project. Your `sonar_cam_message_converter.py` in 24-25WaterCode already bridges HoloOcean to standard sonar reconstruction packages, which is a huge head start. Here's your comprehensive plan:

---

## I. State of the Art Research (2024-2025)

### Recent Breakthrough Publications

**1. VISO - Visual-Inertial-Sonar SLAM (January 2025)** ⭐ Most Relevant
- Fuses stereo camera + IMU + 3D sonar for 6-DoF localization
- Achieves dense 3D reconstruction with photometric rendering
- Published literally days ago at arXiv
- **Relevance**: This is the gold standard approach for your use case

**2. AONeuS - Acoustic-Optical Neural Rendering (August 2024)**
- Physics-based multimodal neural surface reconstruction
- Integrates high-res RGB with low-res depth-resolved imaging sonar
- Uses neural implicit representations (NeRF-like)
- **Relevance**: Great for high-quality reconstruction, but computationally intensive

**3. Multi-Sonar Fusion for 3D Point Clouds (2025)**
- Generates 3D point clouds from multiple sonar viewpoints
- Optimal scan path planning for AUVs
- **Relevance**: Useful for planning your data collection trajectories

**4. Side-Scan Sonar 3D Reconstruction (2024)**
- Shape-from-shading + monocular depth fusion
- Novel depth estimation network
- **Relevance**: If you want to add side-scan sonar later

### Key Technical Insights from Research

1. **Complementary sensors**: Imaging sonar provides range/geometry in low visibility, cameras provide texture/color in clear water
2. **Forward-looking imaging sonar** provides range + azimuth (but not elevation)
3. **Multipath artifacts** are a major challenge - HoloOcean simulates this!
4. **Sensor fusion approaches**: Probabilistic (Extended Kalman Filter), Neural (NeRF-based), or Classical (ICP + feature matching)

---

## II. Your Current Setup Analysis

### Assets You Already Have ✓

**1. HoloOcean Simulation** (`/home/chris/HoloOcean/`)
- ✓ ImagingSonar with realistic physics (multipath, noise models)
- ✓ RGBCamera sensors with configurable resolution
- ✓ IMU, DVL, Depth, Location sensors for ground truth
- ✓ Multiple worlds: Dam, PierHarbor (both excellent for 3D reconstruction testing)
- ✓ HoveringAUV and TorpedoAUV agent types
- ✓ Octree-based sonar simulation with configurable resolution

**2. HoloOcean-ROS Integration** (`/home/chris/holoocean-ros/`)
- ✓ `holoocean_node.py` publishes sensor data to ROS 2 topics
- ✓ Custom message types: `ImagingSonar.msg`, `RGBCamera.msg`
- ✓ Publishes `/clock` for simulation time synchronization
- ✓ Publishes `nav_msgs/Odometry` for ground truth trajectory
- ✓ Example scenarios with sonar + camera configurations

**3. Your Robot Code** (`/home/chris/24-25WaterCode/`)
- ✓ **Critical asset**: `sonar_cam_message_converter.py` already exists!
  - Converts HoloOcean `ImagingSonar` → `sonar_oculus/OculusPing` format
  - Handles bearing angle calculation, range binning
  - Includes odometry passthrough to SLAM
  - JPEG compression for efficiency
- ✓ Camera publishers (`fish_cam` package)
- ✓ IMU publisher (BNO055)
- ✓ Depth sensor integration (Bar02)
- ✓ Docker setup with `slam-toolbox` already installed

### What You're Missing

1. ❌ External reconstruction package (`sonar_camera_reconstruction` or equivalent)
2. ❌ HoloOcean scenario JSON configured for data collection
3. ❌ ROS 2 bag recording workflow
4. ❌ Evaluation metrics and ground truth comparison tools

---

## III. Recommended Implementation Plan

### Phase 1: Set Up HoloOcean Simulation for Data Collection (Week 1)

**Goal**: Create a HoloOcean scenario with AUV, imaging sonar, and cameras in a rich environment

#### Step 1.1: Create Custom Scenario JSON
Create `/home/chris/holoocean-ros/holoocean_main/config/reconstruction_test.json`:

```json
{
    "name": "ReconstructionTest",
    "package_name": "Ocean",
    "world": "Dam",
    "main_agent": "auv0",
    "ticks_per_sec": 30,
    "frames_per_sec": 30,
    "agents": [{
        "agent_name": "auv0",
        "agent_type": "HoveringAUV",
        "location": [0, -700, -50],
        "rotation": [0, 0, 0],
        "control_scheme": 0,
        "sensors": [
            {
                "sensor_type": "DynamicsSensor",
                "ros_publish": true
            },
            {
                "sensor_type": "IMUSensor",
                "ros_publish": true
            },
            {
                "sensor_type": "DVLSensor",
                "ros_publish": true
            },
            {
                "sensor_type": "DepthSensor",
                "ros_publish": true
            },
            {
                "sensor_type": "ImagingSonar",
                "sensor_name": "FrontSonar",
                "socket": "SonarSocket",
                "Hz": 5,
                "configuration": {
                    "Azimuth": 120,
                    "Elevation": 20,
                    "RangeMin": 0.5,
                    "RangeMax": 30,
                    "RangeBins": 512,
                    "AzimuthBins": 512,
                    "AddSigma": 0.02,
                    "MultSigma": 0.001,
                    "Reflections": true
                }
            },
            {
                "sensor_type": "RGBCamera",
                "sensor_name": "FrontCamera",
                "socket": "CameraSocket",
                "Hz": 20,
                "configuration": {
                    "CaptureWidth": 1280,
                    "CaptureHeight": 720,
                    "FOV": 90
                }
            }
        ]
    }]
}
```

**Key decisions**:
- **World choice**: Dam for structured environment (pipes, walls), PierHarbor for natural features
- **Sonar parameters**: 512x512 gives good resolution, 5 Hz is realistic for imaging sonar
- **Camera parameters**: 1280x720 at 20 Hz matches your existing hardware
- **All navigation sensors**: IMU, DVL, Depth for proper localization

#### Step 1.2: Create ROS 2 Parameters File
Create `/home/chris/holoocean-ros/holoocean_main/config/reconstruction_params.yaml`:

```yaml
holoocean_node:
  ros__parameters:
    scenario_path: "config/reconstruction_test.json"
    relative_path: true
    show_viewport: true
    publish_commands: true
    draw_arrow: true
    render_quality: 2
```

#### Step 1.3: Test Basic Simulation
```bash
cd ~/holoocean-ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

ros2 launch holoocean_main holoocean_launch.py params_file:=config/reconstruction_params.yaml
```

**Verify topics**:
```bash
ros2 topic list | grep holoocean
# Should see:
# /holoocean/auv0/DynamicsSensorOdom
# /holoocean/auv0/IMUSensor
# /holoocean/auv0/DVLSensorVelocity
# /holoocean/auv0/FrontSonar
# /holoocean/auv0/FrontCamera
# /clock
```

---

### Phase 2: Set Up Data Collection with ROS 2 Bags (Week 1)

**Goal**: Record synchronized sensor data for offline algorithm development

#### Step 2.1: Create Bag Recording Script
Create `/home/chris/24-25WaterCode/scripts/record_reconstruction_data.sh`:

```bash
#!/bin/bash

# Set use_sim_time for all nodes
ros2 param set /use_sim_time true

# Create bags directory
mkdir -p ~/reconstruction_bags

# Record all relevant topics
ros2 bag record \
  /holoocean/auv0/DynamicsSensorOdom \
  /holoocean/auv0/DynamicsSensorIMU \
  /holoocean/auv0/IMUSensor \
  /holoocean/auv0/DVLSensorVelocity \
  /holoocean/auv0/DepthSensor \
  /holoocean/auv0/FrontSonar \
  /holoocean/auv0/FrontCamera \
  /clock \
  -o ~/reconstruction_bags/holoocean_trajectory_$(date +%Y%m%d_%H%M%S)
```

#### Step 2.2: Create Trajectory Control Node
You need a node to fly the AUV in a scanning pattern. Create `/home/chris/24-25WaterCode/fishROS_ws/src/trajectory_control/trajectory_control/scanning_trajectory.py`:

```python
import rclpy
from rclpy.node import Node
from holoocean_interfaces.msg import AgentCommand, DesiredCommand
import numpy as np

class ScanningTrajectory(Node):
    def __init__(self):
        super().__init__('scanning_trajectory')
        self.cmd_pub = self.create_publisher(AgentCommand, '/holoocean/command/agent', 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        # Lawnmower pattern parameters
        self.start_time = self.get_clock().now()
        self.depth = -5.0  # meters
        self.speed = 0.5   # m/s
        self.leg_length = 20.0  # meters
        self.spacing = 3.0  # meters between passes

    def control_loop(self):
        t = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Lawnmower scanning pattern
        leg_time = self.leg_length / self.speed
        turn_time = 5.0
        cycle_time = leg_time + turn_time

        cycle_num = int(t // cycle_time)
        cycle_t = t % cycle_time

        msg = AgentCommand()

        if cycle_t < leg_time:
            # Straight leg
            direction = 1 if cycle_num % 2 == 0 else -1
            msg.x_force = direction * self.speed
            msg.y_force = 0.0
        else:
            # Turn
            msg.x_force = 0.0
            msg.y_force = self.speed if cycle_num % 2 == 0 else -self.speed

        msg.z_force = self.depth  # Depth control

        self.cmd_pub.publish(msg)
```

**Alternative approach**: Use joystick control (`joy_holoocean.py`) for manual exploration

#### Step 2.3: Collect Multiple Datasets

**Dataset 1: Lawnmower scan** (structured coverage)
```bash
# Terminal 1: Launch HoloOcean
ros2 launch holoocean_main holoocean_launch.py params_file:=config/reconstruction_params.yaml

# Terminal 2: Run trajectory controller
ros2 run trajectory_control scanning_trajectory

# Terminal 3: Record bag
./record_reconstruction_data.sh
```

**Dataset 2: Manual exploration** (natural movement)
```bash
# Terminal 2: Use joystick control instead
ros2 launch holoocean_examples joy_launch.py

# Terminal 3: Record bag
./record_reconstruction_data.sh
```

**Recommended datasets**:
- 5-10 minute trajectories
- 3-5 different starting locations
- Vary distance to objects (2-15m)
- Include loop closures (return to starting point)

---

### Phase 3: Bridge to Reconstruction Algorithms (Week 2)

**Goal**: Convert HoloOcean data to format compatible with SLAM/reconstruction packages

#### Step 3.1: Adapt Your Existing Message Converter

You already have `sonar_cam_message_converter.py`! Adapt it to work with HoloOcean simulation data:

**Location**: `/home/chris/24-25WaterCode/fishROS_ws/src/controller_node/controller_node/sonar_cam_message_converter.py`

**Modifications needed**:
1. Subscribe to `/holoocean/auv0/FrontSonar` (HoloOcean ImagingSonar)
2. Subscribe to `/holoocean/auv0/FrontCamera` (HoloOcean RGBCamera)
3. Subscribe to `/holoocean/auv0/DynamicsSensorOdom` (Odometry)
4. Convert to target format (likely `sonar_oculus/OculusPing` format)
5. Publish converted messages

**The converter should**:
- Reshape sonar data from flat array to 2D (512×512)
- Calculate bearing angles from azimuth bins
- Add proper timestamps
- Synchronize camera + sonar + odometry messages
- Optionally compress images (JPEG)

**Note**: Your existing code already does most of this! Just update the topic names and test with simulation data.

#### Step 3.2: Choose Reconstruction Algorithm

You have several options:

**Option A: Traditional SLAM + Octomap** (Recommended for Phase 1)
- Use `slam_toolbox` (already in your Docker) for 2D pose estimation
- Use `octomap_server` for 3D occupancy mapping
- Fuse sonar range data as 3D points
- Overlay camera texture on 3D map

**Pros**: Well-tested, real-time capable, good baseline
**Cons**: Limited to occupancy grid, no smooth surfaces

**Option B: Sonar-specific SLAM**
Research packages:
- `sonar_oculus` + `sonar_camera_reconstruction` (what your converter targets)
- `bathymetric_slam` (for profiling sonars, but adaptable)
- Custom ICP-based registration

**Pros**: Designed for sonar artifacts
**Cons**: May need significant adaptation

**Option C: Neural Reconstruction (AONeuS approach)**
- Implement NeRF-based acoustic-optical fusion
- Train neural implicit representation
- Beautiful reconstructions

**Pros**: State-of-the-art quality
**Cons**: Requires significant implementation, not real-time

**Recommendation**: Start with Option A (SLAM + Octomap), then move to Option B

#### Step 3.3: Integration Architecture

```
HoloOcean Simulation (holoocean_node)
    ↓
ROS 2 Topics:
    /holoocean/auv0/FrontSonar (ImagingSonar)
    /holoocean/auv0/FrontCamera (Image)
    /holoocean/auv0/DynamicsSensorOdom (Odometry)
    ↓
sonar_cam_message_converter.py
    ↓
Converted Topics:
    /sonar_oculus/ping (or custom format)
    /camera/image_raw
    /odometry/filtered
    ↓
SLAM Algorithm (slam_toolbox / custom)
    ↓
3D Reconstruction Node
    ↓
Outputs:
    /map (OccupancyGrid)
    /octomap (Octomap)
    /reconstruction/mesh (Mesh)
```

---

### Phase 4: Implement 3D Reconstruction Pipeline (Week 2-3)

#### Step 4.1: Create Sonar Point Cloud Generator

Create `/home/chris/24-25WaterCode/fishROS_ws/src/reconstruction/reconstruction/sonar_to_pointcloud.py`:

```python
import rclpy
from rclpy.node import Node
from holoocean_interfaces.msg import ImagingSonar
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class SonarToPointCloud(Node):
    def __init__(self):
        super().__init__('sonar_to_pointcloud')

        self.sonar_sub = self.create_subscription(
            ImagingSonar, '/holoocean/auv0/FrontSonar',
            self.sonar_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/holoocean/auv0/DynamicsSensorOdom',
            self.odom_callback, 10)

        self.pc_pub = self.create_publisher(PointCloud2, '/sonar/pointcloud', 10)

        self.current_pose = None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def sonar_callback(self, msg):
        if self.current_pose is None:
            return

        # Parse sonar configuration (from message or parameters)
        azimuth_fov = np.deg2rad(120)  # 120° FOV
        elevation_fov = np.deg2rad(20)  # 20° FOV
        range_min = 0.5
        range_max = 30.0

        azimuth_bins = 512
        range_bins = 512

        # Reshape flat image to 2D
        intensities = np.array(msg.image_data).reshape((range_bins, azimuth_bins))

        # Generate bearing angles
        azimuth_angles = np.linspace(-azimuth_fov/2, azimuth_fov/2, azimuth_bins)
        ranges = np.linspace(range_min, range_max, range_bins)

        # Threshold intensities to find returns
        threshold = 0.3  # Adjust based on data
        detections = intensities > threshold

        points = []
        for r_idx in range(range_bins):
            for a_idx in range(azimuth_bins):
                if detections[r_idx, a_idx]:
                    r = ranges[r_idx]
                    theta = azimuth_angles[a_idx]

                    # Convert to 3D point (sonar frame: x=forward, y=left, z=up)
                    x = r * np.cos(theta)
                    y = r * np.sin(theta)
                    z = 0  # Imaging sonar doesn't provide elevation

                    # Transform to world frame using odometry
                    # (requires proper transformation - simplified here)
                    points.append([x, y, z, intensities[r_idx, a_idx]])

        # Publish point cloud
        header = msg.header
        header.frame_id = 'auv0/sonar'
        pc_msg = pc2.create_cloud_xyz32(header, points)
        self.pc_pub.publish(pc_msg)
```

**Key improvements needed**:
- Proper coordinate frame transformation (TF2)
- Intensity-to-range peak detection (not just thresholding)
- Handle multipath returns
- Temporal filtering

#### Step 4.2: Integrate Camera Data

Create a node to fuse camera + sonar:

```python
# Subscribes to both /sonar/pointcloud and /camera/image_raw
# Uses odometry to project camera texture onto sonar 3D points
# Publishes colored point cloud
```

#### Step 4.3: Create Occupancy Map

Use `octomap_server`:

```bash
ros2 run octomap_server octomap_server_node \
  --ros-args \
  -r cloud_in:=/sonar/pointcloud \
  -p resolution:=0.1 \
  -p frame_id:=odom
```

This generates a 3D occupancy grid from point clouds.

#### Step 4.4: Mesh Generation (Optional)

For visualization:
- Use `pcl_ros` tools to generate mesh from point cloud
- Use Marching Cubes algorithm on octomap
- Export to PLY/STL for external visualization

---

### Phase 5: Evaluation and Ground Truth Comparison (Week 3)

**Goal**: Validate reconstruction quality against ground truth

#### Step 5.1: Extract Ground Truth from HoloOcean

HoloOcean publishes perfect odometry via `DynamicsSensorOdom`. You can also:

1. **Export Unreal Engine mesh** of the environment
   - Use Unreal Editor to export meshes of Dam/PierHarbor world
   - Convert to common format (OBJ, PLY)

2. **Create ground truth trajectory**
   - Record `DynamicsSensorOdom` topic
   - Extract poses for comparison

3. **Generate synthetic sonar/camera from ground truth mesh**
   - Use HoloOcean's octree visualization to export

#### Step 5.2: Evaluation Metrics

**Trajectory Metrics** (Localization quality):
- Absolute Trajectory Error (ATE)
- Relative Pose Error (RPE)
- Compare estimated odometry vs. ground truth `DynamicsSensorOdom`

**Reconstruction Metrics** (Map quality):
- Point cloud to mesh distance (Hausdorff distance)
- Completeness (% of ground truth covered)
- Accuracy (mean distance from estimated to ground truth)
- Use CloudCompare or Open3D for comparison

**Implementation**:
```python
import numpy as np
from scipy.spatial.distance import directed_hausdorff

def evaluate_reconstruction(estimated_pc, ground_truth_mesh):
    # Sample points from ground truth mesh
    gt_points = mesh.sample_points_uniformly(n=10000)

    # Compute metrics
    hausdorff_dist = directed_hausdorff(estimated_pc, gt_points)[0]
    mean_dist = np.mean(np.min(cdist(estimated_pc, gt_points), axis=1))

    return hausdorff_dist, mean_dist
```

#### Step 5.3: Visualization

Use RViz2 to visualize:
```bash
rviz2
```

Add displays:
- `/holoocean/auv0/DynamicsSensorOdom` (Odometry)
- `/sonar/pointcloud` (PointCloud2)
- `/map` (OccupancyGrid)
- `/octomap` (MarkerArray)
- `/camera/image_raw` (Image)
- TF frames

---

### Phase 6: Iterate and Improve (Week 4+)

#### Step 6.1: Parameter Tuning
- Sonar detection threshold
- Point cloud filtering (outlier removal)
- SLAM parameters (loop closure, optimization)
- Sensor fusion weights

#### Step 6.2: Advanced Features
- **Loop closure detection** using camera features (ORB, SIFT)
- **Bundle adjustment** for trajectory optimization
- **Sonar shadow handling** (regions behind objects)
- **Multipath rejection** using temporal consistency
- **Adaptive resolution** based on distance

#### Step 6.3: Neural Reconstruction (Optional)
If you want state-of-the-art results:
- Implement AONeuS-style neural implicit reconstruction
- Use PyTorch + NeRF framework
- Train on recorded bag files
- Requires GPU (can use your Docker CUDA setup)

---

## IV. Alternative Approaches

### Approach A: Use Existing Sonar SLAM Package

**Option**: Marine SLAM packages like `marine_acoustic_slam`

**Steps**:
1. Find compatible ROS 2 sonar SLAM package
2. Adapt your message converter to match expected format
3. Run SLAM on recorded bags
4. Extract 3D map

**Pros**: Less implementation work
**Cons**: May not exist for imaging sonar, designed for profiling/sidescan

### Approach B: Implement VISO Algorithm

Replicate the VISO paper (January 2025):

**Steps**:
1. Implement stereo camera visual odometry (or use mono + IMU)
2. Implement sonar odometry (ICP on sonar images)
3. Fuse using Extended Kalman Filter or factor graph (GTSAM)
4. Dense reconstruction via photometric rendering

**Pros**: State-of-the-art performance
**Cons**: Significant implementation effort (PhD-level work)

### Approach C: Traditional Visual SLAM + Sonar Enhancement

**Steps**:
1. Use ORB-SLAM3 or RTAB-Map for camera-based SLAM
2. Add sonar range measurements as depth constraints
3. Enhance sparse visual map with dense sonar depth

**Pros**: Leverages mature visual SLAM
**Cons**: May not fully utilize sonar data

---

## V. Recommended Technology Stack

### Core Components
- **ROS 2 Humble** (you already have this)
- **HoloOcean 0.5+** with Ocean package
- **OpenCV** for camera processing
- **PCL (Point Cloud Library)** for 3D processing
- **Octomap** for occupancy mapping

### Optional but Recommended
- **GTSAM** for factor graph optimization (better than EKF)
- **Open3D** for point cloud processing and visualization
- **CloudCompare** for ground truth comparison
- **PlotJuggler** for time-series data analysis

### Visualization
- **RViz2** for real-time ROS visualization
- **Meshlab** for mesh inspection
- **Foxglove** (already in your Docker) for bag analysis

---

## VI. Practical Next Steps (This Week)

1. **Create reconstruction test scenario JSON** (30 min)
   - File: `/home/chris/holoocean-ros/holoocean_main/config/reconstruction_test.json`
   - Use template from Phase 1.1

2. **Test HoloOcean launch with new scenario** (15 min)
   ```bash
   ros2 launch holoocean_main holoocean_launch.py params_file:=config/reconstruction_params.yaml
   ```

3. **Record test bag file** (30 min)
   - Use manual joystick control for 5-minute trajectory
   - Record all sensor topics

4. **Inspect bag contents** (15 min)
   ```bash
   ros2 bag info ~/reconstruction_bags/holoocean_trajectory_XXXXXX
   ros2 topic echo /holoocean/auv0/FrontSonar --once
   ```

5. **Visualize in RViz2** (30 min)
   - Set up displays for all topics
   - Verify data looks reasonable

6. **Test message converter** (1 hour)
   - Run your existing `sonar_cam_message_converter.py` on recorded bag
   - Check output format

7. **Create simple point cloud converter** (2 hours)
   - Implement basic sonar → point cloud (simplified version of Phase 4.1)
   - Visualize in RViz2

---

## VII. Timeline Estimate

| Phase | Tasks | Duration |
|-------|-------|----------|
| 1 | Scenario setup, bag recording | 1 week |
| 2 | Message conversion, data collection | 1 week |
| 3 | Point cloud generation, SLAM integration | 1-2 weeks |
| 4 | 3D reconstruction pipeline | 1-2 weeks |
| 5 | Evaluation, ground truth comparison | 1 week |
| 6 | Iteration, parameter tuning | 2+ weeks |
| **Total** | **Basic system working** | **6-8 weeks** |
| Advanced | Neural reconstruction (optional) | +4-8 weeks |

---

## VIII. Potential Challenges and Solutions

### Challenge 1: Sonar Elevation Ambiguity
**Problem**: Forward-looking imaging sonar only provides azimuth, not elevation
**Solution**:
- Use multiple sonar passes from different heights
- Fuse with camera depth estimation (structure from motion)
- Accept 2.5D reconstruction (elevation map)

### Challenge 2: Sonar Multipath
**Problem**: Reflections create ghost returns
**Solution**:
- HoloOcean simulates this - use as test case
- Implement temporal consistency checks
- Use probabilistic filtering (particle filter)

### Challenge 3: Camera Limited Range
**Problem**: Cameras don't work in turbid water
**Solution**:
- Sonar provides geometry, camera provides texture when available
- Test in clear water regions of HoloOcean (near surface)
- Use flashlight sensor in HoloOcean for illumination

### Challenge 4: Data Association
**Problem**: Matching sonar + camera features is hard
**Solution**:
- Use odometry for initial alignment
- Implement joint optimization (bundle adjustment)
- Use appearance-based loop closure on camera only

---

## IX. Success Criteria

### Minimum Viable Product (6 weeks)
✓ Record synchronized sonar + camera + odometry bags from HoloOcean
✓ Generate point clouds from sonar data
✓ Create 3D occupancy map of simple structure (pipe, wall)
✓ Visualize textured reconstruction in RViz2

### Full Success (8 weeks)
✓ Real-time or near-real-time reconstruction pipeline
✓ Quantitative evaluation (ATE < 0.5m, reconstruction error < 0.1m)
✓ Handle complex environments (Dam or PierHarbor)
✓ Fuse sonar + camera for textured mesh
✓ Loop closure capability

### Stretch Goals (12+ weeks)
✓ Neural implicit reconstruction (NeRF-style)
✓ Multi-session mapping
✓ Real-time performance on embedded hardware
✓ Generalization to real hardware (your physical robot)

---

## X. References and Resources

### Academic Papers (Must Read)
1. **VISO: Robust Underwater Visual-Inertial-Sonar SLAM** (arXiv:2601.01144v1)
   - https://arxiv.org/html/2601.01144v1
2. **AONeuS: A Neural Rendering Framework for Acoustic-Optical Sensor Fusion** (arXiv:2402.03309)
   - https://arxiv.org/html/2402.03309
3. **HoloOcean: Realistic Sonar Simulation** (Potokar et al., IROS 2022)
   - https://robots.et.byu.edu/jmangelson/pubs/2022/Potokar22iros.pdf
4. **HoloOcean: An Underwater Robotics Simulator** (Potokar et al., ICRA 2022)
   - https://www.cs.cmu.edu/~kaess/pub/Potokar22icra.pdf

### Documentation
5. **HoloOcean Documentation** - https://byu-holoocean.github.io/holoocean-docs/v1.0.0/index.html
6. **ROS 2 Humble Docs** - https://docs.ros.org/en/humble/
7. **Octomap Documentation** - https://octomap.github.io/

### Software Libraries
8. **PCL** - Point Cloud Library
9. **GTSAM** - Georgia Tech Smoothing and Mapping
10. **Open3D** - Modern 3D data processing
11. **evo** - Python package for trajectory evaluation

### Additional Research Papers
12. **Three-dimensional reconstruction of underwater side-scan sonar images**
    - https://link.springer.com/article/10.1007/s44295-023-00013-0
13. **Overview of Underwater 3D Reconstruction Technology Based on Optical Images**
    - https://www.mdpi.com/2077-1312/11/5/949

---

## XI. Summary and Recommendations

**You're in a great position!** You have:
1. ✅ High-quality simulation (HoloOcean)
2. ✅ ROS 2 integration already working
3. ✅ Message converter infrastructure (`sonar_cam_message_converter.py`)
4. ✅ Docker environment with SLAM tools
5. ✅ Real robot hardware for eventual deployment

**Recommended immediate path**:
1. **This week**: Set up HoloOcean scenario, record first bag files
2. **Next week**: Get basic sonar → point cloud working
3. **Week 3-4**: Integrate with SLAM/Octomap for 3D reconstruction
4. **Week 5-6**: Evaluate, tune, iterate
5. **Later**: Deploy to real hardware, neural reconstruction

**Start with the simplest approach** (Option A: SLAM + Octomap), get it working end-to-end, then incrementally add sophistication. The bag files you create will be reusable as you improve algorithms.

---

## XII. Contact and Support

**HoloOcean Community**:
- GitHub: https://github.com/byu-holoocean/holoocean
- Issues: https://github.com/byu-holoocean/holoocean/issues

**ROS 2 Community**:
- ROS Discourse: https://discourse.ros.org/
- ROS Answers: https://answers.ros.org/

**Research Groups**:
- BYU FRoStLab (HoloOcean developers): https://frostlab.byu.edu/

---

**Document Version**: 1.0
**Last Updated**: 2026-01-07
