<p align="center">
  <h1 align="center">GstAdaptNode</h1>
  <p align="center">
    <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-blue.svg" alt="License"></a>
    <img src="https://img.shields.io/badge/ROS_2-Humble-green.svg" alt="ROS 2 Humble">
    <img src="https://img.shields.io/badge/GStreamer-1.x-orange.svg" alt="GStreamer">
    <img src="https://img.shields.io/badge/C%2B%2B-17-blue.svg" alt="C++17">
    <img src="https://img.shields.io/badge/zero--copy-intra--process-cyan.svg" alt="Zero-Copy">
  </p>
</p>

Hardware-agnostic ROS 2 perception acceleration. Auto-detects host accelerators (Intel VA-API / NVIDIA NVMM) at runtime to dynamically construct zero-copy GStreamer pipelines directly from standard ROS parameters.

---

## Why Use This?

**`gst_adapt_node::ResizeNode` is a plug-and-play, drop-in replacement for `image_proc::ResizeNode`.**

Same parameters. Same interface. Swap one line in your launch file and get hardware-accelerated image processing with 10x less CPU:

<table>
<tr>
<th>Standard (CPU-bound)</th>
<th>GstAdaptNode (hardware-accelerated)</th>
</tr>
<tr>
<td>

```python
ComposableNode(
    package='image_proc',
    plugin='image_proc::ResizeNode',
    name='resize',
    parameters=[{
        'use_scale': False,
        'height': 480,
        'width': 640,
    }],
)
```

</td>
<td>

```python
ComposableNode(
    package='gst_adapt_node',
    plugin='gst_adapt_node::ResizeNode',
    name='resize',
    parameters=[{
        'use_scale': False,
        'height': 480,
        'width': 640,
    }],
)
```

</td>
</tr>
</table>

The node auto-detects your hardware and selects the optimal backend. No GStreamer knowledge required.

## Benchmark Results

Measured on an Intel desktop (VA-API), 4K (3840x2160) source resized to 640x480 at 30 Hz:

| Metric | `image_proc` (CPU) | `gst_adapt_node` (VA-API) | Improvement |
|--------|-------------------|--------------------------|-------------|
| **CPU Utilization** | 155% (1.55 cores) | 16% (0.16 cores) | **10x reduction** |
| **Latency** | ~20 ms | ~20 ms | Parity |
| **Frame Format** | BGR (24.9 MB/frame) | I420 (12.4 MB/frame) | 2x bandwidth savings |

The accelerated path frees 1.4 CPU cores per camera stream for SLAM, perception, and planning.

## How It Works

```
                    +------------------+
  Startup:          | HardwareDetector |
  Probe /dev/       |   NVIDIA_JETSON  |
  for accelerators  |   INTEL_VAAPI    |
                    |   CPU_FALLBACK   |
                    +--------+---------+
                             |
                    +--------v---------+
  Validate:         | validate_platform|
  Check GStreamer   |   gst_element_   |
  registry          |   factory_find() |
                    +--------+---------+
                             |
                    +--------v---------+
  Build:            | PipelineFactory  |
  Construct         |   appsrc ! ...   |
  pipeline string   |   ! rosimagesink |
                    +--------+---------+
                             |
                    +--------v---------+
  Execute:          | gst_parse_launch |
  Zero-copy appsrc  |   unique_ptr ->  |
  buffer wrapping   |   GstBuffer      |
                    +------------------+
```

### Fallback Chain

| Priority | Platform | Detection | GStreamer Elements |
|----------|-----------------|-------------------------------|--------------------------------------|
| 1 | NVIDIA Jetson | `/dev/nvhost-*`, `/dev/nvmap` | `nvvidconv` (NVMM memory) |
| 2 | Intel VA-API | `/dev/dri/renderD*` | `vaapipostproc` (VASurface memory) |
| 3 | CPU | Always available | `videoscale`, `videoconvert` |

If hardware is detected but the GStreamer plugin is missing, the node logs a warning and falls back to CPU automatically.

### Zero-Copy Data Path

The accelerated pipeline bypasses `rosimagesrc` entirely:

1. **Publisher** (`MediaStreamerNode`) emits I420 frames via `std::unique_ptr`
2. **Subscriber** (`ResizeNode`) receives via intra-process pointer move (zero DDS serialization)
3. **Buffer wrapping** via `gst_buffer_new_wrapped_full()` hands the raw pointer to GStreamer
4. **GPU upload** via `vaapipostproc` / `nvvidconv` operates directly on the wrapped memory
5. **Lifecycle** managed by `GDestroyNotify` callback that frees the `sensor_msgs::msg::Image` after GPU consumption

No copies of the 12.4 MB frame between ROS and GStreamer.

## Quick Start

```bash
# Clone into a colcon workspace
cd ~/ros2_ws/src
git clone --recursive https://github.com/sohams25/GstAdaptNode.git gst_adapt_node

# Install dependencies
sudo apt install ros-humble-image-proc gstreamer1.0-vaapi \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

# Build
cd ~/ros2_ws
colcon build --packages-up-to gst_adapt_node
source install/setup.bash

# Run the A/B stress test (provide any 4K .mp4)
ros2 launch gst_adapt_node A_B_comparison.launch.py video_path:=/path/to/4k_video.mp4
```

### Visual Dashboard

In a second terminal:

```bash
ros2 run gst_adapt_node visualize_demo.py
```

Opens a split-screen window with live latency, FPS, and CPU overlays for both pipelines.

## Parameters

### ResizeNode

Matches `image_proc::ResizeNode` parameter interface:

| Parameter | Type | Default | Description |
|---|---|---|---|
| `use_scale` | bool | `false` | Use proportional scaling instead of absolute |
| `scale_width` | double | `1.0` | Width scale factor (when `use_scale=true`) |
| `scale_height` | double | `1.0` | Height scale factor (when `use_scale=true`) |
| `width` | int | `640` | Output width in pixels (when `use_scale=false`) |
| `height` | int | `480` | Output height in pixels (when `use_scale=false`) |
| `input_topic` | string | `/camera/image_raw` | Source image topic |
| `output_topic` | string | `/camera/image_processed` | Destination image topic |
| `source_width` | int | `3840` | Source frame width (for appsrc caps) |
| `source_height` | int | `2160` | Source frame height (for appsrc caps) |

### MediaStreamerNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `video_path` | string | `/tmp/test_video.mp4` | Input video file |
| `loop` | bool | `true` | Loop on EOF |
| `format` | string | `bgr8` | Output format (`bgr8` or `yuv420p`) |
| `image_topic` | string | `/camera/image_raw` | Publish topic for images |
| `info_topic` | string | `/camera/camera_info` | Publish topic for CameraInfo |

## Components

```
gst_adapt_node
  gst_adapt_node::ResizeNode           # Drop-in image_proc replacement
  gst_adapt_node::MediaStreamerNode     # Video file publisher (zero-copy)
  gst_adapt_node::Synthetic4kPubNode   # Synthetic 4K test source
```

Load into any `rclcpp_components::ComponentContainer` with
`use_intra_process_comms: true` for zero-copy operation.

## Project Structure

```
gst_adapt_node/
+-- CMakeLists.txt
+-- package.xml
+-- LICENSE
+-- config/
|   +-- demo_params.yaml
+-- dependencies/
|   +-- ros-gst-bridge/          (git submodule)
+-- docs/
|   +-- index.html               (project website)
|   +-- assets/
|       +-- cpu_chart.svg
|       +-- latency_chart.svg
+-- include/gst_adapt_node/
|   +-- gst_adapt_node.hpp       (ResizeNode)
|   +-- hardware_detector.hpp
|   +-- media_streamer_node.hpp
|   +-- pipeline_factory.hpp
|   +-- synthetic_4k_pub_node.hpp
+-- launch/
|   +-- A_B_comparison.launch.py
|   +-- gst_adapt_demo.launch.py
+-- scripts/
|   +-- cpu_monitor.py
|   +-- generate_assets.py
|   +-- latency_tracker.py
|   +-- synthetic_4k_pub.py
|   +-- visualize_demo.py
+-- src/
    +-- gst_adapt_node.cpp
    +-- hardware_detector.cpp
    +-- main.cpp
    +-- media_streamer_node.cpp
    +-- pipeline_factory.cpp
    +-- synthetic_4k_pub_node.cpp
```

## Dependencies

- **ROS 2 Humble** (Ubuntu 22.04)
- **GStreamer 1.x** (`libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`)
- **OpenCV 4.x** (via `cv_bridge`)
- **[ros-gst-bridge](https://github.com/BrettRD/ros-gst-bridge)** (included as submodule)
- **Optional:** `gstreamer1.0-vaapi` (Intel), `ros-humble-image-proc` (A/B comparison)

## Roadmap

- [ ] `h264_encode` / `h264_decode` actions in PipelineFactory
- [ ] ROS 2 Lifecycle node integration
- [ ] Multi-stream support (N cameras per node)
- [ ] Direct VA-API / NVENC shared-memory transport (bypass DDS)
- [ ] Jetson Orin runtime validation
- [ ] Integration test suite with CI

## License

Apache-2.0. See [LICENSE](LICENSE).
