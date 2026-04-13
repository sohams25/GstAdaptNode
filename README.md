# GstAdaptNode

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![GStreamer](https://img.shields.io/badge/GStreamer-1.x-orange.svg)](https://gstreamer.freedesktop.org/)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-blue.svg)](https://en.cppreference.com/w/cpp/17)

> **[View the Project Website & Interactive Benchmarks](https://sohams25.github.io/GstAdaptNode/)**

Hardware-agnostic ROS 2 perception acceleration. Auto-detects host accelerators (Intel VA-API / NVIDIA NVMM) at runtime to dynamically construct zero-copy GStreamer pipelines directly from standard ROS parameters.

---

## Why Use This?

**`gst_adapt_node::ResizeNode` is a plug-and-play, drop-in replacement for `image_proc::ResizeNode`.**

Same parameters. Same interface. Accepts standard `bgr8` from any ROS 2 camera. Swap one line in your launch file:

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

The node auto-detects your hardware and selects the optimal backend. No GStreamer knowledge required. No upstream changes to your camera driver.

## Benchmark Results

Measured on an Intel desktop (VA-API), 4K (3840x2160) BGR8 source resized to 640x480 at 30 Hz:

| Metric | `image_proc` (CPU) | `gst_adapt_node` (VA-API) | Improvement |
|--------|-------------------|--------------------------|-------------|
| **CPU Utilization** | 152% (1.52 cores) | 27% (0.27 cores) | **5.6x reduction** |
| **Latency** | 400+ ms (under load) | **5-15 ms** | Sub-frame |
| **Input Format** | bgr8 | bgr8 | Full compatibility |

The accelerated path frees 1.25 CPU cores per camera stream for SLAM, perception, and planning — while accepting the exact same `bgr8` frames as `image_proc`.

<p align="center">
  <img src="docs/assets/cpu_chart.svg" width="420" alt="CPU Utilization: Legacy 152% vs Accelerated 27%">
  &nbsp;&nbsp;
  <img src="docs/assets/latency_chart.svg" width="420" alt="Latency: Legacy 45ms vs Accelerated 15ms">
</p>

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
  pipeline string   |   ! appsink      |
                    +--------+---------+
                             |
                    +--------v---------+
  Execute:          | gst_parse_launch |
  Zero-copy appsrc  |   unique_ptr ->  |
  buffer wrapping   |   GstBuffer      |
                    +------------------+
```

### Hardware BGR Ingestion

Maintains 100% plug-and-play compatibility with standard ROS `bgr8` cameras by performing color conversion on the hardware acceleration path (via VA-API for Intel or CUDA for NVIDIA), keeping the CPU free for perception workloads. The `appsrc` ingests raw BGR frames directly from the ROS message buffer, and the hardware pipeline handles format conversion + resize in a single GPU pass.

### Fallback Chain

| Priority | Platform | Detection | GStreamer Elements |
|----------|-----------------|-------------------------------|--------------------------------------|
| 1 | NVIDIA Jetson | `/dev/nvhost-*`, `/dev/nvmap` | `nvvideoconvert` (CUDA + NVMM) |
| 2 | Intel VA-API | `/dev/dri/renderD*` | `vaapipostproc` (VASurface) |
| 3 | CPU | Always available | `videoscale`, `videoconvert` |

If hardware is detected but the GStreamer plugin is missing, the node logs a warning and falls back to CPU automatically.

### Zero-Copy Data Path

```
MediaStreamerNode (bgr8, unique_ptr)
    | intra-process pointer move (0 copies)
    v
ResizeNode::on_image()
    | gst_buffer_new_wrapped_full (0 copies)
    v
appsrc -> [videoconvert -> vaapipostproc -> vaapipostproc -> videoconvert] -> appsink
    |                    GPU resize (VA-API / NVMM)                           |
    v                                                                         v
GDestroyNotify frees Image              on_new_sample() publishes result
```

The entire input path — from ROS publisher to GPU upload — involves zero memory copies. The `appsink` callback on the output side publishes processed frames directly as ROS messages, bypassing the overhead of `rosimagesrc`/`rosimagesink` bridge elements.

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
+-- config/demo_params.yaml
+-- dependencies/ros-gst-bridge/    (git submodule)
+-- docs/
|   +-- index.html                  (project website)
|   +-- assets/
|       +-- cpu_chart.svg
|       +-- latency_chart.svg
+-- include/gst_adapt_node/         (5 headers)
+-- launch/                         (2 launch files)
+-- scripts/                        (5 Python tools)
+-- src/                            (6 C++ sources)
```

## Dependencies

- **ROS 2 Humble** (Ubuntu 22.04)
- **GStreamer 1.x** (`libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`)
- **OpenCV 4.x** (via `cv_bridge`)
- **[ros-gst-bridge](https://github.com/BrettRD/ros-gst-bridge)** (included as submodule)
- **Optional:** `gstreamer1.0-vaapi` (Intel), `ros-humble-image-proc` (A/B comparison)

## Roadmap

- [ ] `h264_encode` / `h264_decode` actions in PipelineFactory
- [ ] OpenGL bridge for GStreamer 1.22+ (`vapostproc` with zero-CPU BGR ingestion)
- [ ] ROS 2 Lifecycle node integration
- [ ] Multi-stream support (N cameras per node)
- [ ] Direct shared-memory transport (bypass DDS)
- [ ] Jetson Orin runtime validation
- [ ] Integration test suite with CI

## License

Apache-2.0. See [LICENSE](LICENSE).
