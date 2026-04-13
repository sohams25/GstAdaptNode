# gst_adapt_node

**Hardware-Agnostic GStreamer Acceleration for ROS 2**

A composable ROS 2 node that detects available GPU hardware at startup and
builds an optimized GStreamer pipeline automatically. Write one configuration.
Deploy on NVIDIA Jetson, Intel, or bare CPU. The node adapts.

## Abstract

Standard ROS 2 image processing uses CPU-bound OpenCV operations regardless of
the hardware available on the target platform. `gst_adapt_node` replaces this
with a single composable node that:

1. Probes the `/dev/` tree for NVIDIA Jetson or Intel VA-API hardware
2. Validates that the required GStreamer plugins are installed
3. Builds and launches a hardware-accelerated GStreamer pipeline at runtime
4. Falls back gracefully to CPU if no accelerator is found

The result is a drop-in replacement that offloads image scaling (and future
encode/decode) to dedicated hardware, freeing CPU cores for perception and
planning workloads.

## Architecture

### Hardware Auto-Detection

`HardwareDetector` probes the system at startup:

```
/dev/nvhost-*, /dev/nvmap    -->  NVIDIA_JETSON
/dev/dri/renderD*            -->  INTEL_VAAPI
(neither)                    -->  CPU_FALLBACK
```

### Plugin Validation

Before committing to a hardware path, `validate_platform()` checks the
GStreamer element registry for the specific elements the pipeline requires.
If `vaapipostproc` or `nvvidconv` is missing, it downgrades to CPU with a
visible warning. No silent failures.

### Element Map

`PipelineFactory` translates `PlatformInfo` + `PipelineConfig` into a
GStreamer launch string:

| Platform | Pipeline |
|---|---|
| **NVIDIA Jetson** | `rosimagesrc ! video/x-raw,format=BGR ! videoconvert ! video/x-raw,format=I420 ! nvvidconv ! video/x-raw(memory:NVMM),width=W,height=H ! nvvidconv ! videoconvert ! video/x-raw,format=BGR ! rosimagesink` |
| **Intel VA-API** | `rosimagesrc ! video/x-raw,format=BGR ! videoconvert ! video/x-raw,format=NV12 ! vaapipostproc ! video/x-raw(memory:VASurface),width=W,height=H ! vaapipostproc ! videoconvert ! video/x-raw,format=BGR ! rosimagesink` |
| **CPU Fallback** | `rosimagesrc ! videoscale ! videoconvert ! video/x-raw,width=W,height=H ! rosimagesink` |

Caps are pinned at each conversion boundary to minimize memory bandwidth.
Source-side `videoconvert` is constrained to NV12/I420 (12 bpp) rather than
letting GStreamer auto-negotiate a higher-bandwidth format.

### Hybridized Zero-Copy Architecture

The accelerated pipeline eliminates `rosimagesrc` entirely. Instead,
`GstAdaptNode` subscribes directly to the image topic with a
`std::unique_ptr<sensor_msgs::msg::Image>` callback. When co-located in
the same `ComponentContainer`, the 12.4 MB I420 frames transfer via pointer
move (zero DDS serialization). The raw buffer is then wrapped directly into
a GStreamer buffer via `gst_buffer_new_wrapped_full()` — zero copies from
ROS publisher to GPU upload.

A custom `GDestroyNotify` callback ensures the `sensor_msgs::msg::Image`
is freed only after GStreamer finishes consuming the buffer, preventing the
12.4 MB per-frame memory leak.

`MediaStreamerNode` publishes native I420 (YUV 4:2:0 planar) via its
`format` parameter, halving bandwidth compared to BGR (12.4 MB vs 24.9 MB
per 4K frame) and eliminating the CPU-bound `videoconvert` before the
hardware scaler.

```
src/gst_adapt_node/
+-- include/gst_adapt_node/
|   +-- gst_adapt_node.hpp           # GStreamer pipeline node (composable)
|   +-- hardware_detector.hpp        # /dev/ tree probing
|   +-- media_streamer_node.hpp      # Video file publisher (composable)
|   +-- pipeline_factory.hpp         # Platform -> GStreamer string mapping
|   +-- synthetic_4k_pub_node.hpp    # Synthetic 4K test source (composable)
+-- src/
|   +-- gst_adapt_node.cpp           # Pipeline lifecycle + GStreamer bus
|   +-- hardware_detector.cpp
|   +-- main.cpp                     # Standalone executable
|   +-- media_streamer_node.cpp
|   +-- pipeline_factory.cpp
|   +-- synthetic_4k_pub_node.cpp
+-- scripts/
|   +-- cpu_monitor.py               # Per-container CPU telemetry
|   +-- latency_tracker.py           # Glass-to-glass latency comparison
|   +-- synthetic_4k_pub.py          # Python fallback publisher
+-- launch/
|   +-- A_B_comparison.launch.py     # Full A/B stress test
|   +-- gst_adapt_demo.launch.py     # Single-node demo
+-- config/
|   +-- demo_params.yaml
+-- CMakeLists.txt
+-- package.xml
+-- LICENSE
+-- README.md
```

## The A/B Performance Stress Test

The launch file `A_B_comparison.launch.py` runs two isolated pipelines
side-by-side, each with its own zero-copy media source:

| Lane | Source | Processing | Output Topic |
|---|---|---|---|
| **Legacy** | `MediaStreamerNode` (same container) | `image_proc::ResizeNode` (CPU `cv::resize`) | `/legacy/image_processed` |
| **Accelerated** | `MediaStreamerNode` (same container) | `gst_adapt_node::GstAdaptNode` (GStreamer VA-API/NVMM) | `/accelerated/image_processed` |

Both lanes resize from source resolution to 640x480. The `latency_tracker`
measures glass-to-glass time and `cpu_monitor` reports per-container CPU usage.

### Running the stress test

```bash
# Prerequisites
sudo apt install ros-humble-image-proc gstreamer1.0-vaapi

# Build
cd ~/Desktop/Current\ Projects/SideProjects/GSTAdaptNode
colcon build --packages-up-to gst_adapt_node
source install/setup.bash

# Run with a real video file
ros2 launch gst_adapt_node A_B_comparison.launch.py video_path:=/path/to/video.mp4

# Or run with the default placeholder (falls back if file doesn't exist)
ros2 launch gst_adapt_node A_B_comparison.launch.py
```

### Expected results

**Latency** (zero-copy intra-process, 4K source):

```
[LEGACY]       latency:   20-65 ms
[ACCELERATED]  latency:   58-155 ms
```

**CPU utilization** (per-container, reported by `cpu_monitor`):

```
[CPU UTIL]  Legacy: ~180% (1.8 cores)  |  Accelerated: ~16% (0.16 cores)
```

**11x CPU reduction.** The accelerated path offloads resize to VA-API/NVMM
hardware while the custom `appsrc` wrapper eliminates `rosimagesrc`'s
internal DDS subscriber overhead. On systems where CPU is the bottleneck
(multi-camera rigs, edge devices running SLAM + perception), the freed cores
more than compensate for the slightly higher pipeline latency.

## Quickstart

### Single-node demo

```bash
colcon build --packages-up-to gst_adapt_node
source install/setup.bash
ros2 launch gst_adapt_node gst_adapt_demo.launch.py
```

### Standalone executable

```bash
ros2 run gst_adapt_node gst_adapt_node \
  --ros-args -p input_topic:=/camera/image_raw -p target_width:=1280 -p target_height:=720
```

## Parameters

### GstAdaptNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `input_topic` | string | `/camera/image_raw` | Source ROS image topic |
| `output_topic` | string | `/camera/image_processed` | Destination ROS image topic |
| `action` | string | `resize` | Pipeline action (`resize`) |
| `target_width` | int | `640` | Output width in pixels |
| `target_height` | int | `480` | Output height in pixels |

### MediaStreamerNode

| Parameter | Type | Default | Description |
|---|---|---|---|
| `video_path` | string | `/tmp/test_video.mp4` | Path to input video file |
| `loop` | bool | `true` | Loop video on EOF |

## Components

All nodes are registered as composable components:

```
gst_adapt_node
  gst_adapt_node::GstAdaptNode
  gst_adapt_node::Synthetic4kPubNode
  gst_adapt_node::MediaStreamerNode
```

Load into any `rclcpp_components::ComponentContainer` with
`use_intra_process_comms: true` for zero-copy operation.

## Dependencies

- ROS 2 Humble (Ubuntu 22.04)
- GStreamer 1.x (`libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`)
- OpenCV 4.x (via `cv_bridge`)
- [ros-gst-bridge](https://github.com/BrettRD/ros-gst-bridge) (workspace
  submodule in `src/ros-gst-bridge`)
- Optional: `gstreamer1.0-vaapi` (Intel VA-API), `ros-humble-image-proc`
  (legacy A/B comparison)

## Roadmap

- [ ] Encode/decode actions (`h264_encode`, `h264_decode`) in `PipelineFactory`
- [ ] ROS 2 Lifecycle node integration for managed state transitions
- [ ] Multi-stream support (N cameras, M pipelines per node)
- [ ] Direct VA-API / NVENC output to shared-memory transport (bypass DDS entirely)
- [ ] Jetson runtime validation on physical hardware
- [ ] Comprehensive integration test suite

## License

Apache-2.0. See [LICENSE](LICENSE).
