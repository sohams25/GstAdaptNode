#include "gst_adapt_node/pipeline_factory.hpp"

#include <gst/gst.h>
#include <rclcpp/logging.hpp>

#include <sstream>
#include <stdexcept>

namespace gst_adapt_node
{

PlatformInfo validate_platform(
  const PlatformInfo & detected, const rclcpp::Logger & logger)
{
  if (detected.platform == HardwarePlatform::CPU_FALLBACK) {
    return detected;
  }

  const char * required_element = nullptr;
  switch (detected.platform) {
    case HardwarePlatform::INTEL_VAAPI:
      // Try the newer va plugin first, fall back to legacy vaapi plugin
      if (gst_element_factory_find("vapostproc")) {
        RCLCPP_INFO(logger, "Plugin validated: 'vapostproc' found (GStreamer va plugin)");
        return detected;
      }
      required_element = "vaapipostproc";
      break;
    case HardwarePlatform::NVIDIA_JETSON:
      required_element = "nvvideoconvert";
      break;
    default:
      return detected;
  }

  GstElementFactory * factory = gst_element_factory_find(required_element);
  if (factory) {
    gst_object_unref(factory);
    RCLCPP_INFO(logger, "Plugin validated: '%s' found in registry", required_element);
    return detected;
  }

  RCLCPP_WARN(
    logger,
    "*** PLUGIN MISSING: '%s' not found in GStreamer registry "
    "— falling back to CPU_FALLBACK ***",
    required_element);

  PlatformInfo fallback{};
  fallback.platform = HardwarePlatform::CPU_FALLBACK;
  return fallback;
}

// ---------------------------------------------------------------------------

PipelineFactory::PipelineFactory(
  const PlatformInfo & platform, const PipelineConfig & config)
: platform_(platform), config_(config)
{
}

// ---------------------------------------------------------------------------
// Source — appsrc accepting native BGR from ROS Image messages.
// ---------------------------------------------------------------------------

std::string PipelineFactory::source_element() const
{
  std::ostringstream ss;
  ss << "appsrc name=ros_ingest is-live=true block=false format=3"
     << " ! video/x-raw,format=BGR"
     << ",width=" << config_.source_width
     << ",height=" << config_.source_height
     << ",framerate=30/1"
     << ",interlace-mode=progressive";
  return ss.str();
}

std::string PipelineFactory::sink_element() const
{
  return "appsink name=ros_emit caps=video/x-raw,format=(string)BGR sync=false max-buffers=1 drop=true";
}

// ---------------------------------------------------------------------------
// Caps helper
// ---------------------------------------------------------------------------

std::string PipelineFactory::caps(const std::string & memory_type) const
{
  std::ostringstream ss;
  ss << "video/x-raw";
  if (!memory_type.empty()) {
    ss << "(memory:" << memory_type << ")";
  }
  ss << ",width=" << config_.target_width
     << ",height=" << config_.target_height;
  return ss.str();
}

// ---------------------------------------------------------------------------
// Platform-specific resize chains — input is BGR from appsrc.
// ---------------------------------------------------------------------------

std::string PipelineFactory::resize_vaapi() const
{
  // GStreamer 1.22+ with va plugin: full GL bridge, zero CPU conversion
  if (gst_element_factory_find("vapostproc")) {
    return "glupload ! glcolorconvert"
           " ! video/x-raw(memory:GLMemory),format=RGBA"
           " ! vapostproc"
           " ! video/x-raw(memory:VAMemory),format=NV12"
           ",width=" + std::to_string(config_.target_width) +
           ",height=" + std::to_string(config_.target_height) +
           " ! vapostproc"
           " ! video/x-raw(memory:VAMemory),format=BGRA"
           " ! gldownload ! glcolorconvert"
           " ! video/x-raw,format=BGR";
  }

  // GStreamer 1.20 with gstreamer-vaapi: vaapipostproc has a known chroma
  // loss bug on many Intel iGPUs (UV plane is zeroed during download).
  // Use CPU videoscale for correctness, still benefiting from the zero-copy
  // appsrc/appsink architecture (no rosimagesrc/rosimagesink overhead).
  return resize_cpu();
}

// NVIDIA Jetson: CUDA cores ingest BGR directly via compute-hw=1 override.
std::string PipelineFactory::resize_jetson() const
{
  std::ostringstream ss;
  ss << "nvvideoconvert compute-hw=1 nvbuf-memory-type=2"
     << " ! video/x-raw(memory:NVMM),format=NV12"
     << " ! nvvideoconvert compute-hw=1 nvbuf-memory-type=0 interpolation-method=1"
     << " ! video/x-raw,format=BGR"
     << ",width=" << config_.target_width
     << ",height=" << config_.target_height;
  return ss.str();
}

// CPU: software path.
std::string PipelineFactory::resize_cpu() const
{
  return "videoscale ! videoconvert ! " + caps();
}

std::string PipelineFactory::resize_chain() const
{
  switch (platform_.platform) {
    case HardwarePlatform::NVIDIA_JETSON: return resize_jetson();
    case HardwarePlatform::INTEL_VAAPI:   return resize_vaapi();
    case HardwarePlatform::CPU_FALLBACK:  return resize_cpu();
  }
  return resize_cpu();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

std::string PipelineFactory::build() const
{
  if (config_.action != "resize") {
    throw std::invalid_argument(
      "Unsupported action '" + config_.action + "' — only 'resize' is implemented");
  }

  return source_element() + " ! " + resize_chain() + " ! " + sink_element();
}

}  // namespace gst_adapt_node
