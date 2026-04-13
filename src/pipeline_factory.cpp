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
      required_element = "vaapipostproc";
      break;
    case HardwarePlatform::NVIDIA_JETSON:
      required_element = "nvvidconv";
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
// Source — custom appsrc fed by GstAdaptNode's zero-copy subscriber.
// Caps declare I420 at source resolution so hardware elements negotiate
// without any CPU-side videoconvert.
// ---------------------------------------------------------------------------

std::string PipelineFactory::source_element() const
{
  std::ostringstream ss;
  ss << "appsrc name=ros_source is-live=true block=false format=3"
     << " ! video/x-raw,format=I420"
     << ",width=" << config_.source_width
     << ",height=" << config_.source_height
     << ",framerate=30/1"
     << ",interlace-mode=progressive";
  return ss.str();
}

std::string PipelineFactory::sink_element() const
{
  return "rosimagesink ros-topic=" + config_.output_topic + " sync=false max-buffers=1 drop=true";
}

// ---------------------------------------------------------------------------
// Caps helper — builds a video/x-raw caps string with optional memory type
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
// Platform-specific resize chains
// Input is I420 from appsrc — no videoconvert needed before hardware elements.
// ---------------------------------------------------------------------------

// Jetson: I420 is nvvidconv's native input. Upload + resize on GPU,
// download, then videoconvert to BGR for rosimagesink.
std::string PipelineFactory::resize_jetson() const
{
  return "nvvidconv ! " + caps("NVMM") +
         " ! nvvidconv ! videoconvert ! video/x-raw,format=BGR";
}

// Intel VA-API: I420 is vaapipostproc's native input. Upload + resize on GPU,
// download at target res, then videoconvert to BGR for rosimagesink.
std::string PipelineFactory::resize_vaapi() const
{
  return "vaapipostproc ! " + caps("VASurface") +
         " ! vaapipostproc ! videoconvert ! video/x-raw,format=BGR";
}

// CPU: software path — videoscale for resampling, videoconvert for format fixup.
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
// Public API — assembles the full pipeline string
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
