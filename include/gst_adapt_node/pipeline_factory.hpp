#ifndef GST_ADAPT_NODE__PIPELINE_FACTORY_HPP_
#define GST_ADAPT_NODE__PIPELINE_FACTORY_HPP_

#include <string>

#include <rclcpp/logger.hpp>

#include "gst_adapt_node/hardware_detector.hpp"

namespace gst_adapt_node
{

struct PipelineConfig
{
  std::string input_topic  = "/camera/image_raw";
  std::string output_topic = "/camera/image_processed";
  std::string action       = "resize";
  int source_width         = 3840;
  int source_height        = 2160;
  int target_width         = 640;
  int target_height        = 480;
};

// Checks that the GStreamer elements required by the detected platform are
// actually present in the registry. Falls back to CPU_FALLBACK if not.
// Must be called after gst_init().
PlatformInfo validate_platform(const PlatformInfo & detected, const rclcpp::Logger & logger);

class PipelineFactory
{
public:
  PipelineFactory(const PlatformInfo & platform, const PipelineConfig & config);

  std::string build() const;

private:
  PlatformInfo platform_;
  PipelineConfig config_;

  std::string source_element() const;
  std::string sink_element() const;
  std::string resize_chain() const;

  std::string resize_jetson() const;
  std::string resize_vaapi() const;
  std::string resize_cpu() const;

  std::string caps(const std::string & memory_type = {}) const;
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__PIPELINE_FACTORY_HPP_
