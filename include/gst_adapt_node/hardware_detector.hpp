#ifndef GST_ADAPT_NODE__HARDWARE_DETECTOR_HPP_
#define GST_ADAPT_NODE__HARDWARE_DETECTOR_HPP_

#include <string>
#include <vector>

namespace gst_adapt_node
{

enum class HardwarePlatform
{
  NVIDIA_JETSON,
  INTEL_VAAPI,
  CPU_FALLBACK
};

const char * to_string(HardwarePlatform platform);

struct PlatformInfo
{
  HardwarePlatform platform;
  std::string render_device;          // e.g. "/dev/dri/renderD128"
  std::vector<std::string> evidence;  // paths that matched during probing
};

class HardwareDetector
{
public:
  PlatformInfo detect_platform() const;

private:
  bool probe_nvidia_jetson(PlatformInfo & info) const;
  bool probe_intel_vaapi(PlatformInfo & info) const;
  static bool path_exists(const std::string & path);
  static std::vector<std::string> glob_paths(const std::string & pattern);
};

}  // namespace gst_adapt_node

#endif  // GST_ADAPT_NODE__HARDWARE_DETECTOR_HPP_
