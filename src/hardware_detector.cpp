#include "gst_adapt_node/hardware_detector.hpp"

#include <glob.h>
#include <sys/stat.h>

#include <algorithm>

namespace gst_adapt_node
{

// ---------------------------------------------------------------------------
// Enum → string
// ---------------------------------------------------------------------------

const char * to_string(HardwarePlatform platform)
{
  switch (platform) {
    case HardwarePlatform::NVIDIA_JETSON: return "NVIDIA_JETSON";
    case HardwarePlatform::INTEL_VAAPI:   return "INTEL_VAAPI";
    case HardwarePlatform::CPU_FALLBACK:  return "CPU_FALLBACK";
  }
  return "UNKNOWN";
}

// ---------------------------------------------------------------------------
// Filesystem helpers
// ---------------------------------------------------------------------------

bool HardwareDetector::path_exists(const std::string & path)
{
  struct stat st;
  return stat(path.c_str(), &st) == 0;
}

std::vector<std::string> HardwareDetector::glob_paths(const std::string & pattern)
{
  std::vector<std::string> results;
  glob_t buf{};

  if (glob(pattern.c_str(), GLOB_NOSORT, nullptr, &buf) == 0) {
    for (size_t i = 0; i < buf.gl_pathc; ++i) {
      results.emplace_back(buf.gl_pathv[i]);
    }
  }
  globfree(&buf);
  return results;
}

// ---------------------------------------------------------------------------
// NVIDIA Jetson probe
//   Primary indicators: /dev/nvhost-* devices and /dev/nvmap (NVMM allocator).
//   These are present on all Jetson variants (TX2, Xavier, Orin) and absent on
//   desktop NVIDIA GPUs, which makes them a reliable Jetson discriminator.
// ---------------------------------------------------------------------------

bool HardwareDetector::probe_nvidia_jetson(PlatformInfo & info) const
{
  auto nvhost_devices = glob_paths("/dev/nvhost-*");
  bool has_nvmap = path_exists("/dev/nvmap");

  if (nvhost_devices.empty() && !has_nvmap) {
    return false;
  }

  info.platform = HardwarePlatform::NVIDIA_JETSON;

  for (auto & dev : nvhost_devices) {
    info.evidence.push_back(std::move(dev));
  }
  if (has_nvmap) {
    info.evidence.emplace_back("/dev/nvmap");
  }

  return true;
}

// ---------------------------------------------------------------------------
// Intel VA-API probe
//   Looks for DRI render nodes (/dev/dri/renderD*). On Intel systems these
//   back the VA-API encode/decode path used by GStreamer's vaapih264enc etc.
//   We pick the first render node as the default device.
// ---------------------------------------------------------------------------

bool HardwareDetector::probe_intel_vaapi(PlatformInfo & info) const
{
  auto render_nodes = glob_paths("/dev/dri/renderD*");
  if (render_nodes.empty()) {
    return false;
  }

  std::sort(render_nodes.begin(), render_nodes.end());

  info.platform = HardwarePlatform::INTEL_VAAPI;
  info.render_device = render_nodes.front();
  info.evidence = std::move(render_nodes);

  return true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

PlatformInfo HardwareDetector::detect_platform() const
{
  PlatformInfo info{};

  // Jetson takes priority — if we're on a Jetson, the DRI nodes (if present)
  // are not what we want for GStreamer acceleration.
  if (probe_nvidia_jetson(info)) {
    return info;
  }

  if (probe_intel_vaapi(info)) {
    return info;
  }

  info.platform = HardwarePlatform::CPU_FALLBACK;
  return info;
}

}  // namespace gst_adapt_node
