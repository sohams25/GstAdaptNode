#include "gst_adapt_node/gst_adapt_node.hpp"
#include "gst_adapt_node/pipeline_factory.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <gst/video/video.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cstring>

using namespace std::chrono_literals;

namespace gst_adapt_node
{

ResizeNode::ResizeNode(const rclcpp::NodeOptions & options)
: Node("resize_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  gst_init(nullptr, nullptr);

  declare_parameters();
  detect_hardware();

  // Determine if a WORKING GPU accelerator is available.
  // On GStreamer 1.20, vaapipostproc exists but has a chroma loss bug —
  // only vapostproc (GStreamer 1.22+) or nvvideoconvert (Jetson) actually work.
  bool has_working_gpu =
    (platform_info_.platform == HardwarePlatform::NVIDIA_JETSON) ||
    (platform_info_.platform == HardwarePlatform::INTEL_VAAPI &&
     gst_element_factory_find("vapostproc"));

  if (has_working_gpu) {
    build_pipeline();
    launch_pipeline();
  } else {
    launch_direct();
  }
}

ResizeNode::~ResizeNode()
{
  shutdown_pipeline();
}

void ResizeNode::declare_parameters()
{
  declare_parameter("input_topic", "/camera/image_raw");
  declare_parameter("output_topic", "/camera/image_processed");
  declare_parameter("action", "resize");
  declare_parameter("source_width", 3840);
  declare_parameter("source_height", 2160);
  declare_parameter("use_scale", false);
  declare_parameter("scale_height", 1.0);
  declare_parameter("scale_width", 1.0);
  declare_parameter("height", 480);
  declare_parameter("width", 640);
}

void ResizeNode::detect_hardware()
{
  HardwareDetector detector;
  platform_info_ = detector.detect_platform();

  RCLCPP_INFO(get_logger(), "Detected platform: %s",
    to_string(platform_info_.platform));

  if (!platform_info_.render_device.empty()) {
    RCLCPP_INFO(get_logger(), "Render device: %s",
      platform_info_.render_device.c_str());
  }

  for (const auto & path : platform_info_.evidence) {
    RCLCPP_DEBUG(get_logger(), "  evidence: %s", path.c_str());
  }

  platform_info_ = validate_platform(platform_info_, get_logger());
}

void ResizeNode::build_pipeline()
{
  PipelineConfig config;
  config.input_topic   = get_parameter("input_topic").as_string();
  config.output_topic  = get_parameter("output_topic").as_string();
  config.action        = get_parameter("action").as_string();
  config.source_width  = get_parameter("source_width").as_int();
  config.source_height = get_parameter("source_height").as_int();

  bool use_scale = get_parameter("use_scale").as_bool();
  if (use_scale) {
    config.target_width = static_cast<int>(
      config.source_width * get_parameter("scale_width").as_double());
    config.target_height = static_cast<int>(
      config.source_height * get_parameter("scale_height").as_double());
  } else {
    config.target_width  = get_parameter("width").as_int();
    config.target_height = get_parameter("height").as_int();
  }

  PipelineFactory factory(platform_info_, config);
  pipeline_string_ = factory.build();

  RCLCPP_INFO(get_logger(), "Generated pipeline: %s", pipeline_string_.c_str());
}

// ---------------------------------------------------------------------------
// GStreamer execution: appsrc (input) + appsink (output)
// ---------------------------------------------------------------------------

void ResizeNode::launch_pipeline()
{
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_string_.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
    g_error_free(error);
    pipeline_ = nullptr;
    return;
  }

  // Get appsrc (input)
  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_ingest");
  if (!appsrc_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsrc 'ros_ingest'");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  g_object_set(G_OBJECT(appsrc_),
    "is-live", TRUE,
    "format", GST_FORMAT_TIME,
    "block", FALSE,
    "max-bytes", static_cast<guint64>(75 * 1024 * 1024),  // ~3 BGR 4K frames
    "max-buffers", static_cast<guint64>(3),
    nullptr);

  // Get appsink (output) and wire up the new-sample signal
  appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_emit");
  if (!appsink_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsink 'ros_emit'");
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  g_object_set(G_OBJECT(appsink_), "emit-signals", TRUE, nullptr);
  g_signal_connect(appsink_, "new-sample", G_CALLBACK(on_new_sample), this);

  // Create the output publisher
  auto output_topic = get_parameter("output_topic").as_string();
  output_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
    gst_object_unref(appsink_);
    appsink_ = nullptr;
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  auto input_topic = get_parameter("input_topic").as_string();
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic, 10,
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      on_image(std::move(msg));
    });

  RCLCPP_INFO(get_logger(),
    "Pipeline launched — appsrc(%s) -> appsink(%s) (zero-copy)",
    input_topic.c_str(), output_topic.c_str());

  bus_timer_ = create_wall_timer(100ms, std::bind(&ResizeNode::poll_bus, this));
}

// ---------------------------------------------------------------------------
// appsrc input: zero-copy buffer wrapping
// ---------------------------------------------------------------------------

void ResizeNode::destroy_ros_image(gpointer user_data)
{
  auto * msg = static_cast<sensor_msgs::msg::Image *>(user_data);
  delete msg;
}

void ResizeNode::on_image(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!appsrc_) { return; }

  // Extract the ROS timestamp BEFORE releasing ownership
  GstClockTime pts =
    static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
    msg->header.stamp.nanosec;

  auto * raw_msg = msg.release();
  uint8_t * data = raw_msg->data.data();
  gsize size = raw_msg->data.size();

  GstBuffer * buffer = gst_buffer_new_wrapped_full(
    static_cast<GstMemoryFlags>(0),
    data, size, 0, size,
    raw_msg, destroy_ros_image);

  // Tunnel the ROS timestamp through GStreamer as the buffer PTS
  GST_BUFFER_PTS(buffer) = pts;

  GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (flow != GST_FLOW_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "appsrc push failed: %s", gst_flow_get_name(flow));
  }
}

// ---------------------------------------------------------------------------
// appsink output: pull processed frames and publish as ROS Image.
// Called on GStreamer's streaming thread — publishers are thread-safe.
// ---------------------------------------------------------------------------

GstFlowReturn ResizeNode::on_new_sample(GstAppSink * sink, gpointer user_data)
{
  auto * self = static_cast<ResizeNode *>(user_data);

  GstSample * sample = gst_app_sink_pull_sample(sink);
  if (!sample) { return GST_FLOW_ERROR; }

  GstBuffer * buffer = gst_sample_get_buffer(sample);
  GstCaps * caps = gst_sample_get_caps(sample);

  GstVideoInfo info;
  if (!gst_video_info_from_caps(&info, caps)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ)) {
    gst_sample_unref(sample);
    return GST_FLOW_ERROR;
  }

  int w = info.width;
  int h = info.height;
  GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT(&info);

  // Convert any GStreamer format to BGR via OpenCV — handles stride and planes
  cv::Mat bgr_out;

  if (fmt == GST_VIDEO_FORMAT_BGR) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC3,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    bgr_out = raw.clone();
  } else if (fmt == GST_VIDEO_FORMAT_BGRA || fmt == GST_VIDEO_FORMAT_BGRx) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC4,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    cv::cvtColor(raw, bgr_out, cv::COLOR_BGRA2BGR);
  } else if (fmt == GST_VIDEO_FORMAT_RGBA || fmt == GST_VIDEO_FORMAT_RGBx) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC4,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    cv::cvtColor(raw, bgr_out, cv::COLOR_RGBA2BGR);
  } else if (fmt == GST_VIDEO_FORMAT_RGB) {
    int stride = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    cv::Mat raw(h, w, CV_8UC3,
      const_cast<void *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0)),
      static_cast<size_t>(stride));
    cv::cvtColor(raw, bgr_out, cv::COLOR_RGB2BGR);
  } else if (fmt == GST_VIDEO_FORMAT_NV12) {
    // Y plane + interleaved UV plane — assemble for OpenCV
    cv::Mat nv12(h * 3 / 2, w, CV_8UC1);
    auto * y_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    auto * uv_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
    int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    int uvs = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    for (int r = 0; r < h; ++r)
      std::memcpy(nv12.ptr(r), y_src + r * ys, w);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(nv12.ptr(h + r), uv_src + r * uvs, w);
    cv::cvtColor(nv12, bgr_out, cv::COLOR_YUV2BGR_NV12);
  } else if (fmt == GST_VIDEO_FORMAT_I420 || fmt == GST_VIDEO_FORMAT_YV12) {
    // 3 separate planes: Y, U, V
    cv::Mat yuv(h * 3 / 2, w, CV_8UC1);
    auto * y_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 0));
    auto * u_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 1));
    auto * v_src = static_cast<const uint8_t *>(GST_VIDEO_FRAME_PLANE_DATA(&frame, 2));
    int ys = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    int us = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 1);
    int vs = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 2);
    for (int r = 0; r < h; ++r)
      std::memcpy(yuv.ptr(r), y_src + r * ys, w);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(yuv.ptr(h + r), u_src + r * us, w / 2);
    for (int r = 0; r < h / 2; ++r)
      std::memcpy(yuv.ptr(h + h / 2 + r), v_src + r * vs, w / 2);
    int code = (fmt == GST_VIDEO_FORMAT_I420) ?
      cv::COLOR_YUV2BGR_I420 : cv::COLOR_YUV2BGR_YV12;
    cv::cvtColor(yuv, bgr_out, code);
  } else {
    RCLCPP_WARN_THROTTLE(self->get_logger(), *self->get_clock(), 2000,
      "Unsupported appsink format: %s — skipping",
      gst_video_format_to_string(fmt));
    gst_video_frame_unmap(&frame);
    gst_sample_unref(sample);
    return GST_FLOW_OK;
  }

  // Extract PTS before unmapping
  GstClockTime pts = GST_BUFFER_PTS(buffer);

  gst_video_frame_unmap(&frame);
  gst_sample_unref(sample);

  // Publish guaranteed-correct BGR frame
  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  if (GST_CLOCK_TIME_IS_VALID(pts)) {
    msg->header.stamp.sec = static_cast<int32_t>(pts / 1000000000ULL);
    msg->header.stamp.nanosec = static_cast<uint32_t>(pts % 1000000000ULL);
  } else {
    msg->header.stamp = self->now();
  }
  msg->header.frame_id = "media_frame";
  msg->width = bgr_out.cols;
  msg->height = bgr_out.rows;
  msg->encoding = "bgr8";
  msg->step = bgr_out.cols * 3;
  msg->is_bigendian = false;
  msg->data.assign(bgr_out.data, bgr_out.data + bgr_out.total() * bgr_out.elemSize());

  self->output_pub_->publish(std::move(msg));
  return GST_FLOW_OK;
}

// ---------------------------------------------------------------------------
// Bus message pump
// ---------------------------------------------------------------------------

void ResizeNode::poll_bus()
{
  if (!pipeline_) { return; }

  GstBus * bus = gst_element_get_bus(pipeline_);
  GstMessage * msg = gst_bus_pop_filtered(
    bus,
    static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

  if (msg) {
    switch (GST_MESSAGE_TYPE(msg)) {
      case GST_MESSAGE_ERROR: {
        GError * err = nullptr;
        gchar * debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        RCLCPP_ERROR(get_logger(), "GStreamer error: %s", err->message);
        if (debug) {
          RCLCPP_DEBUG(get_logger(), "  debug: %s", debug);
          g_free(debug);
        }
        g_error_free(err);
        shutdown_pipeline();
        break;
      }
      case GST_MESSAGE_EOS:
        RCLCPP_INFO(get_logger(), "GStreamer end-of-stream");
        shutdown_pipeline();
        break;
      default:
        break;
    }
    gst_message_unref(msg);
  }
  gst_object_unref(bus);
}

// ---------------------------------------------------------------------------
// Teardown
// ---------------------------------------------------------------------------

void ResizeNode::shutdown_pipeline()
{
  if (bus_timer_) {
    bus_timer_->cancel();
    bus_timer_.reset();
  }

  image_sub_.reset();

  if (appsink_) {
    gst_object_unref(appsink_);
    appsink_ = nullptr;
  }

  if (appsrc_) {
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
  }

  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    RCLCPP_INFO(get_logger(), "Pipeline shut down");
  }
}

// ---------------------------------------------------------------------------
// Direct mode — OpenCV resize, zero GStreamer overhead.
// Used when no working GPU accelerator is available.
// ---------------------------------------------------------------------------

void ResizeNode::launch_direct()
{
  direct_mode_ = true;

  bool use_scale = get_parameter("use_scale").as_bool();
  int src_w = get_parameter("source_width").as_int();
  int src_h = get_parameter("source_height").as_int();
  if (use_scale) {
    target_w_ = static_cast<int>(src_w * get_parameter("scale_width").as_double());
    target_h_ = static_cast<int>(src_h * get_parameter("scale_height").as_double());
  } else {
    target_w_ = get_parameter("width").as_int();
    target_h_ = get_parameter("height").as_int();
  }

  auto output_topic = get_parameter("output_topic").as_string();
  output_pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, 10);

  auto input_topic = get_parameter("input_topic").as_string();
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    input_topic, 10,
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      on_image_direct(std::move(msg));
    });

  RCLCPP_INFO(get_logger(),
    "Direct mode — cv::resize(%s → %dx%d) on %s (no GStreamer overhead)",
    input_topic.c_str(), target_w_, target_h_, output_topic.c_str());
}

void ResizeNode::on_image_direct(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!output_pub_) { return; }
  if (msg->width == 0 || msg->height == 0) { return; }

  // Wrap the incoming BGR data as a cv::Mat (zero-copy reference)
  cv::Mat src(msg->height, msg->width, CV_8UC3, msg->data.data(), msg->step);

  // Resize
  cv::Mat dst;
  cv::resize(src, dst, cv::Size(target_w_, target_h_), 0, 0, cv::INTER_LINEAR);

  // Publish
  auto out = std::make_unique<sensor_msgs::msg::Image>();
  out->header = msg->header;
  out->width = dst.cols;
  out->height = dst.rows;
  out->encoding = "bgr8";
  out->step = dst.cols * 3;
  out->is_bigendian = false;
  out->data.assign(dst.data, dst.data + dst.total() * dst.elemSize());

  output_pub_->publish(std::move(out));
}

}  // namespace gst_adapt_node

RCLCPP_COMPONENTS_REGISTER_NODE(gst_adapt_node::ResizeNode)
