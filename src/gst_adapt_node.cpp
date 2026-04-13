#include "gst_adapt_node/gst_adapt_node.hpp"
#include "gst_adapt_node/pipeline_factory.hpp"

#include <rclcpp_components/register_node_macro.hpp>

using namespace std::chrono_literals;

namespace gst_adapt_node
{

GstAdaptNode::GstAdaptNode(const rclcpp::NodeOptions & options)
: Node("gst_adapt_node", rclcpp::NodeOptions(options).use_intra_process_comms(true))
{
  gst_init(nullptr, nullptr);

  declare_parameters();
  detect_hardware();
  build_pipeline();
  launch_pipeline();
}

GstAdaptNode::~GstAdaptNode()
{
  shutdown_pipeline();
}

// ---------------------------------------------------------------------------
// Parameter declaration
// ---------------------------------------------------------------------------

void GstAdaptNode::declare_parameters()
{
  declare_parameter("input_topic", "/camera/image_raw");
  declare_parameter("output_topic", "/camera/image_processed");
  declare_parameter("action", "resize");
  declare_parameter("source_width", 3840);
  declare_parameter("source_height", 2160);
  declare_parameter("target_width", 640);
  declare_parameter("target_height", 480);
}

// ---------------------------------------------------------------------------
// Hardware detection + plugin validation
// ---------------------------------------------------------------------------

void GstAdaptNode::detect_hardware()
{
  HardwareDetector detector;
  platform_info_ = detector.detect_platform();

  RCLCPP_INFO(
    get_logger(), "Detected platform: %s",
    to_string(platform_info_.platform));

  if (!platform_info_.render_device.empty()) {
    RCLCPP_INFO(
      get_logger(), "Render device: %s",
      platform_info_.render_device.c_str());
  }

  for (const auto & path : platform_info_.evidence) {
    RCLCPP_DEBUG(get_logger(), "  evidence: %s", path.c_str());
  }

  platform_info_ = validate_platform(platform_info_, get_logger());
}

// ---------------------------------------------------------------------------
// Pipeline string generation
// ---------------------------------------------------------------------------

void GstAdaptNode::build_pipeline()
{
  PipelineConfig config;
  config.input_topic   = get_parameter("input_topic").as_string();
  config.output_topic  = get_parameter("output_topic").as_string();
  config.action        = get_parameter("action").as_string();
  config.source_width  = get_parameter("source_width").as_int();
  config.source_height = get_parameter("source_height").as_int();
  config.target_width  = get_parameter("target_width").as_int();
  config.target_height = get_parameter("target_height").as_int();

  PipelineFactory factory(platform_info_, config);
  pipeline_string_ = factory.build();

  RCLCPP_INFO(get_logger(), "Generated pipeline: %s", pipeline_string_.c_str());
}

// ---------------------------------------------------------------------------
// GStreamer execution + zero-copy subscriber
// ---------------------------------------------------------------------------

void GstAdaptNode::launch_pipeline()
{
  GError * error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_string_.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(get_logger(), "Pipeline parse error: %s", error->message);
    g_error_free(error);
    pipeline_ = nullptr;
    return;
  }

  appsrc_ = gst_bin_get_by_name(GST_BIN(pipeline_), "ros_source");
  if (!appsrc_) {
    RCLCPP_ERROR(get_logger(), "Failed to find appsrc element 'ros_source'");
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING");
    gst_object_unref(appsrc_);
    appsrc_ = nullptr;
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    return;
  }

  // Subscribe with unique_ptr lambda for zero-copy intra-process transfer.
  // Lambda (not std::bind) is required so rclcpp correctly deduces the
  // UniquePtr callback signature for the intra-process take-ownership path.
  auto topic = get_parameter("input_topic").as_string();
  image_sub_ = create_subscription<sensor_msgs::msg::Image>(
    topic, 10,
    [this](sensor_msgs::msg::Image::UniquePtr msg) {
      on_image(std::move(msg));
    });

  RCLCPP_INFO(get_logger(), "Pipeline launched — appsrc fed from %s (zero-copy)", topic.c_str());

  bus_timer_ = create_wall_timer(100ms, std::bind(&GstAdaptNode::poll_bus, this));
}

// ---------------------------------------------------------------------------
// Zero-copy image callback — wraps the ROS message buffer directly into a
// GstBuffer without copying. The GDestroyNotify ensures the Image is freed
// only after GStreamer finishes processing the buffer.
// ---------------------------------------------------------------------------

void GstAdaptNode::destroy_ros_image(gpointer user_data)
{
  auto * msg = static_cast<sensor_msgs::msg::Image *>(user_data);
  delete msg;
}

void GstAdaptNode::on_image(sensor_msgs::msg::Image::UniquePtr msg)
{
  if (!appsrc_) { return; }

  auto * raw_msg = msg.release();
  uint8_t * data = raw_msg->data.data();
  gsize size = raw_msg->data.size();

  GstBuffer * buffer = gst_buffer_new_wrapped_full(
    static_cast<GstMemoryFlags>(0),
    data, size, 0, size,
    raw_msg, destroy_ros_image);

  GstFlowReturn flow = gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer);
  if (flow != GST_FLOW_OK) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "appsrc push failed: %s", gst_flow_get_name(flow));
  }
}

// ---------------------------------------------------------------------------
// Bus message pump
// ---------------------------------------------------------------------------

void GstAdaptNode::poll_bus()
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

void GstAdaptNode::shutdown_pipeline()
{
  if (bus_timer_) {
    bus_timer_->cancel();
    bus_timer_.reset();
  }

  image_sub_.reset();

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

}  // namespace gst_adapt_node

RCLCPP_COMPONENTS_REGISTER_NODE(gst_adapt_node::GstAdaptNode)
