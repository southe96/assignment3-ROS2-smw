#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <functional>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

namespace
{
// 生成参数说明；read_only = true 表示节点运行中不能用 ros2 param set 修改
rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text, bool read_only)
{
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.description = text;
  descriptor.read_only = read_only;
  return descriptor;
}
}  // namespace

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  // 版本号 4 个字节：主.次.修正.测试，例如 0x04080201 = V4.8.2.1
  const unsigned int sdk_version = MV_CC_GetSDKVersion();
  RCLCPP_INFO(
    get_logger(), "MVS SDK version: 0x%08X (V%u.%u.%u.%u)", sdk_version,
    (sdk_version >> 24) & 0xFF, (sdk_version >> 16) & 0xFF,
    (sdk_version >> 8) & 0xFF, sdk_version & 0xFF);

  // 1. 声明参数（这里写的是默认值，camera.yaml 里的值会覆盖它）
  serial_number_ = declare_parameter<std::string>(
    "serial_number", "", describe("要打开的相机序列号，留空表示不按序列号筛选", true));
  ip_address_ = declare_parameter<std::string>(
    "ip_address", "", describe("要打开的网口相机 IP，例如 192.168.1.10；留空表示不按 IP 筛选", true));
  const std::string image_topic = declare_parameter<std::string>(
    "image_topic", "image_raw", describe("图像话题名", true));
  frame_id_ = declare_parameter<std::string>(
    "frame_id", "camera_optical_frame", describe("图像消息 header.frame_id", true));
  reconnect_interval_ = declare_parameter<double>(
    "reconnect_interval", 1.0, describe("打开失败或断线后，每隔多少秒重试一次", true));
  declare_parameter<double>(
    "exposure_time", 5000.0,
    describe("曝光时间，单位 us（微秒）；设置时自动关闭自动曝光；范围以相机为准", false));
  declare_parameter<double>(
    "gain", 0.0, describe("增益，单位 dB；设置时自动关闭自动增益；范围以相机为准", false));
  declare_parameter<double>(
    "frame_rate", 0.0, describe("帧率上限，单位 fps；0 表示不限制（相机尽量快）", false));
  declare_parameter<std::string>(
    "pixel_format", "YUV422_YUYV_Packed",
    describe("像素格式，可选：" + supportedPixelFormatNames(), false));

  // 2. 参数回调：每次 ros2 param set 都会先经过它，由它决定这次修改成功还是失败
  param_callback_handle_ = add_on_set_parameters_callback(
    std::bind(&CameraNode::onSetParameters, this, std::placeholders::_1));

  // 3. 图像发布者：队列长度 5，可靠传输（RViz2 默认的 Reliable 能收到）
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::QoS(5));

  // 4. 每 5 秒打印一次帧率：设置帧率、相机实际帧率、节点发布帧率
  last_fps_time_ = now();
  fps_timer_ = create_wall_timer(
    std::chrono::seconds(5), std::bind(&CameraNode::reportFrameRate, this));

  // 5. 初始化 SDK
  const int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    RCLCPP_FATAL(get_logger(), "MV_CC_Initialize 失败：%s", sdkErrorToString(ret).c_str());
    return;
  }
  sdk_initialized_ = true;

  // 6. 启动取图线程：打开相机、恢复参数、取图、断线重连都在这个线程里做
  running_ = true;
  grab_thread_ = std::thread(&CameraNode::grabLoop, this);
}

CameraNode::~CameraNode()
{
  // 先让取图线程退出，再按 StopGrabbing → CloseDevice → DestroyHandle → Finalize 的顺序释放
  running_ = false;
  if (grab_thread_.joinable()) {
    grab_thread_.join();
  }
  closeCamera();
  if (sdk_initialized_) {
    MV_CC_Finalize();
  }
  RCLCPP_INFO(get_logger(), "节点退出，SDK 资源已释放");
}

}  // namespace hikrobot_camera
