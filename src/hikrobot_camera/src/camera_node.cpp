#include "hikrobot_camera/camera_node.hpp"

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

  // 2. 图像发布者：队列长度 5，可靠传输（RViz2 默认的 Reliable 能收到）
  image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, rclcpp::QoS(5));

  // 3. 初始化 SDK
  const int ret = MV_CC_Initialize();
  if (ret != MV_OK) {
    RCLCPP_FATAL(get_logger(), "MV_CC_Initialize 失败：%s", sdkErrorToString(ret).c_str());
    return;
  }
  sdk_initialized_ = true;

  // 4. 打开相机 → 开始取流 → 启动取图线程
  if (!openCamera()) {
    RCLCPP_ERROR(get_logger(), "相机没有打开。节点继续运行，但不会出图（阶段 8 会加上自动重试）");
    return;
  }
  if (!startGrabbing()) {
    return;
  }
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
