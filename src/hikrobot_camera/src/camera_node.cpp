#include "hikrobot_camera/camera_node.hpp"

#include "MvCameraControl.h"

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_WARN(
    get_logger(),
    "Training scaffold only: camera connection, image publishing, and camera "
    "parameter control are NOT implemented.");

  // 版本号 4 个字节：主.次.修正.测试，例如 0x04080201 = V4.8.2.1
  const unsigned int sdk_version = MV_CC_GetSDKVersion();
  RCLCPP_INFO(
    get_logger(), "MVS SDK version: 0x%08X (V%u.%u.%u.%u)", sdk_version,
    (sdk_version >> 24) & 0xFF, (sdk_version >> 16) & 0xFF,
    (sdk_version >> 8) & 0xFF, sdk_version & 0xFF);

  // TODO(student): Implement the requirements in docs/assignment.md.
  // - Device selection and connection.
  // - Image acquisition and sensor_msgs/msg/Image publishing.
  // - Camera parameter inspection and updates.
  // - Disconnection recovery and resource cleanup.
}

}  // namespace hikrobot_camera
