#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace hikrobot_camera
{

// 把 SDK 返回值变成 "0x80000203（...）" 这样的文字（定义在 camera_device.cpp）
std::string sdkErrorToString(int ret);

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  // ---- 阶段 5：选择、打开、关闭相机（camera_device.cpp）----
  bool openCamera();
  void closeCamera();

  // 启动时读一次的参数
  std::string serial_number_;
  std::string ip_address_;

  // 相机句柄：后面阶段会有多个线程用到它，读写前先锁 camera_mutex_
  std::mutex camera_mutex_;
  void * handle_ = nullptr;
  bool sdk_initialized_ = false;
  int last_device_count_ = -1;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_
