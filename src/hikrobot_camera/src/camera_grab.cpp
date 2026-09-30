#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "MvCameraControl.h"
#include "sensor_msgs/image_encodings.hpp"

namespace hikrobot_camera
{

namespace
{
namespace enc = sensor_msgs::image_encodings;

const PixelFormatInfo kPixelFormats[] = {
  {"Mono8", PixelType_Gvsp_Mono8, enc::MONO8, 1},
  {"BayerRG8", PixelType_Gvsp_BayerRG8, enc::BAYER_RGGB8, 1},
  {"BayerGR8", PixelType_Gvsp_BayerGR8, enc::BAYER_GRBG8, 1},
  {"BayerGB8", PixelType_Gvsp_BayerGB8, enc::BAYER_GBRG8, 1},
  {"BayerBG8", PixelType_Gvsp_BayerBG8, enc::BAYER_BGGR8, 1},
  {"RGB8Packed", PixelType_Gvsp_RGB8_Packed, enc::RGB8, 3},
  {"BGR8Packed", PixelType_Gvsp_BGR8_Packed, enc::BGR8, 3},
  {"YUV422_YUYV_Packed", PixelType_Gvsp_YUV422_YUYV_Packed, enc::YUV422_YUY2, 2},
  {"YUV422_Packed", PixelType_Gvsp_YUV422_Packed, enc::YUV422, 2},
};

constexpr unsigned int kGrabTimeoutMs = 100;
} 

const PixelFormatInfo * findPixelFormatByName(const std::string & name)
{
  for (const auto & format : kPixelFormats) {
    if (name == format.name) {
      return &format;
    }
  }
  return nullptr;
}

const PixelFormatInfo * findPixelFormatByValue(unsigned int sdk_value)
{
  for (const auto & format : kPixelFormats) {
    if (sdk_value == format.sdk_value) {
      return &format;
    }
  }
  return nullptr;
}

std::string supportedPixelFormatNames()
{
  std::string names;
  for (const auto & format : kPixelFormats) {
    if (!names.empty()) {
      names += ", ";
    }
    names += format.name;
  }
  return names;
}

bool CameraNode::startGrabbing()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  const int ret = MV_CC_StartGrabbing(handle_);
  if (ret != MV_OK) {
    RCLCPP_ERROR(get_logger(), "开始取流失败：%s", sdkErrorToString(ret).c_str());
    return false;
  }
  grabbing_ = true;
  RCLCPP_INFO(get_logger(), "开始取流，图像发布到 %s", image_pub_->get_topic_name());
  return true;
}

CameraNode::GrabResult CameraNode::grabOnce(sensor_msgs::msg::Image & msg)
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (handle_ == nullptr || !grabbing_) {
    return GrabResult::kError;
  }

  MV_FRAME_OUT frame;
  std::memset(&frame, 0, sizeof(frame));
  const int ret = MV_CC_GetImageBuffer(handle_, &frame, kGrabTimeoutMs);
  if (ret == static_cast<int>(MV_E_NODATA)) {
    return GrabResult::kNoFrame; 
  }
  if (ret != MV_OK) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "取图失败：%s", sdkErrorToString(ret).c_str());
    return GrabResult::kError;
  }

  GrabResult result = GrabResult::kNoFrame;
  const MV_FRAME_OUT_INFO_EX & info = frame.stFrameInfo;
  const PixelFormatInfo * format = findPixelFormatByValue(info.enPixelType);
  if (format == nullptr) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "相机当前像素格式 0x%08X 本节点不支持，请把 pixel_format 设为其中之一：%s",
      static_cast<unsigned int>(info.enPixelType), supportedPixelFormatNames().c_str());
  } else {
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.width = info.nWidth;
    msg.height = info.nHeight;
    msg.encoding = format->encoding;
    msg.is_bigendian = false;
    msg.step = msg.width * format->bytes_per_pixel;
    const size_t size = static_cast<size_t>(msg.step) * msg.height;
    if (info.nFrameLen < size) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "帧数据长度 %u 小于应有的 %zu，丢弃这一帧",
        info.nFrameLen, size);
    } else {
      msg.data.assign(frame.pBufAddr, frame.pBufAddr + size);
      result = GrabResult::kFrame;
    }
  }

  MV_CC_FreeImageBuffer(handle_, &frame);
  return result;
}

void CameraNode::grabLoop()
{
  int error_count = 0;
  while (running_ && rclcpp::ok()) {
    if (handle_ == nullptr) {
      if (!openCamera()) {
        sleepWhileRunning(reconnect_interval_);
        continue;
      }
      applyAllCameraParameters();
      if (!startGrabbing()) {
        closeCamera();
        sleepWhileRunning(reconnect_interval_);
        continue;
      }
      error_count = 0;
    }

    if (disconnected_ || error_count >= 10) {
      RCLCPP_WARN(
        get_logger(), "相机连接断开，释放旧句柄，每 %.1f 秒尝试重连一次", reconnect_interval_);
      closeCamera();
      disconnected_ = false;
      sleepWhileRunning(reconnect_interval_);
      continue;
    }

    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    const GrabResult result = grabOnce(*msg);
    if (result == GrabResult::kFrame) {
      error_count = 0;
      image_pub_->publish(std::move(msg));
      ++published_frames_;
    } else if (result == GrabResult::kError) {
      ++error_count;
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void CameraNode::sleepWhileRunning(double seconds)
{
  const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
  while (running_ && rclcpp::ok() && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

} 
