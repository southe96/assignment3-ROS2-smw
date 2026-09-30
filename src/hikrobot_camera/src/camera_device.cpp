#include "hikrobot_camera/camera_node.hpp"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "MvCameraControl.h"

namespace hikrobot_camera
{

namespace
{
// SDK 里的字符串是固定长度的 unsigned char 数组，末尾不一定有 '\0'
std::string toString(const unsigned char * text, size_t max_len)
{
  const char * p = reinterpret_cast<const char *>(text);
  return std::string(p, strnlen(p, max_len));
}

std::string serialOf(const MV_CC_DEVICE_INFO & dev)
{
  if (dev.nTLayerType == MV_GIGE_DEVICE) {
    const auto & info = dev.SpecialInfo.stGigEInfo;
    return toString(info.chSerialNumber, sizeof(info.chSerialNumber));
  }
  if (dev.nTLayerType == MV_USB_DEVICE) {
    const auto & info = dev.SpecialInfo.stUsb3VInfo;
    return toString(info.chSerialNumber, sizeof(info.chSerialNumber));
  }
  return "";
}

std::string modelOf(const MV_CC_DEVICE_INFO & dev)
{
  if (dev.nTLayerType == MV_GIGE_DEVICE) {
    const auto & info = dev.SpecialInfo.stGigEInfo;
    return toString(info.chModelName, sizeof(info.chModelName));
  }
  if (dev.nTLayerType == MV_USB_DEVICE) {
    const auto & info = dev.SpecialInfo.stUsb3VInfo;
    return toString(info.chModelName, sizeof(info.chModelName));
  }
  return "unknown";
}

// 网口相机的 IP 是一个 32 位整数，最高 8 位是第一段；USB 相机没有 IP，返回空字符串
std::string ipOf(const MV_CC_DEVICE_INFO & dev)
{
  if (dev.nTLayerType != MV_GIGE_DEVICE) {
    return "";
  }
  const unsigned int ip = dev.SpecialInfo.stGigEInfo.nCurrentIp;
  char text[32];
  std::snprintf(
    text, sizeof(text), "%u.%u.%u.%u",
    (ip >> 24) & 0xFF, (ip >> 16) & 0xFF, (ip >> 8) & 0xFF, ip & 0xFF);
  return text;
}

std::string describeDevice(const MV_CC_DEVICE_INFO & dev)
{
  std::string text = modelOf(dev) + " SN=" + serialOf(dev);
  if (dev.nTLayerType == MV_GIGE_DEVICE) {
    text += " IP=" + ipOf(dev);
  } else {
    text += " (USB)";
  }
  return text;
}
}  // namespace

std::string sdkErrorToString(int ret)
{
  const char * meaning = "其他错误，查 /opt/MVS/include/MvErrorDefine.h";
  switch (static_cast<unsigned int>(ret)) {
    case MV_E_HANDLE: meaning = "句柄无效"; break;
    case MV_E_SUPPORT: meaning = "相机不支持这个功能或参数"; break;
    case MV_E_CALLORDER: meaning = "函数调用顺序错误"; break;
    case MV_E_PARAMETER: meaning = "参数错误"; break;
    case MV_E_NODATA: meaning = "超时，没有收到图像"; break;
    case MV_E_GC_RANGE: meaning = "值超出范围"; break;
    case MV_E_GC_ACCESS: meaning = "当前不能修改（自动模式开着，或正在取流）"; break;
    case MV_E_ACCESS_DENIED: meaning = "无访问权限，相机被其他程序占用"; break;
    case MV_E_USB_READ: meaning = "USB 读取错误"; break;
    default: break;
  }
  char code[16];
  std::snprintf(code, sizeof(code), "0x%08X", static_cast<unsigned int>(ret));
  return std::string(code) + "（" + meaning + "）";
}

bool CameraNode::openCamera()
{
  // 1. 枚举网口和 USB 相机
  MV_CC_DEVICE_INFO_LIST list;
  std::memset(&list, 0, sizeof(list));
  int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &list);
  if (ret != MV_OK) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "枚举设备失败：%s", sdkErrorToString(ret).c_str());
    return false;
  }

  // 相机数量有变化时，把找到的相机都列出来
  const int count = static_cast<int>(list.nDeviceNum);
  if (count != last_device_count_) {
    last_device_count_ = count;
    RCLCPP_INFO(get_logger(), "找到 %d 台相机", count);
    for (int i = 0; i < count; ++i) {
      if (list.pDeviceInfo[i] != nullptr) {
        RCLCPP_INFO(get_logger(), "  [%d] %s", i, describeDevice(*list.pDeviceInfo[i]).c_str());
      }
    }
  }

  // 2. 按序列号 / IP 筛选，留空的条件不参与筛选
  std::vector<MV_CC_DEVICE_INFO *> matches;
  for (int i = 0; i < count; ++i) {
    MV_CC_DEVICE_INFO * dev = list.pDeviceInfo[i];
    if (dev == nullptr) {
      continue;
    }
    if (!serial_number_.empty() && serialOf(*dev) != serial_number_) {
      continue;
    }
    if (!ip_address_.empty() && ipOf(*dev) != ip_address_) {
      continue;
    }
    matches.push_back(dev);
  }

  const bool no_selector = serial_number_.empty() && ip_address_.empty();
  if (matches.empty()) {
    if (count == 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "没有找到任何相机：检查 USB 线 / 网线和供电");
    } else if (!serial_number_.empty() && !ip_address_.empty()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "标识冲突：没有一台相机同时满足 serial_number=%s 和 ip_address=%s",
        serial_number_.c_str(), ip_address_.c_str());
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "找不到指定的相机（serial_number='%s' ip_address='%s'），请对照上面列出的相机修改 camera.yaml",
        serial_number_.c_str(), ip_address_.c_str());
    }
    return false;
  }
  if (matches.size() > 1) {
    if (no_selector) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "找到 %zu 台相机，但 serial_number 和 ip_address 都没填，不知道打开哪一台",
        matches.size());
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "标识冲突：有 %zu 台相机同时符合 serial_number='%s' ip_address='%s'",
        matches.size(), serial_number_.c_str(), ip_address_.c_str());
    }
    return false;
  }

  MV_CC_DEVICE_INFO * dev = matches.front();
  const std::string name = describeDevice(*dev);
  if (no_selector) {
    RCLCPP_WARN_ONCE(
      get_logger(), "serial_number 和 ip_address 都没填，只有一台相机，直接打开它：%s",
      name.c_str());
  }

  // 3. 检查相机是否被其他程序占用
  if (!MV_CC_IsDeviceAccessible(dev, MV_ACCESS_Exclusive)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "相机 %s 被其他程序占用（比如 MVS 客户端），请关掉那个程序", name.c_str());
    return false;
  }

  // 4. 创建句柄、打开相机
  void * handle = nullptr;
  ret = MV_CC_CreateHandle(&handle, dev);
  if (ret != MV_OK) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "创建句柄失败：%s", sdkErrorToString(ret).c_str());
    return false;
  }
  ret = MV_CC_OpenDevice(handle, MV_ACCESS_Exclusive, 0);
  if (ret != MV_OK) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000, "打开相机 %s 失败：%s", name.c_str(),
      sdkErrorToString(ret).c_str());
    MV_CC_DestroyHandle(handle);
    return false;
  }

  // 5. 网口相机设置最佳包大小（官方 GrabImage 示例里的做法，USB 相机不需要）
  if (dev->nTLayerType == MV_GIGE_DEVICE) {
    const int packet_size = MV_CC_GetOptimalPacketSize(handle);
    if (packet_size > 0) {
      MV_CC_SetIntValueEx(handle, "GevSCPSPacketSize", packet_size);
    }
  }

  // 6. 关闭触发模式（0 = Off），相机连续出图
  ret = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "关闭触发模式失败：%s", sdkErrorToString(ret).c_str());
  }

  // 7. 阶段 8：注册异常回调，相机断线时 SDK 会调用 onException
  ret = MV_CC_RegisterExceptionCallBack(handle, &CameraNode::onException, this);
  if (ret != MV_OK) {
    RCLCPP_WARN(get_logger(), "注册异常回调失败：%s", sdkErrorToString(ret).c_str());
  }

  {
    std::lock_guard<std::mutex> lock(camera_mutex_);
    handle_ = handle;
    grabbing_ = false;
  }
  disconnected_ = false;
  RCLCPP_INFO(get_logger(), "已打开相机 %s", name.c_str());
  return true;
}

void CameraNode::closeCamera()
{
  std::lock_guard<std::mutex> lock(camera_mutex_);
  if (handle_ == nullptr) {
    return;
  }
  // 顺序和官方示例一样：停止取流 → 关闭设备 → 销毁句柄
  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
  RCLCPP_INFO(get_logger(), "相机已关闭（StopGrabbing → CloseDevice → DestroyHandle）");
}

void CameraNode::onException(unsigned int msg_type, void * user)
{
  // 这个函数在 SDK 自己的线程里运行，这里只做标记，真正的重连交给取图线程
  auto * self = static_cast<CameraNode *>(user);
  if (msg_type == MV_EXCEPTION_DEV_DISCONNECT) {
    self->disconnected_ = true;
  }
}

}  // namespace hikrobot_camera
