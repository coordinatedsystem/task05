#pragma once

#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <chrono>
#include <opencv2/opencv.hpp>

// ROS 2 相关头文件
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

// 海康 SDK 头文件（请根据实际路径调整）
#include "/home/coordsys/zbx/MVSROS_project/MVS_SDK/SDK/include/MvCameraControl.h"
using namespace std::chrono_literals;

/**
 * @brief 海康相机驱动类
 * 提供初始化、采集、重连、参数设置等功能
 */
class HikCameraDriver {
public:
    /**
     * @brief 构造函数
     * @param node ROS 2 节点共享指针
     */
    explicit HikCameraDriver(std::shared_ptr<rclcpp::Node> node);

    /**
     * @brief 析构函数
     * 自动清理资源
     */
    ~HikCameraDriver();

    /**
     * @brief 初始化相机（打开设备、设置参数）
     * @return 成功返回 true
     */
    bool initialize();

    /**
     * @brief 开始图像采集
     * @return 成功返回 true
     */
    bool startCapture();

    /**
     * @brief 停止图像采集（不关闭设备）
     */
    void stopCapture();

    /**
     * @brief 关闭设备（释放连接，但不销毁句柄）
     */
    void closeDevice();

    /**
     * @brief 打印当前相机信息（如宽度、帧率、像素格式等）
     */
    void printCameraInfo();

    //void checkAndUpdateFps();

    bool isDeviceConnected();

    bool initializeVideoSource();
    void publishFrameAsImageMsg(const cv::Mat& frame);

private:
    // ========================================
    // 回调函数（Callback Functions）
    // ========================================

    /**
     * @brief SDK 图像数据回调函数（静态，供海康 SDK 调用）
     * @param pData 图像数据指针
     * @param pFrameInfo 帧信息结构体
     * @param pUser 用户数据指针（指向当前对象）
     */
    static void __stdcall onImageCallback(
        unsigned char* pData,
        MV_FRAME_OUT_INFO_EX* pFrameInfo,
        void* pUser
    );

    /**
     * @brief ROS 参数设置回调函数
     * 处理 exposure_time, gain, frame_rate, pixel_format 等参数变更
     * @param parameters 待设置的参数列表
     * @return 参数设置结果
     */
    rcl_interfaces::msg::SetParametersResult onParameterChange(
        const std::vector<rclcpp::Parameter>& parameters
    );

    /**
     * @brief 定时器回调：定期检查相机连接状态，自动重连
     */
    void reconnectTimerCallback();

    /**
     * @brief 辅助函数：递增重试次数，若达到上限则退出节点
     */
    void incrementAndCheckRetry();

    // ========================================
    // 成员变量（Member Variables）
    // ========================================

    // ROS 相关
    std::shared_ptr<rclcpp::Node> node_;                        ///< 持有节点共享指针
    image_transport::Publisher image_pub_;                      ///< 图像发布者
    rclcpp::TimerBase::SharedPtr reconnect_timer_;              ///< 重连状态检查定时器

    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 相机连接与状态
    std::atomic<bool> is_connected_{false};   ///< 设备是否已成功打开
    std::atomic<bool> is_capturing_{false};   ///< 是否正在采集图像

    // 重试机制
    int retry_count_ = 0;                     ///< 当前连续重试次数
    const int max_retries_ = 10;              ///< 最大重试次数（超过则退出）

    // 海康 SDK 相关
    void* camera_handle_ = nullptr;           ///< 相机设备句柄
    MV_CC_DEVICE_INFO device_info_{};         ///< 设备信息（用于重建连接）
    std::string camera_ip_;                   ///< 配置的目标相机 IP 地址
    std::string camera_sn_;                   ///< 配置的目标相机序列号

    // FPS 相关
    double last_set_fps_ = -1.0;           // 上次设置的帧率
    double last_actual_fps_ = -1.0;        // 上次实际帧率
    double current_resulting_fps_ = 0.0;   // 当前实际帧率（可用于 diagnostics）

    std::mutex fps_mutex_;                 // 如果多线程访问，加锁

    // 自动重连机制
    rclcpp::Time last_reconnect_time_;        ///< 上次尝试重连的时间戳
    std::chrono::milliseconds min_reconnect_interval_; ///< 最小重连间隔（防止频繁重连）

    rclcpp::TimerBase::SharedPtr fps_timer_;

    std::atomic<uint64_t> last_timestamp_ns_{0};     // 上一帧的硬件时间戳（ns）
    std::atomic<int> frame_count_{0};                // 当前累计帧数
    std::atomic<int64_t> last_print_time_ns_{0};     // 上次打印日志的时间（用硬件时间）
    static constexpr int64_t LOG_INTERVAL_NS = 1'000'000'000; // 每1秒打印一次

    bool use_video_file_ = false;
    std::string video_file_path_;
    cv::VideoCapture video_capture_;
    std::atomic<bool> video_capture_thread_running_{false};
    std::thread video_thread_;

    image_transport::ImageTransport image_transport_;
    
    void videoPlaybackThread();
};