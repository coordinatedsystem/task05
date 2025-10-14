#include <rclcpp/rclcpp.hpp>
#include "hik_camera/hik_camera_driver.hpp"
#include <opencv2/opencv.hpp>

int main(int argc, char* argv[]) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);

    // 创建节点
    auto node = std::make_shared<rclcpp::Node>("hik_camera");

    try {
        // 构造 HikCameraDriver 即完成所有初始化工作
        // 所有逻辑（是否用文件、是否连相机、是否启动采集）都在 driver 内部处理
        HikCameraDriver driver(node);

        RCLCPP_INFO(node->get_logger(), "Hikvision camera driver initialized. Entering spin...");

        // 进入事件循环，处理回调（图像发布、参数变化、重连定时器等）
        rclcpp::spin(node);

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("HikCameraDriver"), "Exception in HikCameraDriver: %s", e.what());
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        RCLCPP_FATAL(rclcpp::get_logger("HikCameraDriver"), "Unknown exception in HikCameraDriver");
        rclcpp::shutdown();
        return 1;
    }

    // 正常关闭
    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("HikCameraDriver"), "ROS shutdown completed.");
    return 0;
}