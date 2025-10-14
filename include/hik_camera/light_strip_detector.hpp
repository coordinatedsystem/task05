// include/hik_camera/light_strip_detector.hpp
#ifndef LIGHT_STRIP_DETECTOR_HPP
#define LIGHT_STRIP_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include "hik_camera/armor_classifier.hpp" 
#include "hik_camera/pose_estimator.hpp"

class LightStripDetector : public rclcpp::Node
{
public:
    LightStripDetector();

private:
    // 回调函数：接收原始图像
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    // 图像处理函数（可拆分）
    cv::Mat preprocess_image(const cv::Mat& input_image);

    // 灯条检测函数
    std::vector<cv::RotatedRect> detect_light_strips(const cv::Mat& processed_image);

    // 成员变量
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_processed_;  // 发布处理后图像
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_annotated_;  // 发布标注灯条的图像
    std::vector<cv::RotatedRect> combine_light_strips(const std::vector<cv::RotatedRect>& light_strip_boxes);
    // 参数（可后续通过ROS参数配置）
    int h_min_, h_max_; 
    int s_min_, s_max_;
    int v_min_, v_max_;

    int roi_width_;
    int roi_height_;

    std::shared_ptr<ArmorClassifier> classifier_;

    CameraIntrinsics loadCameraParamsFromMatlab();

     // 👉 新增：相机内参和畸变系数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 👉 新增：装甲板物理尺寸（单位：毫米）
    float armor_width_mm_ = 135.0f;
    float armor_height_mm_ = 56.0f;

    std::unique_ptr<PoseEstimator> pose_estimator_;

    bool pose_estimator_on = false;

};

#endif // LIGHT_STRIP_DETECTOR_HPP