// include/hik_camera/pose_estimator.hpp
#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <memory>

struct CameraIntrinsics {
    cv::Mat camera_matrix;     // 3x3
    cv::Mat dist_coeffs;       // 5x1 or 4x1
};

class PoseEstimator {
public:
    // 构造函数：传入相机内参
    PoseEstimator(const CameraIntrinsics& intrinsics);

    // 设置装甲板尺寸（单位：毫米）
    void setArmorSize(float width_mm, float height_mm);

    // 计算单个装甲板的 3D 中心位置（在相机坐标系下）
    bool estimate(
        const std::vector<cv::Point2f>& image_corners,  // 检测到的4个角点像素坐标
        cv::Point3f& center_in_camera                   // 输出：中心在相机坐标系中的 (X,Y,Z)
    );

private:
    CameraIntrinsics intrinsics_;
    float width_mm_, height_mm_;

    // 内部：生成世界坐标（以左上角为原点，向右向下为正）
    std::vector<cv::Point3f> getWorldPoints() const;

};