// src/pose_estimator.cpp

#include "hik_camera/pose_estimator.hpp"
#include <opencv2/calib3d.hpp> // 包含 solvePnP

PoseEstimator::PoseEstimator(const CameraIntrinsics& intrinsics)
    : intrinsics_(intrinsics), width_mm_(0.0f), height_mm_(0.0f) 
{
    // 可以在这里验证 intrinsics_.camera_matrix 和 intrinsics_.dist_coeffs 是否有效
}

void PoseEstimator::setArmorSize(float width_mm, float height_mm) {
    if (width_mm <= 0 || height_mm <= 0) {
        // 可以抛出异常或记录警告
        return;
    }
    width_mm_ = width_mm;
    height_mm_ = height_mm;
}

bool PoseEstimator::estimate(
    const std::vector<cv::Point2f>& image_corners,
    cv::Point3f& center_in_camera
) {
    // 检查输入
    if (image_corners.size() != 4 || width_mm_ <= 0 || height_mm_ <= 0) {
        return false;
    }

    // 获取世界坐标系中的4个角点
    std::vector<cv::Point3f> world_points = getWorldPoints();

    // 存储旋转向量和平移向量
    cv::Mat rvec, tvec;

    // 使用 solvePnP 计算姿态
    bool success = cv::solvePnP(
        world_points,                // 3D 点
        image_corners,               // 对应的 2D 点
        intrinsics_.camera_matrix,   // 相机内参矩阵
        intrinsics_.dist_coeffs,     // 畸变系数
        rvec,                        // 输出：旋转向量
        tvec,                        // 输出：平移向量
        false,                       // 不使用初始猜测
        cv::SOLVEPNP_IPPE             // 使用 IPPE 方法（适合平面物体）
    );

    if (!success) {
        return false;
    }

    // 将平移向量赋值给输出
    center_in_camera.x = tvec.at<double>(0);
    center_in_camera.y = tvec.at<double>(1);
    center_in_camera.z = tvec.at<double>(2);

    return true;
}

std::vector<cv::Point3f> PoseEstimator::getWorldPoints() const {
    std::vector<cv::Point3f> points;
    // 假设装甲板左上角为世界坐标系原点
    points.emplace_back(0.0f, 0.0f, 0.0f);                     // 左上
    points.emplace_back(width_mm_, 0.0f, 0.0f);                // 右上
    points.emplace_back(width_mm_, height_mm_, 0.0f);           // 右下
    points.emplace_back(0.0f, height_mm_, 0.0f);                // 左下
    return points;
}