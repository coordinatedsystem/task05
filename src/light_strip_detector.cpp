// src/light_strip_detector.cpp
#include "hik_camera/light_strip_detector.hpp"
#include <iomanip>
#include "hik_camera/armor_classifier.hpp"
#include "hik_camera/pose_estimator.hpp"

LightStripDetector::LightStripDetector()
    : Node("light_strip_detector"),
      h_min_(0), h_max_(30),
      s_min_(100), s_max_(255),
      v_min_(100), v_max_(255)
{
    RCLCPP_INFO(this->get_logger(), "Light Strip Detector node started.");

    // 创建订阅者，订阅 /image_raw
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10,
        std::bind(&LightStripDetector::image_callback, this, std::placeholders::_1));
    
    // 声明 ROI 参数（可通过命令行传入）
    this->declare_parameter("roi_width", 200);
    this->declare_parameter("roi_height", 400);

    // 获取参数
    roi_width_ = this->get_parameter("roi_width").as_int();
    roi_height_ = this->get_parameter("roi_height").as_int();    

    // 创建发布者，发布处理后的图像（用于调试）
    publisher_processed_ = this->create_publisher<sensor_msgs::msg::Image>("/image_processed", 10);
    
    // 创建发布者，发布标注了灯条的图像
    publisher_annotated_ = this->create_publisher<sensor_msgs::msg::Image>("/image_with_light_strips", 10);

    try {
        classifier_ = std::make_shared<ArmorClassifier>("/home/coordsys/zbx/MVSROS_project/MVS_ROS2/hik_camera/number_recognition_model/models/best_armor_model.pt");
    } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load armor classifier.");
        rclcpp::shutdown();
    }

     try {
        auto intrinsics = loadCameraParamsFromMatlab();
        pose_estimator_ = std::make_unique<PoseEstimator>(intrinsics);
        pose_estimator_->setArmorSize(135.0f, 56.0f); // 设定装甲板尺寸为毫米
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize PoseEstimator: %s", e.what());
        rclcpp::shutdown();
    }
}

void LightStripDetector::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // 根据输入图像的编码选择正确的转换方式
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV bridge error: %s", e.what());
        return;
    }

    cv::Mat& original_image = cv_ptr->image;

    // Step 1: 图像预处理
    cv::Mat processed_image = preprocess_image(original_image);

    // Step 2: 发布处理后的图像（便于调试查看效果）
    auto processed_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", processed_image).toImageMsg();
    publisher_processed_->publish(*processed_msg);

    // Step 3: 检测所有候选灯条
    std::vector<cv::RotatedRect> light_strip_boxes = detect_light_strips(processed_image);

    // Step 4: 联合判断，组合成装甲板区域
    std::vector<cv::RotatedRect> armor_boxes = combine_light_strips(light_strip_boxes);  // 新增：使用combine_light_strips函数

    // Step 5: 在原图上分别绘制灯条和装甲板区域
    cv::Mat annotated_image = original_image.clone(); // 使用原图标注更直观

    // 绘制单个灯条
    for (const auto& rrect : light_strip_boxes) {
        cv::Point2f points[4];
        rrect.points(points);  // 获取四个顶点
        for (int j = 0; j < 4; ++j) {
            cv::line(annotated_image, points[j], points[(j+1)%4], cv::Scalar(255, 0, 0), 2); 
        }

        // 可选：绘制中心点
        //cv::circle(annotated_image, rrect.center, 3, cv::Scalar(0, 0, 255), -1); // 红色圆点


        // 可选：显示角度
        // std::stringstream ss;
        // ss << "Angle: " << rrect.angle << "°";
        // cv::putText(annotated_image, ss.str(), rrect.center + cv::Point2f(10, 10),
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);
    }
    float armor_height;
    float armor_width;
    //绘制装甲板区域
    for (const auto& rrect : armor_boxes) { // 使用不同的颜色来区分
        // cv::Point2f points[4];
        // rrect.points(points);  // 获取四个顶点
        // for (int j = 0; j < 4; ++j) {
        //     cv::line(annotated_image, points[j], points[(j+1)%4], cv::Scalar(0, 255, 0), 2);  // 蓝色线表示装甲板区域
        // }

        // 可选：绘制中心点
        //cv::circle(annotated_image, rrect.center, 3, cv::Scalar(0, 0, 255), -1); // 同样用红色圆点标记中心

        // 可选：显示角度
        // std::stringstream ss;
        // ss << "Armor Angle: " << rrect.angle << "°"; // 标记装甲板的角度
        // cv::putText(annotated_image, ss.str(), rrect.center + cv::Point2f(10, -10), 
        //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0), 1);   
        armor_height = rrect.size.width;  // 装甲板高度
        armor_width = rrect.size.height;   // 装甲板宽度
    }

    // for (const auto& armor_rect : armor_boxes) {
    //     cv::Point2f image_points[4];
    //     armor_rect.points(image_points);
    //     std::vector<cv::Point2f> corners = {image_points[0], image_points[1], image_points[2], image_points[3]};

    //     cv::Point3f center_3d;
    //     bool success = pose_estimator_->estimate(corners, center_3d);

    //     // if (success) {
    //     //     RCLCPP_INFO(this->get_logger(),
    //     //         "Armor center in camera frame: X=%.1f mm, Y=%.1f mm, Z=%.1f mm",
    //     //         center_3d.x, center_3d.y, center_3d.z);
            
    //     //     // 这里可以根据需要发布中心点位置信息
    //     // }
    //}

    cv::Point2f roi_center;
    bool has_valid_armor = !armor_boxes.empty();
       
    float roi_height = armor_height * 2.0f; 
    float roi_width = roi_height * 0.7f;
    if (has_valid_armor) {
        // 使用第一个装甲板的中心作为 ROI 中心
        roi_center = armor_boxes[0].center;

        // 创建一个固定大小的轴对齐矩形（非旋转）
        cv::Rect2f roi_rect(
            roi_center.x - roi_width/ 2.0f,
            roi_center.y - roi_height/ 2.0f,
            roi_width,
            roi_height
        );

        // 边界检查，防止 ROI 超出图像范围
        roi_rect = roi_rect & cv::Rect2f(0, 0, annotated_image.cols, annotated_image.rows);

        // 绘制 ROI 矩形（红色实线）
        cv::rectangle(annotated_image, roi_rect, cv::Scalar(0, 0, 255), 2);

        RCLCPP_DEBUG(this->get_logger(), "Drawn ROI at (%.1f, %.1f) size %dx%d",
                    roi_center.x, roi_center.y, roi_width, roi_height);

        // 4. 数字识别
        cv::Mat roi = annotated_image(roi_rect);
        int cls = classifier_->classify(roi);
        std::string number = classifier_->getClassName(cls);

        // 5. 绘制结果
        cv::rectangle(annotated_image, roi_rect, cv::Scalar(0, 255, 0), 2);
        cv::putText(annotated_image, number, roi_center + cv::Point2f(0, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "No armor plate detected, skipping ROI.");
    }

    // Step 6: 发布标注后的图像
    auto annotated_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", annotated_image).toImageMsg();
    publisher_annotated_->publish(*annotated_msg);

    RCLCPP_DEBUG(this->get_logger(), "Detected %zu light strips and %zu armor plates.", light_strip_boxes.size(), armor_boxes.size());
}

cv::Mat LightStripDetector::preprocess_image(const cv::Mat& input_image)
{
    cv::Mat gray_image;
    // 1. 转换为灰度图
    if (input_image.channels() == 3)
    {
        cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray_image = input_image.clone();
    }

    cv::Mat blurred_image;
    // 2. 高斯滤波去噪
    cv::blur(gray_image, blurred_image, cv::Size(5, 5));

    cv::Mat binary_image;
    // 3. 二值化
    // 方式一：固定阈值（例如 127）
    cv::threshold(blurred_image, binary_image, 160, 255, cv::THRESH_BINARY);

    //方式二：Otsu 自动阈值（推荐，适合光照变化场景）
    //cv::threshold(blurred_image, binary_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat processed_image;

    // 4. 形态学处理
    cv::morphologyEx(binary_image, processed_image, cv::MORPH_OPEN, kernel);  // 开运算：去噪点
    cv::morphologyEx(processed_image, processed_image, cv::MORPH_CLOSE, kernel); // 关运算：闭合间隙

    return processed_image;
}

std::vector<cv::RotatedRect> LightStripDetector::detect_light_strips(const cv::Mat& processed_image)
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // 提取外部轮廓
    cv::findContours(processed_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<cv::RotatedRect> valid_rotated_boxes;

    for (size_t i = 0; i < contours.size(); ++i) {
        double contour_area = cv::contourArea(contours[i]);
        
        // 过滤太小的区域
        if (contour_area < 100) {
            continue;
        }

        // 获取最小外接旋转矩形
        cv::RotatedRect rotated_rect = cv::minAreaRect(contours[i]);
        cv::Size2f rect_size = rotated_rect.size;
        float width = rect_size.width;
        float height = rect_size.height;

        // 确保 width >= height（统一长边为 width）
        if (width < height) {
            std::swap(width, height);
        }

        double aspect_ratio = width / height;
        double rect_area = width * height;
        double fill_ratio = contour_area / rect_area;

        // 综合判断条件（可根据实际调整）
        if (
            aspect_ratio > 4.0 &&     // 细长条（LED 灯条）
            aspect_ratio < 10.0 &&
            fill_ratio > 0.5 &&      // 实心程度高
            fill_ratio <= 1.0 &&
            contour_area > 100       // 面积不能太小
        ) {
            valid_rotated_boxes.push_back(rotated_rect);  // 直接保存旋转矩形

            RCLCPP_DEBUG(this->get_logger(), 
                "Valid light strip %zu: area=%.1f, aspect_ratio=%.2f, fill_ratio=%.2f, angle=%.1f°", 
                i, contour_area, aspect_ratio, fill_ratio, rotated_rect.angle);
        }
        else {
            RCLCPP_DEBUG(this->get_logger(), 
                "Filtered contour %zu: area=%.1f, aspect_ratio=%.2f, fill_ratio=%.2f", 
                i, contour_area, aspect_ratio, fill_ratio);
        }
    }

    return valid_rotated_boxes;
}

std::vector<cv::RotatedRect> LightStripDetector::combine_light_strips(const std::vector<cv::RotatedRect>& light_strip_boxes)
{
    std::vector<cv::RotatedRect> matched_boxes;

    for (size_t i = 0; i < light_strip_boxes.size(); ++i) {
        for (size_t j = i + 1; j < light_strip_boxes.size(); ++j) {
            const auto& rect1 = light_strip_boxes[i];
            const auto& rect2 = light_strip_boxes[j];

            // 面积差异不超过 20%
            double area1 = rect1.size.width * rect1.size.height;
            double area2 = rect2.size.width * rect2.size.height;
            double area_ratio = std::min(area1, area2) / std::max(area1, area2);
            if (area_ratio < 0.5) {  // 即差异 > 20%
                continue;
            }

            // 角度差异不超过 10 度
            double angle_diff = fabs(rect1.angle - rect2.angle);
            if (angle_diff > 7.0) {
                continue;
            }

            // 合并两个灯条的顶点，计算最小外接矩形
            cv::Point2f points1[4], points2[4];
            rect1.points(points1);  // 获取四个顶点
            rect2.points(points2);

            std::vector<cv::Point2f> all_points;
            all_points.insert(all_points.end(), points1, points1 + 4);
            all_points.insert(all_points.end(), points2, points2 + 4);

            // 计算包围所有点的最小旋转矩形
            cv::RotatedRect combined_rect = cv::minAreaRect(all_points);

            // 可选：进一步过滤（例如宽高比、面积等）
            float width = std::max(combined_rect.size.width, combined_rect.size.height);
            float height = std::min(combined_rect.size.width, combined_rect.size.height);
            if (width > 0 && height > 0) {
                double aspect_ratio = width / height;
                if (aspect_ratio < 1.0 || aspect_ratio > 5.0) {  // 装甲板通常不会太细长
                    continue;
                }
            }

            matched_boxes.push_back(combined_rect);

            RCLCPP_DEBUG(this->get_logger(), 
                "Matched armor plate: angle_diff=%.1f°, area_ratio=%.2f", 
                angle_diff, area_ratio);
        }
    }

    return matched_boxes;
}

CameraIntrinsics LightStripDetector::loadCameraParamsFromMatlab() {
    CameraIntrinsics params;
    
    // 硬编码的相机内参和畸变系数
    params.camera_matrix = (cv::Mat_<double>(3, 3) << 
        5.1992, 0, 0,
        0, 5.2776, 0,
        0.4597, 0.4994, 0.0010);
    
    params.dist_coeffs = (cv::Mat_<double>(4, 1) << 
        0.9181, -50.1263, 0, 0); // k1, k2, p1, p2, k3
    
    return params;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LightStripDetector>());
    rclcpp::shutdown();
    return 0;
}