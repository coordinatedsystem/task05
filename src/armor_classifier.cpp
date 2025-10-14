// armor_classifier.cpp
#include "hik_camera/armor_classifier.hpp"
#include <iostream>

ArmorClassifier::ArmorClassifier(const std::string& model_path) {
    try {
        module_ = std::make_shared<torch::jit::script::Module>(torch::jit::load(model_path));
        module_->eval();
        std::cout << "模型加载成功: " << model_path << std::endl;
    } catch (const c10::Error& e) {
        std::cerr << "模型加载失败: " << e.msg() << std::endl;
        exit(-1);
    }
}

int ArmorClassifier::classify(const cv::Mat& roi) {
    cv::Mat gray, resized, binary;

    if (roi.channels() == 3) {
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = roi.clone();
    }

    cv::resize(gray, resized, cv::Size(20, 28), 0, 0, cv::INTER_NEAREST); 

    cv::threshold(resized, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    binary.convertTo(binary, CV_32F, 1.0 / 255.0);  // 0.0 或 1.0

    auto tensor = torch::from_blob(binary.data, {1, 1, 28, 20}, torch::kFloat32);

    try {
        at::Tensor output = module_->forward({tensor}).toTensor();
        return output.argmax(1).item<int>();
    } catch (const c10::Error& e) {
        std::cerr << "Torch 推理错误: " << e.msg() << std::endl;
        return -1;  // 返回无效类别
    }
}

std::string ArmorClassifier::getClassName(int idx) {
    // 避免 signed/unsigned 比较警告
    if (idx >= 0 && static_cast<size_t>(idx) < class_names_.size()) {
        return class_names_[idx];
    }
    return "unknown";
}