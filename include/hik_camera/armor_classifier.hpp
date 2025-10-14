// armor_classifier.hpp
#ifndef HIK_CAMERA_ARMOR_CLASSIFIER_HPP_
#define HIK_CAMERA_ARMOR_CLASSIFIER_HPP_

#include <torch/script.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

class ArmorClassifier {
public:
    // 构造函数
    explicit ArmorClassifier(const std::string& model_path);

    // 分类函数
    int classify(const cv::Mat& roi);

    std::string getClassName(int idx);

private:
    std::shared_ptr<torch::jit::script::Module> module_;

    std::vector<std::string> class_names_ = {
        "1", "2", "3", "4", "5", "6", "7", "8", "negative"
    };
};

#endif // HIK_CAMERA_ARMOR_CLASSIFIER_HPP_