#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <iostream>
#include <onnxruntime_cxx_api.h>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <chrono>

// YOLO11 检测结果结构
struct Detection {
  cv::Rect box;
  float confidence;
  int classId;
};

class YOLO11Detector {
public:
  YOLO11Detector(const std::string &modelPath, float confThreshold = 0.35f,
                 float iouThreshold = 0.45f)
      : confThreshold_(confThreshold), iouThreshold_(iouThreshold) {

    // 初始化 ONNX Runtime
    env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "YOLO11");
    Ort::SessionOptions sessionOptions;
    sessionOptions.SetIntraOpNumThreads(4);
    sessionOptions.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);

    session_ = Ort::Session(env_, modelPath.c_str(), sessionOptions);

    // 获取输入输出信息
    Ort::AllocatorWithDefaultOptions allocator;

    // 输入信息
    auto inputName = session_.GetInputNameAllocated(0, allocator);
    inputName_ = inputName.get();
    auto inputShape =
        session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    inputHeight_ = inputShape[2];
    inputWidth_ = inputShape[3];

    // 输出信息
    auto outputName = session_.GetOutputNameAllocated(0, allocator);
    outputName_ = outputName.get();

    std::cout << "load success!" << std::endl;
    std::cout << "input size: " << inputWidth_ << "x" << inputHeight_
              << std::endl;
  }

  std::vector<Detection> detect(const cv::Mat &image) {
    cv::Mat blob;
    preprocess(image, blob);

    // 准备输入张量
    std::vector<int64_t> inputShape = {1, 3, inputHeight_, inputWidth_};
    size_t inputTensorSize = 1 * 3 * inputHeight_ * inputWidth_;

    std::vector<float> inputTensorValues(inputTensorSize);
    std::memcpy(inputTensorValues.data(), blob.data,
                inputTensorSize * sizeof(float));

    auto memoryInfo =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
        memoryInfo, inputTensorValues.data(), inputTensorSize,
        inputShape.data(), inputShape.size());

    // 运行推理
    const char *inputNames[] = {inputName_.c_str()};
    const char *outputNames[] = {outputName_.c_str()};

    auto outputTensors = session_.Run(Ort::RunOptions{nullptr}, inputNames,
                                      &inputTensor, 1, outputNames, 1);

    // 获取输出
    float *outputData = outputTensors[0].GetTensorMutableData<float>();
    auto outputShape = outputTensors[0].GetTensorTypeAndShapeInfo().GetShape();

    // 后处理
    return postprocess(outputData, outputShape, image.cols, image.rows);
  }

private:
  Ort::Env env_{nullptr};
  Ort::Session session_{nullptr};
  std::string inputName_;
  std::string outputName_;
  int64_t inputWidth_ = 960;
  int64_t inputHeight_ = 960;
  float confThreshold_;
  float iouThreshold_;

  void preprocess(const cv::Mat &image, cv::Mat &blob) {
    cv::dnn::blobFromImage(
      image, blob, 1.0 / 255.0,
      cv::Size(inputWidth_, inputHeight_),
      cv::Scalar(), true, false, CV_32F
    );
  }

  std::vector<Detection> postprocess(float *output,
                                     const std::vector<int64_t> &shape,
                                     int origWidth, int origHeight) {
    std::vector<Detection> detections;
    std::vector<cv::Rect> boxes;
    std::vector<float> confidences;
    std::vector<int> classIds;

    // YOLO11 输出格式: [1, 84, 8400] -> [1, 4+num_classes, num_predictions]
    // 需要转置为 [8400, 84]
    int numClasses = shape[1] - 4; // 通常是80个类别
    int numPredictions = shape[2];

    float xFactor = (float)origWidth / inputWidth_;
    float yFactor = (float)origHeight / inputHeight_;

    for (int i = 0; i < numPredictions; i++) {
      // 获取边界框坐标 (cx, cy, w, h)
      float cx = output[0 * numPredictions + i];
      float cy = output[1 * numPredictions + i];
      float w = output[2 * numPredictions + i];
      float h = output[3 * numPredictions + i];

      // 找到最大类别置信度
      float maxClassConf = 0;
      int maxClassId = 0;
      for (int c = 0; c < numClasses; c++) {
        float classConf = output[(4 + c) * numPredictions + i];
        if (classConf > maxClassConf) {
          maxClassConf = classConf;
          maxClassId = c;
        }
      }

      if (maxClassConf >= confThreshold_) {
        // 转换为 xyxy 格式并缩放到原始图像尺寸
        int x1 = static_cast<int>((cx - w / 2) * xFactor);
        int y1 = static_cast<int>((cy - h / 2) * yFactor);
        int x2 = static_cast<int>((cx + w / 2) * xFactor);
        int y2 = static_cast<int>((cy + h / 2) * yFactor);

        boxes.push_back(cv::Rect(x1, y1, x2 - x1, y2 - y1));
        confidences.push_back(maxClassConf);
        classIds.push_back(maxClassId);
      }
    }

    // NMS
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confThreshold_, iouThreshold_,
                      indices);

    for (int idx : indices) {
      Detection det;
      det.box = boxes[idx];
      det.confidence = confidences[idx];
      det.classId = classIds[idx];
      detections.push_back(det);
    }

    return detections;
  }
};

// 在图像上绘制检测结果
void drawDetections(cv::Mat &image, const std::vector<Detection> &detections) {
  // 锥桶类别名称 (3类)
  static const std::vector<std::string> classNames = {
      "red_cone", "blue_cone", "yellow_cone"};
  // 对应颜色 (BGR格式)
  static const std::vector<cv::Scalar> classColors = {
      cv::Scalar(0, 0, 255),  // 黄色
      cv::Scalar(255, 0, 0),  // 蓝色
      cv::Scalar(0, 165, 255) // 橙色
  };

  for (const auto &det : detections) {
    // 根据类别选择颜色
    cv::Scalar color = (static_cast<size_t>(det.classId) < classColors.size())
                           ? classColors[det.classId]
                           : cv::Scalar(0, 255, 0);

    // 绘制边界框
    cv::rectangle(image, det.box, color, 2);

    // 准备标签文本
    std::string label;
    if (static_cast<size_t>(det.classId) < classNames.size()) {
      label = classNames[det.classId];
    } else {
      label = "class" + std::to_string(det.classId);
    }
    label +=
        ": " + std::to_string(static_cast<int>(det.confidence * 100)) + "%";

    // 绘制标签背景
    int baseLine;
    cv::Size labelSize =
        cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    int top = std::max(det.box.y, labelSize.height);
    cv::rectangle(image, cv::Point(det.box.x, top - labelSize.height - 5),
                  cv::Point(det.box.x + labelSize.width, top), color,
                  cv::FILLED);

    // 绘制标签文本
    cv::putText(image, label, cv::Point(det.box.x, top - 3),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
  }
}

class ImageSubscriberComponent : public rclcpp::Node{
public:
  explicit ImageSubscriberComponent(const rclcpp::NodeOptions & options) : Node("image_subscriber_component", options){
    image_sub_ = image_transport::create_subscription(
      this,
      "/virtual_cam/image_raw",
      std::bind(&ImageSubscriberComponent::image_callback, this, std::placeholders::_1),
      "raw",
      rmw_qos_profile_sensor_data
    );
  }
private:
  std::string modelPath = "/home/as/as_training-2025-main/weight/best.onnx";
  YOLO11Detector detector = YOLO11Detector(modelPath);
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg){
    auto start = std::chrono::high_resolution_clock::now();
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    cv::Mat frame = cv_ptr -> image;
    auto start1 = std::chrono::high_resolution_clock::now();
    auto detections = detector.detect(frame);
    auto end1 = std::chrono::high_resolution_clock::now();
    drawDetections(frame, detections);
    cv::imshow("Subscribed Image", frame);
    cv::waitKey(1);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1-start1);
    RCLCPP_INFO(this->get_logger(), "总用时:%ldms, 推理用时:%ldms", duration.count(), duration1.count());
  }
  image_transport::Subscriber image_sub_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(ImageSubscriberComponent)

// int main(int argc, char *argv[]) {
//   // 默认路径，可以通过命令行参数修改
//   std::string modelPath = "/home/as/hw/as_training-2025/weight/best.onnx";
//   std::string imagePath = "/home/as/hw/as_training-2025/weight/image.png";
//   std::string outputPath = "result.png";

//     try {
//         // 加载模型
//         YOLO11Detector detector(modelPath);

//         // 读取图像
//         cv::Mat image = cv::imread(imagePath);
//         if (image.empty()) {
//             std::cerr << "无法读取图像: " << imagePath << std::endl;
//             return -1;
//         }
//         std::cout << "图像尺寸: " << image.cols << "x" << image.rows << std::endl;

//         // 运行检测
//         auto detections = detector.detect(image);

//         // 打印检测结果
//         std::cout << "\n检测到 " << detections.size() << " 个目标:" << std::endl;
//         for (const auto& det : detections) {
//             std::cout << "  类别: " << det.classId
//                       << ", 置信度: " << det.confidence
//                       << ", 边界框: [" << det.box.x << ", " << det.box.y
//                       << ", " << det.box.x + det.box.width << ", " << det.box.y + det.box.height << "]"
//                       << std::endl;
//         }

//         // 在图像上绘制检测框
//         drawDetections(image, detections);

//         // 保存结果
//         cv::imwrite(outputPath, image);
//         std::cout << "\n结果已保存到: " << outputPath << std::endl;

//         // 显示结果 (如果有显示器)
//         cv::imshow("YOLO11 Detection", image);
//         cv::waitKey(0);

//     } catch (const std::exception& e) {
//         std::cerr << "错误: " << e.what() << std::endl;
//         return -1;
//     }

//     return 0;
// }
