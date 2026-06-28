#include <ros/ros.h>
#include <px4_controller/tbag.h>
#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>

// 正确实现 TensorRT 日志记录器
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            ROS_WARN_STREAM("[TensorRT] " << msg);
    }
} gLogger;

// 模型参数
const int INPUT_H = 640;
const int INPUT_W = 640;
const int NUM_ANCHORS = 8400;
const int NUM_CLASSES = 80;
const float CONF_THRESH = 0.4f;
const float IOU_THRESH = 0.45f;

// 窗口显示大小
//int new_h = 540;
//int new_w = static_cast<int>(1920.0f / 1200.0f * new_h); 
int new_h = 540;
int new_w = 864;

struct Detection {
    float x1, y1, x2, y2, conf;
};

// 异步取帧（安全类型处理）
cv::Mat latest_frame;
std::mutex frame_mutex;
bool stop_flag = false;

void capture_thread(cv::VideoCapture& cap) {
    cv::Mat tmp;
    while (!stop_flag) {
        try {
            cap >> tmp;
            if (tmp.empty()) continue;
            if (tmp.type() == CV_8UC3) {
                std::lock_guard<std::mutex> lock(frame_mutex);
                tmp.copyTo(latest_frame);
            } else {
                ROS_WARN_THROTTLE(5, "Unexpected frame type %d, skipping", tmp.type());
            }
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Capture error: " << e.what());
        }
    }
}

// 自定义 NMS
void apply_nms(std::vector<Detection>& dets, float iou_thres) {
    if (dets.empty()) return;
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) { return a.conf > b.conf; });
    std::vector<Detection> result;
    while (!dets.empty()) {
        result.push_back(dets[0]);
        std::vector<Detection> remaining;
        for (size_t i = 1; i < dets.size(); ++i) {
            float xx1 = std::max(dets[0].x1, dets[i].x1);
            float yy1 = std::max(dets[0].y1, dets[i].y1);
            float xx2 = std::min(dets[0].x2, dets[i].x2);
            float yy2 = std::min(dets[0].y2, dets[i].y2);
            float w = std::max(0.0f, xx2 - xx1);
            float h = std::max(0.0f, yy2 - yy1);
            float inter = w * h;
            float area1 = (dets[0].x2 - dets[0].x1) * (dets[0].y2 - dets[0].y1);
            float area2 = (dets[i].x2 - dets[i].x1) * (dets[i].y2 - dets[i].y1);
            float iou = inter / (area1 + area2 - inter);
            if (iou <= iou_thres) remaining.push_back(dets[i]);
        }
        dets = remaining;
    }
    dets = result;
}

// YOLOv8 输出解码 + NMS
std::vector<Detection> parse_yolov8(float* output, int img_width, int img_height) {
    std::vector<Detection> raw_dets;
    //float scale_x = 1200.0f / INPUT_W;
    //float scale_y = 1200.0f / INPUT_H;

    float scale_x = static_cast<float>(img_width) / INPUT_W;
    float scale_y = static_cast<float>(img_height) / INPUT_H;

    for (int i = 0; i < NUM_ANCHORS; ++i) {
        float x_center = output[0 * NUM_ANCHORS + i];
        float y_center = output[1 * NUM_ANCHORS + i];
        float width    = output[2 * NUM_ANCHORS + i];
        float height   = output[3 * NUM_ANCHORS + i];

        float x1 = x_center - width / 2.0f;
        float y1 = y_center - height / 2.0f;
        float x2 = x_center + width / 2.0f;
        float y2 = y_center + height / 2.0f;

        float max_conf = 0.0f;
        for (int c = 0; c < NUM_CLASSES; ++c) {
            float score = output[(4 + c) * NUM_ANCHORS + i];
            if (score > max_conf) max_conf = score;
        }

        if (max_conf >= CONF_THRESH) {
            Detection d;
            d.x1 = std::max(0.0f, std::min(x1 * scale_x, static_cast<float>(img_width - 1)));
            d.y1 = std::max(0.0f, std::min(y1 * scale_y, static_cast<float>(img_height - 1)));
            d.x2 = std::max(0.0f, std::min(x2 * scale_x, static_cast<float>(img_width - 1)));
            d.y2 = std::max(0.0f, std::min(y2 * scale_y, static_cast<float>(img_height - 1)));
            d.conf = max_conf;
            if (d.x2 > d.x1 && d.y2 > d.y1) raw_dets.push_back(d);
        }
    }

    apply_nms(raw_dets, IOU_THRESH);
    if (raw_dets.size() > 3) raw_dets.resize(3);
    return raw_dets;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<px4_controller::tbag>("IR", 10);

    // ---------- 1. 加载 TensorRT 引擎 ----------
    std::string engine_path = "/home/ros/Desktop/VisionTest/best20250731.engine";
    std::ifstream engine_file(engine_path, std::ios::binary);
    if (!engine_file.is_open()) {
        ROS_ERROR("Cannot open engine file: %s", engine_path.c_str());
        return -1;
    }
    engine_file.seekg(0, std::ios::end);
    size_t size = engine_file.tellg();
    engine_file.seekg(0, std::ios::beg);
    std::vector<char> engine_data(size);
    engine_file.read(engine_data.data(), size);
    engine_file.close();

    nvinfer1::IRuntime* runtime = nvinfer1::createInferRuntime(gLogger);
    nvinfer1::ICudaEngine* engine = runtime->deserializeCudaEngine(engine_data.data(), size);
    nvinfer1::IExecutionContext* context = engine->createExecutionContext();
    if (!engine || !context) {
        ROS_ERROR("Failed to create TensorRT engine/context");
        return -1;
    }

    int input_idx = engine->getBindingIndex("images");
    int output_idx = engine->getBindingIndex("output0");
    if (input_idx < 0 || output_idx < 0) {
        ROS_ERROR("Invalid binding names.");
        return -1;
    }

    size_t input_size = 1 * 3 * INPUT_H * INPUT_W * sizeof(float);
    size_t output_size = 1 * (NUM_CLASSES + 4) * NUM_ANCHORS * sizeof(float);
    void* buffers[2];
    cudaMalloc(&buffers[input_idx], input_size);
    cudaMalloc(&buffers[output_idx], output_size);

    // ---------- 2. 打开摄像头 (V4L2 + MJPG 120fps) ----------
    // ---------- 2. 打开摄像头 (GStreamer MJPG 120fps) ----------
    cv::VideoCapture cap;
    std::string gst_pipe =
        "v4l2src device=/dev/video0 ! "
        "image/jpeg, width=1920, height=1200, framerate=120/1 ! "
        "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
        "appsink max-buffers=1 drop=1";

    try {
        ROS_INFO("Opening camera with GStreamer pipeline...");
        cap.open(gst_pipe, cv::CAP_GSTREAMER);
        if (!cap.isOpened()) {
            ROS_ERROR("Failed to open camera via GStreamer.");
            return -1;
        }
        ROS_INFO("GStreamer camera opened successfully.");
    } catch (const cv::Exception& e) {
        ROS_ERROR_STREAM("GStreamer open exception: " << e.what());
        return -1;
    }

    std::thread t_capture(capture_thread, std::ref(cap));

    // ---------- 3. 主循环 ----------
    cv::Mat frame, display_frame;
    cv::namedWindow("Detection", cv::WINDOW_NORMAL);
    //cv::resizeWindow("Detection", 960, 540);

    double prev_time = ros::Time::now().toSec();
    double fps = 0.0;

    while (ros::ok()) {
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!latest_frame.empty())
                latest_frame.copyTo(frame);
        }
        if (frame.empty()) {
            ros::Duration(0.001).sleep();
            continue;
        }

        try {
            cv::Mat rgb, resized, blob;
            cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
            cv::resize(rgb, resized, cv::Size(INPUT_W, INPUT_H));
            resized.convertTo(blob, CV_32FC3, 1.0 / 255.0);

            std::vector<float> input_data(3 * INPUT_H * INPUT_W);
            for (int c = 0; c < 3; ++c)
                for (int h = 0; h < INPUT_H; ++h)
                    for (int w = 0; w < INPUT_W; ++w)
                        input_data[c * INPUT_H * INPUT_W + h * INPUT_W + w] = blob.at<cv::Vec3f>(h, w)[c];
            cudaMemcpy(buffers[input_idx], input_data.data(), input_size, cudaMemcpyHostToDevice);

            context->executeV2(buffers);

            std::vector<float> output_cpu(output_size / sizeof(float));
            cudaMemcpy(output_cpu.data(), buffers[output_idx], output_size, cudaMemcpyDeviceToHost);

            auto detections = parse_yolov8(output_cpu.data(), frame.cols, frame.rows);

            // 绘制
            display_frame = frame.clone();
            int w = display_frame.cols, h = display_frame.rows;
            cv::line(display_frame, cv::Point(0,0), cv::Point(w,0), cv::Scalar(255,0,0), 2);
            cv::line(display_frame, cv::Point(0,0), cv::Point(0,h), cv::Scalar(0,255,0), 2);
            cv::putText(display_frame, "X", cv::Point(w-20,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);
            cv::putText(display_frame, "Y", cv::Point(20,h-20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);

            for (size_t i = 0; i < detections.size(); ++i) {
                const Detection& d = detections[i];
                cv::rectangle(display_frame, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2),
                              cv::Scalar(0,255,0), 2);
                std::string label = cv::format("%.2f", d.conf);
                cv::putText(display_frame, label, cv::Point(d.x1, d.y1-10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
            }

            double now = ros::Time::now().toSec();
            double elapsed = now - prev_time;
            if (elapsed > 0) fps = 1.0 / elapsed;
            prev_time = now;
            cv::putText(display_frame, "FPS: " + std::to_string((int)fps),
                        cv::Point(10,30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);

            cv::Mat display_small;
            cv::resize(display_frame, display_small, cv::Size(new_w, new_h));
            cv::imshow("Detection", display_small);
            cv::waitKey(1);

            // 发布 ROS 消息
            px4_controller::tbag msg;
            msg.Target1_Exist = 0; msg.Target2_Exist = 0; msg.Target3_Exist = 0;
            for (size_t i = 0; i < detections.size(); ++i) {
                const Detection& d = detections[i];
                if (i == 0) {
                    msg.Target1_Exist = 1; msg.Target1_PR = d.conf;
                    msg.Target1_LU_x = d.x1; msg.Target1_LU_y = d.y1;
                    msg.Target1_RU_x = d.x2; msg.Target1_RU_y = d.y1;
                    msg.Target1_RD_x = d.x2; msg.Target1_RD_y = d.y2;
                    msg.Target1_LD_x = d.x1; msg.Target1_LD_y = d.y2;
                } else if (i == 1) {
                    msg.Target2_Exist = 1; msg.Target2_PR = d.conf;
                    msg.Target2_LU_x = d.x1; msg.Target2_LU_y = d.y1;
                    msg.Target2_RU_x = d.x2; msg.Target2_RU_y = d.y1;
                    msg.Target2_RD_x = d.x2; msg.Target2_RD_y = d.y2;
                    msg.Target2_LD_x = d.x1; msg.Target2_LD_y = d.y2;
                } else if (i == 2) {
                    msg.Target3_Exist = 1; msg.Target3_PR = d.conf;
                    msg.Target3_LU_x = d.x1; msg.Target3_LU_y = d.y1;
                    msg.Target3_RU_x = d.x2; msg.Target3_RU_y = d.y1;
                    msg.Target3_RD_x = d.x2; msg.Target3_RD_y = d.y2;
                    msg.Target3_LD_x = d.x1; msg.Target3_LD_y = d.y2;
                }
            }
            pub.publish(msg);

            ROS_INFO("send target1 lu: x=%.1f,y=%.1f,msg.Target1_LU_x,msg.Target1_LU_y");

            ros::spinOnce();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Main loop error: " << e.what());
            break;
        }

        if (cv::waitKey(1) == 'q') break;
    }

    // 清理
    stop_flag = true;
    t_capture.join();
    cudaFree(buffers[input_idx]);
    cudaFree(buffers[output_idx]);
    context->destroy();
    engine->destroy();
    runtime->destroy();
    cap.release();
    cv::destroyAllWindows();

    return 0;
}