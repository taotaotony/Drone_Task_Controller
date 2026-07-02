#include <ros/ros.h>
#include <px4_controller/tbag.h>
#include <opencv2/opencv.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/core/cuda_stream_accessor.hpp>
#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <cuda_runtime_api.h>
#include <fstream>
#include <vector>
#include <algorithm>
#include <thread>
#include <mutex>
#include <deque>
#include <cmath>
#include <limits>

// ======================== TensorRT Logger ========================
class Logger : public nvinfer1::ILogger {
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING)
            ROS_WARN_STREAM("[TensorRT] " << msg);
    }
} gLogger;

// ======================== 模型参数 ========================
const int INPUT_H = 640;
const int INPUT_W = 640;
const int NUM_ANCHORS = 8400;
const int NUM_CLASSES = 80;
const float CONF_THRESH = 0.4f;
const float IOU_THRESH = 0.45f;
const float CENTER_DIST_RATIO = 0.5f;   // 中心距离合并阈值(相对于较小框宽度)
const int   NEW_W = 864;
const int   NEW_H = 540;

// ======================== 跟踪参数 ========================
const int   TRACK_CONFIRM_FRAMES = 3;
const int   TRACK_MAX_LOST      = 10;
const float TRACK_IOU_MATCH     = 0.3f;
const float KF_PROCESS_NOISE    = 0.01f;
const float KF_MEASURE_NOISE    = 0.1f;
const float CONF_HYSTERESIS_LOW = 0.25f;

// ======================== 数据结构 ========================
struct Detection {
    float x1, y1, x2, y2, conf;
    float cx() const { return (x1 + x2) * 0.5f; }
    float cy() const { return (y1 + y2) * 0.5f; }
    float area() const { return (x2 - x1) * (y2 - y1); }
    float width() const { return x2 - x1; }
};

// ======================== 判定两个检测框是否为同一目标 ========================
// 针对圆形桶目标：除了 IOU，还检查中心距离和中心包含关系
inline bool is_same_target(const Detection& a, const Detection& b,
                           float iou_thres, float center_dist_ratio) {
    // 1. 计算 IOU
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float iw = std::max(0.0f, xx2 - xx1);
    float ih = std::max(0.0f, yy2 - yy1);
    float inter = iw * ih;
    float area_a = a.area();
    float area_b = b.area();
    float iou = inter / (area_a + area_b - inter + 1e-6f);

    if (iou > iou_thres) return true;

    // 2. 中心包含：如果 b 的中心落在 a 内部，视为同一目标
    float bcx = b.cx(), bcy = b.cy();
    if (bcx >= a.x1 && bcx <= a.x2 && bcy >= a.y1 && bcy <= a.y2) return true;

    // 3. 反向：如果 a 的中心落在 b 内部
    float acx = a.cx(), acy = a.cy();
    if (acx >= b.x1 && acx <= b.x2 && acy >= b.y1 && acy <= b.y2) return true;

    // 4. 中心距离：两框中心距离 < 较小框宽度 * ratio
    float dx = acx - bcx;
    float dy = acy - bcy;
    float dist = std::sqrt(dx * dx + dy * dy);
    float min_w = std::min(a.width(), b.width());
    if (dist < min_w * center_dist_ratio) return true;

    return false;
}

// ======================== Kalman 滤波器 (匀速运动模型) ========================
class KalmanTracker {
public:
    enum State { kUnconfirmed, kConfirmed, kLost };

    KalmanTracker(const Detection& det, int track_id)
        : id_(track_id), state_(kUnconfirmed), lost_frames_(0),
          confirm_count_(1), total_frames_(1)
    {
        kf_.init(8, 4, 0, CV_32F);
        kf_.transitionMatrix = (cv::Mat_<float>(8, 8) <<
            1,0,0,0,1,0,0,0,
            0,1,0,0,0,1,0,0,
            0,0,1,0,0,0,1,0,
            0,0,0,1,0,0,0,1,
            0,0,0,0,1,0,0,0,
            0,0,0,0,0,1,0,0,
            0,0,0,0,0,0,1,0,
            0,0,0,0,0,0,0,1);
        kf_.measurementMatrix = (cv::Mat_<float>(4, 8) <<
            1,0,0,0,0,0,0,0,
            0,1,0,0,0,0,0,0,
            0,0,1,0,0,0,0,0,
            0,0,0,1,0,0,0,0);
        cv::setIdentity(kf_.processNoiseCov, cv::Scalar::all(KF_PROCESS_NOISE));
        cv::setIdentity(kf_.measurementNoiseCov, cv::Scalar::all(KF_MEASURE_NOISE));
        cv::setIdentity(kf_.errorCovPost, cv::Scalar::all(1.0));

        kf_.statePost.at<float>(0) = det.cx();
        kf_.statePost.at<float>(1) = det.cy();
        kf_.statePost.at<float>(2) = det.width();
        kf_.statePost.at<float>(3) = det.y2 - det.y1;
        kf_.statePost.at<float>(4) = 0;
        kf_.statePost.at<float>(5) = 0;
        kf_.statePost.at<float>(6) = 0;
        kf_.statePost.at<float>(7) = 0;

        best_conf_ = det.conf;
        update_bbox();
    }

    void predict() {
        kf_.predict();
        update_bbox();
    }

    void update(const Detection& det) {
        cv::Mat measurement = (cv::Mat_<float>(4, 1) <<
            det.cx(), det.cy(), det.width(), det.y2 - det.y1);
        kf_.correct(measurement);

        lost_frames_ = 0;
        total_frames_++;
        best_conf_ = std::max(best_conf_, det.conf);

        if (state_ == kUnconfirmed) {
            confirm_count_++;
            if (confirm_count_ >= TRACK_CONFIRM_FRAMES)
                state_ = kConfirmed;
        }
        update_bbox();
    }

    void mark_lost() {
        lost_frames_++;
        if (lost_frames_ > TRACK_MAX_LOST)
            state_ = kLost;
    }

    Detection get_bbox() const { return bbox_; }
    int id() const { return id_; }
    State state() const { return state_; }
    bool is_confirmed() const { return state_ == kConfirmed; }
    bool is_lost() const { return state_ == kLost; }
    int lost_frames() const { return lost_frames_; }
    float confidence() const { return best_conf_; }

    float smoothed_conf() const {
        float raw = best_conf_;
        float bonus = std::min(0.15f, total_frames_ * 0.002f);
        return std::min(1.0f, raw + bonus);
    }

private:
    void update_bbox() {
        float cx = kf_.statePost.at<float>(0);
        float cy = kf_.statePost.at<float>(1);
        float w  = kf_.statePost.at<float>(2);
        float h  = kf_.statePost.at<float>(3);
        bbox_.x1 = cx - w * 0.5f;
        bbox_.y1 = cy - h * 0.5f;
        bbox_.x2 = cx + w * 0.5f;
        bbox_.y2 = cy + h * 0.5f;
        bbox_.conf = best_conf_;
    }

    cv::KalmanFilter kf_;
    Detection bbox_;
    int id_;
    State state_;
    int lost_frames_;
    int confirm_count_;
    int total_frames_;
    float best_conf_;
};

// ======================== SORT 多目标跟踪器 ========================
class SORTTracker {
public:
    SORTTracker() : next_id_(0) {}

    std::vector<Detection> update(const std::vector<Detection>& detections, int img_w, int img_h) {
        for (auto& track : tracks_) {
            track.predict();
        }

        std::vector<std::vector<float>> iou_matrix(tracks_.size(),
            std::vector<float>(detections.size(), 0.0f));
        for (size_t t = 0; t < tracks_.size(); ++t) {
            Detection tbox = tracks_[t].get_bbox();
            for (size_t d = 0; d < detections.size(); ++d) {
                iou_matrix[t][d] = 1.0f - compute_iou(tbox, detections[d]);
            }
        }

        std::vector<int> track_match(tracks_.size(), -1);
        std::vector<bool> det_matched(detections.size(), false);
        greedy_match(iou_matrix, track_match, det_matched, TRACK_IOU_MATCH);

        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (track_match[t] >= 0) {
                tracks_[t].update(detections[track_match[t]]);
            } else {
                tracks_[t].mark_lost();
            }
        }

        for (size_t d = 0; d < detections.size(); ++d) {
            if (!det_matched[d]) {
                tracks_.emplace_back(detections[d], next_id_++);
            }
        }

        tracks_.erase(
            std::remove_if(tracks_.begin(), tracks_.end(),
                [](const KalmanTracker& t) { return t.is_lost(); }),
            tracks_.end());

        std::vector<Detection> output;
        for (auto& track : tracks_) {
            if (track.is_confirmed()) {
                Detection d = track.get_bbox();
                d.conf = track.smoothed_conf();
                output.push_back(d);
            }
        }

        std::sort(output.begin(), output.end(),
            [](const Detection& a, const Detection& b) { return a.conf > b.conf; });
        if (output.size() > 3) output.resize(3);

        return output;
    }

private:
    float compute_iou(const Detection& a, const Detection& b) {
        float xx1 = std::max(a.x1, b.x1);
        float yy1 = std::max(a.y1, b.y1);
        float xx2 = std::min(a.x2, b.x2);
        float yy2 = std::min(a.y2, b.y2);
        float w = std::max(0.0f, xx2 - xx1);
        float h = std::max(0.0f, yy2 - yy1);
        float inter = w * h;
        float area_a = a.area();
        float area_b = b.area();
        float denom = area_a + area_b - inter;
        return (denom > 0.0f) ? inter / denom : 0.0f;
    }

    void greedy_match(const std::vector<std::vector<float>>& cost,
                      std::vector<int>& track_match,
                      std::vector<bool>& det_matched,
                      float threshold)
    {
        struct Match { int t, d; float c; };
        std::vector<Match> candidates;
        for (size_t t = 0; t < cost.size(); ++t)
            for (size_t d = 0; d < cost[t].size(); ++d)
                if (cost[t][d] < 1.0f - threshold)
                    candidates.push_back({(int)t, (int)d, cost[t][d]});

        std::sort(candidates.begin(), candidates.end(),
            [](const Match& a, const Match& b) { return a.c < b.c; });

        for (const auto& m : candidates) {
            if (track_match[m.t] < 0 && !det_matched[m.d]) {
                track_match[m.t] = m.d;
                det_matched[m.d] = true;
            }
        }
    }

    std::vector<KalmanTracker> tracks_;
    int next_id_;
};

// ======================== 异步取帧 ========================
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
            }
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Capture error: " << e.what());
        }
    }
}

// ======================== 改进版 NMS: IOU + 中心距离联合去重 ========================
// 针对圆形桶目标光照变化导致的重复检测:
//   1. 标准 IOU > 阈值 → 合并
//   2. 任一框的中心落在对方框内 → 视为同一目标合并
//   3. 中心距离 < 较小框宽度 * ratio → 合并
void apply_nms(std::vector<Detection>& dets, float iou_thres) {
    if (dets.empty()) return;
    // 按置信度降序排列
    std::sort(dets.begin(), dets.end(),
              [](const Detection& a, const Detection& b) { return a.conf > b.conf; });

    std::vector<Detection> result;
    while (!dets.empty()) {
        // 保留置信度最高的框
        result.push_back(dets[0]);
        std::vector<Detection> remaining;

        for (size_t i = 1; i < dets.size(); ++i) {
            if (!is_same_target(dets[0], dets[i], iou_thres, CENTER_DIST_RATIO)) {
                remaining.push_back(dets[i]);
            }
        }
        dets = remaining;
    }
    dets = result;
}

// ======================== YOLOv8 输出解码 ========================
std::vector<Detection> parse_yolov8(float* output, int img_width, int img_height) {
    std::vector<Detection> raw_dets;
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
    return raw_dets;
}

// ======================== CUDA 加速预处理 ========================
void preprocess_cuda(const cv::Mat& frame, float* d_input, cudaStream_t stream) {
    cv::cuda::Stream cv_stream = cv::cuda::StreamAccessor::wrapStream(stream);

    cv::cuda::GpuMat gpu_frame, gpu_rgb, gpu_resized, gpu_float;
    gpu_frame.upload(frame, cv_stream);

    cv::cuda::cvtColor(gpu_frame, gpu_rgb, cv::COLOR_BGR2RGB, 0, cv_stream);
    cv::cuda::resize(gpu_rgb, gpu_resized, cv::Size(INPUT_W, INPUT_H), 0, 0,
                     cv::INTER_LINEAR, cv_stream);
    gpu_resized.convertTo(gpu_float, CV_32F, 1.0 / 255.0, cv_stream);

    std::vector<cv::cuda::GpuMat> channels(3);
    for (int c = 0; c < 3; ++c) {
        channels[c] = cv::cuda::GpuMat(INPUT_H, INPUT_W, CV_32F,
            d_input + c * INPUT_H * INPUT_W);
    }
    cv::cuda::split(gpu_float, channels, cv_stream);

    cv_stream.waitForCompletion();
}

// ======================== 主函数 ========================
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

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    std::vector<float> output_cpu(output_size / sizeof(float));

    // ---------- 2. 打开摄像头 (GStreamer) ----------
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

    // ---------- 3. 初始化 SORT 跟踪器 ----------
    SORTTracker tracker;

    // ---------- 4. 主循环 ----------
    cv::Mat frame, display_frame;
    cv::namedWindow("Detection", cv::WINDOW_NORMAL);

    double prev_time = ros::Time::now().toSec();
    double fps = 0.0;
    int frame_count = 0;
    double fps_accum = 0.0;
    double fps_avg = 0.0;

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
            // ---- 4a. CUDA 预处理 + TensorRT 推理 ----
            preprocess_cuda(frame, (float*)buffers[input_idx], stream);
            context->enqueueV2(buffers, stream, nullptr);
            cudaMemcpyAsync(output_cpu.data(), buffers[output_idx], output_size,
                            cudaMemcpyDeviceToHost, stream);
            cudaStreamSynchronize(stream);

            // ---- 4b. 解码检测结果 ----
            auto detections = parse_yolov8(output_cpu.data(), frame.cols, frame.rows);

            // ---- 4c. SORT 多目标跟踪 ----
            auto tracked = tracker.update(detections, frame.cols, frame.rows);

            // ---- 4d. 绘制显示 ----
            display_frame = frame.clone();
            int w = display_frame.cols, h = display_frame.rows;

            cv::line(display_frame, cv::Point(0,0), cv::Point(w,0), cv::Scalar(255,0,0), 2);
            cv::line(display_frame, cv::Point(0,0), cv::Point(0,h), cv::Scalar(0,255,0), 2);
            cv::putText(display_frame, "X", cv::Point(w-20,20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,0,0), 2);
            cv::putText(display_frame, "Y", cv::Point(20,h-20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);

            for (size_t i = 0; i < tracked.size(); ++i) {
                const Detection& d = tracked[i];
                cv::rectangle(display_frame, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2),
                              cv::Scalar(0, 255, 0), 3);
                cv::drawMarker(display_frame,
                    cv::Point((d.x1 + d.x2) / 2, (d.y1 + d.y2) / 2),
                    cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 20, 2);
                std::string label = cv::format("T%d %.2f", (int)i + 1, d.conf);
                cv::putText(display_frame, label, cv::Point(d.x1, d.y1 - 10),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
            }

            for (size_t i = 0; i < detections.size(); ++i) {
                const Detection& d = detections[i];
                cv::rectangle(display_frame, cv::Point(d.x1, d.y1), cv::Point(d.x2, d.y2),
                              cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            }

            double now = ros::Time::now().toSec();
            double elapsed = now - prev_time;
            if (elapsed > 0) {
                fps = 1.0 / elapsed;
                fps_accum += fps;
                frame_count++;
                if (frame_count >= 30) {
                    fps_avg = fps_accum / frame_count;
                    fps_accum = 0.0;
                    frame_count = 0;
                }
            }
            prev_time = now;

            cv::putText(display_frame, "FPS: " + std::to_string((int)fps) +
                        " (avg: " + std::to_string((int)fps_avg) + ")",
                        cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 255, 0), 2);
            cv::putText(display_frame, "Tracked: " + std::to_string(tracked.size()) +
                        "  Raw: " + std::to_string(detections.size()),
                        cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(255, 255, 0), 2);

            if (tracked.empty()) {
                cv::putText(display_frame, "NO TARGET", cv::Point(w/2 - 80, h/2),
                            cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0, 0, 255), 3);
            }

            cv::Mat display_small;
            cv::resize(display_frame, display_small, cv::Size(NEW_W, NEW_H));
            cv::imshow("Detection", display_small);
            cv::waitKey(1);

            // ---- 4e. 发布 ROS 消息 ----
            px4_controller::tbag msg;
            msg.Target1_Exist = 0; msg.Target2_Exist = 0; msg.Target3_Exist = 0;

            for (size_t i = 0; i < tracked.size() && i < 3; ++i) {
                const Detection& d = tracked[i];
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

            ros::spinOnce();
        } catch (const cv::Exception& e) {
            ROS_ERROR_STREAM("Main loop error: " << e.what());
            break;
        }

        if (cv::waitKey(1) == 'q') break;
    }

    // ---------- 5. 清理 ----------
    stop_flag = true;
    if (t_capture.joinable()) t_capture.join();
    cudaStreamDestroy(stream);
    cudaFree(buffers[input_idx]);
    cudaFree(buffers[output_idx]);
    context->destroy();
    engine->destroy();
    runtime->destroy();
    cap.release();
    cv::destroyAllWindows();

    return 0;
}