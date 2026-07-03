#include "calibration.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <stdexcept>
#include <iostream>   // 用于错误输出（可改为日志）

using namespace cv;
using namespace std;



// ---------------- HeightCalibration 成员函数 ----------------

void HeightCalibration::addCalibrationData(double h, Point2f p1, Point2f p2) {
    heights_.push_back(h);
    pts1_.push_back(p1);
    pts2_.push_back(p2);
}

void HeightCalibration::sortAndValidate() {
    if (heights_.empty()) return;

    // 生成索引并按高度排序
    vector<size_t> idx(heights_.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&](size_t i, size_t j) {
        return heights_[i] < heights_[j];
    });

    vector<double> sorted_h;
    vector<Point2f> sorted_p1, sorted_p2;
    sorted_h.reserve(heights_.size());
    sorted_p1.reserve(heights_.size());
    sorted_p2.reserve(heights_.size());

    for (size_t i : idx) {
        sorted_h.push_back(heights_[i]);
        sorted_p1.push_back(pts1_[i]);
        sorted_p2.push_back(pts2_[i]);
    }

    heights_ = move(sorted_h);
    pts1_ = move(sorted_p1);
    pts2_ = move(sorted_p2);
}

pair<Point2f, Point2f> HeightCalibration::query(double h) const {
    if (heights_.empty()) {
        throw runtime_error("HeightCalibration: 标定数据为空，无法查询");
    }

    // 边界处理
    if (h <= heights_.front()) {
        return {pts1_.front(), pts2_.front()};
    }
    if (h >= heights_.back()) {
        return {pts1_.back(), pts2_.back()};
    }

    // 二分查找区间
    auto it = upper_bound(heights_.begin(), heights_.end(), h);
    int idx_high = int(it - heights_.begin());
    int idx_low = idx_high - 1;

    double h_low = heights_[idx_low];
    double h_high = heights_[idx_high];
    double ratio = (h - h_low) / (h_high - h_low);

    Point2f p1_low = pts1_[idx_low];
    Point2f p1_high = pts1_[idx_high];
    Point2f p2_low = pts2_[idx_low];
    Point2f p2_high = pts2_[idx_high];

    Point2f p1_interp = p1_low + (p1_high - p1_low) * ratio;
    Point2f p2_interp = p2_low + (p2_high - p2_low) * ratio;

    return {p1_interp, p2_interp};
}

size_t HeightCalibration::size() const {
    return heights_.size();
}

// ---------------- 辅助函数实现 ----------------

bool loadCalibrationFromFile(const string& filename, HeightCalibration& calib) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "[HeightCalibration] 无法打开文件: " << filename << endl;
        return false;
    }

    string line;
    int line_num = 0;
    while (getline(file, line)) {
        line_num++;

        // 去除首尾空白
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);

        if (line.empty() || line[0] == '#') continue;

        stringstream ss(line);
        double h, x1, y1, x2, y2;
        char comma;

        // 尝试按空格分隔
        if (ss >> h >> x1 >> y1 >> x2 >> y2) {
            calib.addCalibrationData(h,
                                     Point2f((float)x1, (float)y1),
                                     Point2f((float)x2, (float)y2));
        } else {
            // 尝试按逗号分隔 (CSV)
            ss.clear();
            ss.str(line);
            if (ss >> h >> comma >> x1 >> comma >> y1 >> comma >> x2 >> comma >> y2) {
                calib.addCalibrationData(h,
                                         Point2f((float)x1, (float)y1),
                                         Point2f((float)x2, (float)y2));
            } else {
                cerr << "[HeightCalibration] 警告: 第 " << line_num << " 行格式错误，已跳过" << endl;
            }
        }
    }

    file.close();

    if (calib.size() == 0) {
        cerr << "[HeightCalibration] 未读取到有效数据" << endl;
        return false;
    }

    calib.sortAndValidate();
    cout << "[HeightCalibration] 成功加载 " << calib.size() << " 组数据" << endl;
    return true;
}