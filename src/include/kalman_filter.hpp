#pragma once
#include <opencv2/opencv.hpp>

namespace at{
class AngleKalmanFilter {
public:
    AngleKalmanFilter(float dt = 1.0f) {
        kf = cv::KalmanFilter(2, 1, 0);  // 状态：[角度, 角速度]；测量：[角度]
        // 状态转移矩阵 A
        kf.transitionMatrix = (cv::Mat_<float>(2, 2) <<
            1, dt,
            0, 1);
        // 测量矩阵 H：只测量角度
        kf.measurementMatrix = (cv::Mat_<float>(1, 2) <<
            1, 0);
        // 初始化状态
        kf.statePre = (cv::Mat_<float>(2, 1) << 0, 0);
        // 协方差设置
        cv::setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));   // Q
        cv::setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-1)); // R
        cv::setIdentity(kf.errorCovPost, cv::Scalar::all(1));         // P
    }
    // 输入测量角度（单位：度），返回滤波后的角度（度）
    [[nodiscard]] float update(float measured_angle) {
        // 角度 wrap 到 [-180, 180]
        measured_angle = wrapAngle(measured_angle);
        // 预测
        kf.predict();
        // 纠正
        cv::Mat measurement = (cv::Mat_<float>(1, 1) << measured_angle);
        cv::Mat estimate = kf.correct(measurement);

        return estimate.at<float>(0);  // 返回滤波后的角度
    }
    float getVelocity() const {
        return kf.statePost.at<float>(1);
    }

private:
    cv::KalmanFilter kf;

    float wrapAngle(float angle) const {
        while (angle > 180.f) angle -= 360.f;
        while (angle < -180.f) angle += 360.f;
        return angle;
    }
};
} // namespace at
