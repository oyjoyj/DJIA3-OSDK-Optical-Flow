#include <iostream>

class KalmanFilter {
public:
    KalmanFilter(double initial_state, double initial_covariance, double process_noise, double measurement_noise)
        : state(initial_state), covariance(initial_covariance), process_noise(process_noise), measurement_noise(measurement_noise) {}
    void predict() {
        // 状态预测
        state = state; // 状态预测公式（对于一维恒定的例子来说没有动态模型）
        // 协方差预测
        covariance = covariance + process_noise;
    }
    void correct(double measurement) {
        // 卡尔曼增益计算
        double kalman_gain = covariance / (covariance + measurement_noise);
        // 状态更新
        state = state + kalman_gain * (measurement - state);
        // 协方差更新
        covariance = (1 - kalman_gain) * covariance;
    }
    double getState() const {
        return state;
    }
private:
    double state;        // 状态估计
    double covariance;   // 协方差
    double process_noise; // 过程噪声
    double measurement_noise; // 测量噪声
};
