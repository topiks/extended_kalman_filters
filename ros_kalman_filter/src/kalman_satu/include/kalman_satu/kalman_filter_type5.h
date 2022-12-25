#ifndef KALMAN_FILTER_TYPE5_H
#define KALMAN_FILTER_TYPE5_H

#include "iostream"

using namespace std;

class KalmanFilterType5
{
    public:
        void init(float _s_estimate, float _err_variance_estimate, float _err_variance_measurement, float _q_noise_variance);
        void predict();
        void update(float _s_measurement);

        float s_estimate;
        float err_variance_estimate;

        float s_predict;
        float err_variance_predict;

        float s_measurement;
        float err_variance_measurement;

        float q_noise_variance;
};

void KalmanFilterType5::init(float _s_estimate, float _err_variance_estimate, float _err_variance_measurement, float _q_noise_variance)
{
    s_estimate = _s_estimate;
    err_variance_estimate = _err_variance_estimate;
    err_variance_measurement = _err_variance_measurement;
    q_noise_variance = _q_noise_variance;
}

void KalmanFilterType5::predict()
{
    s_predict = s_estimate;
    err_variance_predict = err_variance_estimate + q_noise_variance;

    cout << "s_predict: " << s_predict << endl;
    cout << "err_variance_predict = " << err_variance_predict << " = " << err_variance_estimate << " + " << q_noise_variance << endl;

    cout << "-------------" << endl;
}

void KalmanFilterType5::update(float _s_measurement)
{
    predict();

    float kalman_gain = err_variance_predict / (err_variance_predict + err_variance_measurement);
    s_estimate = s_predict + kalman_gain * (_s_measurement - s_predict);
    err_variance_estimate = (1 - kalman_gain) * err_variance_predict;

    cout << "kalman gain = " << kalman_gain << " = " << err_variance_predict << " / (" << err_variance_predict << " + " << err_variance_measurement << ")" << endl;
    cout << "s_estimate: " << s_estimate << " = " << s_predict << " + " << kalman_gain << " * (" << _s_measurement << " - " << s_predict << ")" << endl;
    cout << "err_variance_estimate: " << err_variance_estimate << " = (1 - " << kalman_gain << ") * " << err_variance_predict << endl;

    cout << "======================================" << endl;
}

#endif