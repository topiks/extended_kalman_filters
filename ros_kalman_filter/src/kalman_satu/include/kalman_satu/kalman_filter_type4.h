#ifndef KALMAN_FILTER_TYPE4_H
#define KALMAN_FILTER_TYPE4_H

#include "iostream"

using namespace std;

class KalmanFilterType4
{
    public:
        void init(float _s_estimate, float _variance_estimate, float _variance_measurement);
        void predict();
        void update(float _s_measurement);

        float kalman_gain;

        float s_estimate;
        float variance_estimate;

        float s_predict;
        float variance_predict;

        float s_measurement;
        float variance_measurement;
};

void KalmanFilterType4::init(float _s_estimate, float _variance_estimate, float _variance_measurement)
{
    s_estimate = _s_estimate;
    variance_estimate = _variance_estimate;
    variance_measurement = _variance_measurement;
}

void KalmanFilterType4::predict()
{
    s_predict = s_estimate;
    variance_predict = variance_estimate;

    cout << "s_predict: " << s_predict << endl;
    cout << "variance_predict: " << variance_predict << endl;

    cout << "-------------" << endl;
}

void KalmanFilterType4::update(float _s_measurement)
{
    predict();

    kalman_gain = variance_predict / (variance_predict + variance_measurement);
    s_estimate = s_predict + kalman_gain * (_s_measurement - s_predict);
    variance_estimate = (1 - kalman_gain) * variance_predict;

    cout << "kalman gain = " << kalman_gain << " = " << variance_predict << " / (" << variance_predict << " + " << variance_measurement << ")" << endl;
    cout << "s_estimate: " << s_estimate << " = " << s_predict << " + " << kalman_gain << " * (" << _s_measurement << " - " << s_predict << ")" << endl;
    cout << "variance_estimate: " << variance_estimate << " = (1 - " << kalman_gain << ") * " << variance_predict << endl;

    cout << "======================================" << endl;
}

#endif