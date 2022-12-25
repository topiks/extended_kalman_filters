#ifndef KALMAN_FILTER_TYPE2_H
#define KALMAN_FILTER_TYPE2_H

#include <iostream>

using namespace std;

class KalmanFilterType2
{
    public:
        void init(float _s_estimate, float _v_estimate, float _dt);
        void update(float _z_measurement);
        void predict();

        float s_estimate;
        float v_estimate;

        float s_predict;
        float v_predict;

        float dt;

        float gain_s = 0.55;
        float gain_v = 0.3;

        float s_measurement;
};

void KalmanFilterType2::init(float _s_estimate, float _v_estimate, float _dt)
{
    s_estimate = _s_estimate;
    v_estimate = _v_estimate;
    dt = _dt;
}

void KalmanFilterType2::update(float _z_measurement)
{
    predict();
    s_estimate = s_predict + gain_s * (_z_measurement - s_predict);
    v_estimate = v_predict + gain_v * (_z_measurement - s_predict) / dt;

    cout << "s_estimate = " << s_estimate << " = " << s_predict << " + " << gain_s << " * " << "(" << _z_measurement << " - " << s_predict << ")" << endl;
    cout << "v_estimate = " << v_estimate << " = " << v_predict << " + " << gain_v << " * " << "(" << _z_measurement << " - " << s_predict << ")" << " / " << dt << endl;

    cout << "==========================" << endl;
}

void KalmanFilterType2::predict()
{
    s_predict = s_estimate + v_estimate * dt;
    v_predict = v_estimate;

    cout << "s_predict = " << s_predict << " = " << s_estimate << " + " << v_estimate << " * " << dt << endl;
    cout << "v_predict = " << v_predict << " = " << v_estimate << endl;
}

#endif