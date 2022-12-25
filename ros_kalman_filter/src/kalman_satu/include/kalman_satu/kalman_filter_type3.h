#ifndef KALMAN_FILTER_TYPE3_H
#define KALMAN_FILTER_TYPE3_H

#include <iostream>

using namespace std;

class KalmanFilterType3
{
    public:
        void init(float _s_estimate, float _v_estimate, float _a_estimate, float _dt);
        void update(float _z_measurement);
        void predict();

        float s_estimate;
        float v_estimate;
        float a_estimate;

        float s_predict;
        float v_predict;
        float a_predict;

        float dt;

        float gain_s = 0.7;
        float gain_v = 0.2;
        float gain_a = 0.05;
};

void KalmanFilterType3::init(float _s_estimate, float _v_estimate, float _a_estimate, float _dt)
{
    s_estimate = _s_estimate;
    v_estimate = _v_estimate;
    a_estimate = _a_estimate;
    dt = _dt;
}

void KalmanFilterType3::update(float _z_measurement)
{
    predict();
    s_estimate = s_predict + gain_s * (_z_measurement - s_predict);
    v_estimate = v_predict + gain_v * (_z_measurement - s_predict) / dt;
    a_estimate = a_predict + gain_a * (_z_measurement - s_predict) / dt / dt;

    cout << "s_estimate = " << s_estimate << " = " << s_predict << " + " << gain_s << " * " << "(" << _z_measurement << " - " << s_predict << ")" << endl;
    cout << "v_estimate = " << v_estimate << " = " << v_predict << " + " << gain_v << " * " << "(" << _z_measurement << " - " << s_predict << ")" << " / " << dt << endl;
    cout << "a_estimate = " << a_estimate << " = " << a_predict << " + " << gain_a << " * " << "(" << _z_measurement << " - " << s_predict << ")" << " / " << dt << " / " << dt << endl;

    cout << "==========================" << endl;
}

void KalmanFilterType3::predict()
{
    s_predict = s_estimate + v_estimate * dt + 0.5 * a_estimate * dt * dt;
    v_predict = v_estimate + a_estimate * dt;
    a_predict = a_estimate;

    cout << "s_predict = " << s_predict << " = " << s_estimate << " + " << v_estimate << " * " << dt << " + " << 0.5 << " * " << a_estimate << " * " << dt << " * " << dt << endl;
    cout << "v_predict = " << v_predict << " = " << v_estimate << " + " << a_estimate << " * " << dt << endl;
    cout << "a_predict = " << a_predict << " = " << a_estimate << endl;
}

#endif