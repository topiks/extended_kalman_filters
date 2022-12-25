#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <iostream>
#include <iomanip>

using namespace std;

/*
    Pemodelan Kalman Filter ini menggunakan constant acceleration model
    Variabel v dan a digunakan untuk mengkalkulasi posisi 
*/

class KalmanFilter
{
public:
    void init(float _s, float _v, float _a, float _dt, float _error_est, float _error_meas);
    void predict();
    void measure(float _s);
    void update();

    float s_now, s_predict;
    float v_now, v_predict;
    float a_now, a_predict;
    float dt;

    //=============================

    float error_estimate;
    float error_measurement;

    //=============================

    float z_measure;

    //=============================

    float kalman_gain;

};

void KalmanFilter::init(float _s, float _v, float _a, float _dt, float _error_est, float _error_meas)
{
    s_now = _s;
    v_now = _v;
    a_now = _a;
    dt = _dt;

    //=============================

    error_estimate = _error_est;
    error_measurement = _error_meas;
}

void KalmanFilter::predict()
{
    s_predict = s_now + v_now * dt + 0.5 * a_now * dt * dt;
    v_predict = v_now + a_now * dt;
    a_predict = a_now;

    // cout << "s_predict " << s_predict << " v_predict " << v_predict << " a_predict " << a_predict << endl;
    // cout << "=============================" << endl;
    // cout << "s_predict = " << s_predict << " = " << s_now << " + " << v_now << " * " << dt << " + " << 0.5 << " * " << a_now << " * " << dt << " * " << dt << endl;
    // cout << "v_predict = " << v_predict << " = " << v_now << " + " << a_now << " * " << dt << endl;
    // cout << "a_predict = " << a_predict << " = " << a_now << endl;

    // cout << "-----------------------------" << endl;

}

void KalmanFilter::measure(float _s)
{
    z_measure = _s;
}

void KalmanFilter::update()
{
    // estimate kalman gain
    kalman_gain = error_estimate / (error_estimate + error_measurement);

    // cout << "kalman_gain = " << kalman_gain <<  endl;
    // cout << "-----------------------------" << endl;

    // estimate current state with update function
    s_now = s_predict + kalman_gain * (z_measure - s_predict);
    v_now = v_predict + 0.1 * (z_measure - s_predict) / dt;
    a_now = a_predict + 0.1 * (z_measure - s_predict) / (dt * dt);

    error_estimate = (1 - kalman_gain) * error_estimate;

    // cout << s_now << " = " << s_predict << " + " << kalman_gain << "( " << z_measure << " - " << s_predict << " )" << endl;

    // cout << "s_now = " << s_now << " = " << s_predict << " + " << kalman_gain << " * " << "( " << z_measure << " - " << s_predict << " )" << endl;
    // cout << "v_now = " << v_now << " = " << v_predict << " + " << "0.1" << " * " << "( " << z_measure << " - " << s_predict << " )" << " / " << dt << endl;
    // cout << "a_now = " << a_now << " = " << a_predict << " + " << "0.1" << " * " << "( " << z_measure << " - " << s_predict << " )" << " / " << dt << " * " << dt << endl;

    // cout << "=============================" << endl;

    // calculate the next estimate with predict function
    predict();
}  





#endif