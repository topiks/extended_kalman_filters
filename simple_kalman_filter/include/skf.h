#ifndef SKF_H
#define SKF_H

#include <iostream>
#include <cmath>

using namespace std;

class SimpleKalmanFilter
{
public:
    SimpleKalmanFilter(double e_est_, double e_mea_, double sensitivity_);

    double e_mea;
    double sensitivity;
    double e_est, e_est_last = 0.0;
    double est_val, est_val_last = 0.0;

    double update_estimate(double mea_val_);

private:
    double k_gain;    
};

SimpleKalmanFilter::SimpleKalmanFilter(double e_est_, double e_mea_, double sensitivity_)
{
    e_est = e_est_;
    e_mea = e_mea_;
    sensitivity = sensitivity_;
}

double SimpleKalmanFilter::update_estimate(double mea_val_)
{
    k_gain = e_est / (e_est + e_mea);
    est_val = est_val_last + k_gain * (mea_val_ - est_val_last);
    e_est = (1.0 - k_gain) * e_est_last + fabs(est_val_last - est_val) * sensitivity;

    est_val_last = est_val;
    e_est_last = e_est;

    return est_val;
}

#endif