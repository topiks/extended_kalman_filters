#ifndef KALMAN_FILTER_TYPE6_H
#define KALMAN_FILTER_TYPE6_H

#include <iostream>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

class KalmanFilterType6
{
    public:
        void init(float _dt, float _q_noise, float _P_mea, const MatrixXd& _X_est, const MatrixXd& _P_est);
        void predict();
        void update(float _z_measure, int _mode);

        MatrixXd F{3,3};

        MatrixXd X_est{3, 1};
        MatrixXd P_est{3, 3};

        MatrixXd X_predict;
        MatrixXd P_predict;

        MatrixXd Q_noise{3,3};

        MatrixXd H{1,3};

        MatrixXd K_gain;
        MatrixXd P_mea{1, 1};

        int mode = 0;

        float dt;

};

void KalmanFilterType6::init(float _dt, float _q_noise, float _P_mea, const MatrixXd& _X_est, const MatrixXd& _P_est)
{
    dt = _dt;

    // x estimate
    X_est = _X_est;

    // transition matrix
    F << 1, dt, 0.5 * dt * dt,
         0, 1,  dt,
         0, 0,  1;

    // covariance matrix
    P_est = _P_est;

    // noise covariance matrix
    Q_noise << 0.25 * dt * dt * dt * dt, 0.5 * dt * dt * dt, 0.5 * dt * dt,
               0.5 * dt * dt * dt, dt * dt, dt,
               0.5 * dt * dt, dt, 1;
    Q_noise = Q_noise * _q_noise;

    // measurement covariance matrix
    P_mea << _P_mea;
}

void KalmanFilterType6::update(float _z_measure, int _mode)
{
    predict();
    MatrixXd Z_measure{3,1};

    // mode 0: position
    // mode 1: velocity
    if(_mode == 0)
    {
        Z_measure << _z_measure, 0, 0;
        H << 1, 0, 0;
    }
    else if(_mode == 1)
    {
        Z_measure << 0, _z_measure, 0;
        H << 0, 1, 0;
    }

    Z_measure = H * Z_measure;
    
    K_gain = P_predict * H.transpose() * (H * P_predict * H.transpose() + P_mea).inverse();

    X_est = X_predict + K_gain * (Z_measure - H * X_predict);
    P_est = (MatrixXd::Identity(3,3) - K_gain * H) * P_predict * (MatrixXd::Identity(3,3) - K_gain * H).transpose() + K_gain * P_mea * K_gain.transpose();

    // cout << "Z_measure: " << Z_measure << endl;

    // cout << "X_est: " << X_est << endl;
    // cout << "======" << endl;
}

void KalmanFilterType6::predict()
{
    X_predict = F * X_est;
    P_predict = F * P_est * F.transpose() + Q_noise;

    // cout << "X_predict: " << X_predict << endl;
    // cout << "P_predict: " << P_predict << endl;
    // cout << "---------" << endl;
}

#endif