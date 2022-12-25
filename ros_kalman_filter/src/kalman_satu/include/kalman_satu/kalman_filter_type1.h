#ifndef KALMAN_FILTER_TYPE1_H
#define KALMAN_FILTER_TYPE1_H

class KalmanFilterType1
{
public:
    void init(float _x_estimate);    
    void update(float _z_measurement);
    void predict();

    float gain = 0.1;

    float x_predict;
    float x_estimate;
    float z_measurement;
};

void KalmanFilterType1::init(float _x_estimate)
{
    x_estimate = _x_estimate;
}

void KalmanFilterType1::update(float _z_measurement)
{
    predict();
    x_estimate = x_predict + gain * (_z_measurement - x_predict);
}

void KalmanFilterType1::predict()
{
    x_predict = x_estimate;
}

#endif