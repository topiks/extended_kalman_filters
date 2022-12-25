#include "ros/ros.h"
#include "ros/package.h"
#include "fstream"
#include "iostream"
#include "opencv2/opencv.hpp"
#include "kalman_satu/kalman_filter.h"
#include "kalman_satu/data_dummy.h"
#include "kalman_satu/kalman_filter_type1.h"
#include "kalman_satu/kalman_filter_type2.h"
#include "kalman_satu/kalman_filter_type3.h"
#include "kalman_satu/kalman_filter_type4.h"
#include "kalman_satu/kalman_filter_type5.h"
#include "kalman_satu/kalman_filter_type6.h"

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

//==============================================================================

int kalman_satu_init();
void cllbck_tim_50_hz(const ros::TimerEvent& event);
void cllbck_tim_odo_1_hz(const ros::TimerEvent& event);
void cllbck_tim_icp_1_hz(const ros::TimerEvent& event);

void draw_graph();
void draw_true_values();
int y_tengah(int y);
int y_tengah_1(int y);

void draw_velocity();  

void kalman_filter();
void kalman_filter_2();

//==============================================================================

ros::Timer tim_50_hz;
ros::Timer tim_odo_1_hz;
ros::Timer tim_icp_1_hz;

//==============================================================================

vector<vector<double_t>> data_raw;

double odo_x = 0;
static int odo_cnt = 0;

double icp_x = 0;
static int icp_cnt = 0;

double icp_score = 0;

cv::Mat blank(800, 1200, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Point point_odo_before = cv::Point(0, blank.rows / 2);
cv::Point point_icp_before = cv::Point(0, blank.rows / 2);

cv::Mat blank_true_value(800, 800, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Point point_dumm_before = cv::Point(0, blank_true_value.rows);

cv::Point point_kf_before = cv::Point(0, blank.rows / 2);

//==============================================================================

float vel_x = 0;

cv::Mat blank_velocity(800, 1200, CV_8UC3, cv::Scalar(0, 0, 0));
cv::Point point_vx_before = cv::Point(0, blank_velocity.rows / 2);

//==============================================================================

KalmanFilter kf;
KalmanFilterType1 kf1;
KalmanFilterType2 kf2;
KalmanFilterType3 kf3;
KalmanFilterType4 kf4;
KalmanFilterType5 kf5;
KalmanFilterType6 kf6;

DataDummy dd;

//==============================================================================

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kalman_satu");

    ros::NodeHandle NH;
    ros::AsyncSpinner AS(0);

    tim_50_hz = NH.createTimer(ros::Duration(0.05), cllbck_tim_50_hz);
    tim_odo_1_hz = NH.createTimer(ros::Duration(0.05), cllbck_tim_odo_1_hz);
    tim_icp_1_hz = NH.createTimer(ros::Duration(0.05), cllbck_tim_icp_1_hz);

    if(kalman_satu_init() == -1)
        ros::shutdown();

    AS.start();
    ros::waitForShutdown();

    return 0;
    
}

//==============================================================================

void cllbck_tim_50_hz(const ros::TimerEvent& event)
{
    draw_velocity();
    draw_graph();
}

void cllbck_tim_odo_1_hz(const ros::TimerEvent& event)
{
    if(odo_cnt < data_raw.size())
    {
        odo_x = data_raw[odo_cnt][0];
        odo_cnt++;

        // generate vx
        if(odo_cnt > 1)
            vel_x = (data_raw[odo_cnt][0] - data_raw[odo_cnt - 1][0]) / 0.05;
    }
    else
        odo_x = 0;

    
}

void cllbck_tim_icp_1_hz(const ros::TimerEvent& event)
{
    if(icp_cnt < data_raw.size())
    {
        icp_x = data_raw[icp_cnt][3];
        icp_score = data_raw[icp_cnt][6];
        icp_cnt++;
    }
    else
        icp_x = 0;
}

//==============================================================================

int kalman_satu_init()
{
    // get ros package path
    string ros_pkg_path = ros::package::getPath("kalman_satu");

    // get data path
    string data_path = ros_pkg_path + "/data";

    // load csv
    fstream file(data_path + "/recordICP.csv", ios::in);
    if (!file.is_open())
    {
        cout << "File not found" << endl;
        return -1;
    }

    //===================================
    
    string line, collumn;

    while (getline(file, line))
    {
        vector<double_t> data_row;
        stringstream ss(line);

        // get data if not csv header
        if (line[0] != 'x')
        {
            while (getline(ss, collumn, ','))
            {
                data_row.push_back(stod(collumn));
            }
            data_raw.push_back(data_row);
        }
    }

    //===================================

    file.close();

    //===================================

    // kalman_filter();
    // draw_true_values();

    //===================================

    // kf.init(400, 10, 0, 0.01, 50, 25);
    // kf.predict();

    // kf1.init(350);

    // kf2.init(350, 10, 0.05);

    // kf3.init(350, 10, 0, 0.05);

    // kf4.init(350, 500, 1);

    // kf5.init(350, 100, 10, 0.5);

    MatrixXd _x(3, 1);
    _x << 350, 100, 0;

    MatrixXd _p(3, 3);
    _p << 100, 0, 0,
          0, 100, 0,
          0, 0, 100;

    kf6.init(0.05, 0.001, 50, _x, _p);

    return 0;
}

void draw_velocity()
{
    // draw velocity
    cv::Point point_vx_now = cv::Point(odo_cnt * 2, y_tengah_1(vel_x));
    cv::line(blank_velocity, point_vx_before, point_vx_now, cv::Scalar(0, 0, 255), 1, 8, 0);

    cv::line(blank_velocity, cv::Point(0, blank_velocity.rows / 2), cv::Point(blank_velocity.cols, blank_velocity.rows / 2), cv::Scalar(255, 255, 255), 1, 8, 0);

    point_vx_before = point_vx_now;

    cv::imshow("velocity", blank_velocity);
    cv::waitKey(1);
}

void draw_graph()
{
    if(odo_x != 0 and icp_x != 0)
    {
        if(odo_cnt <= data_raw.size())
        {
            // save point now
            cv::Point point_odo_now = cv::Point(odo_cnt * 2, y_tengah(odo_x));
            cv::Point point_icp_now = cv::Point(odo_cnt * 2, y_tengah(icp_x));

            // kalman filter
            // kf.measure(odo_x);
            // cv::Point point_kf_now = cv::Point(odo_cnt * 10, int(y_tengah(kf.s_now)));
            // kf.update();

            // kalman filter 1
            // kf5.update(odo_x);
            // kf5.update(icp_x);


            // kf6.update(odo_x);
            // kf6.update(icp_x);
            if(odo_cnt % 10 == 0 && icp_score < 500 )
            {
                kf6.update(icp_x, 0);

                // draw point now
                cv::Point point_icp_good = cv::Point(odo_cnt * 2, int(y_tengah(kf6.X_est.coeff(0, 0))));
                cv::circle(blank, point_icp_good, 10, cv::Scalar(255, 255, 0), -1);
            }

            kf6.update(vel_x, 1);


            cv::Point point_kf_now = cv::Point(odo_cnt * 2, int(y_tengah(kf6.X_est.coeff(0, 0))));
 
            // cv::line(blank, point_odo_before, point_odo_now, cv::Scalar(0, 0, 255), 2, 8, 0);
            cv::line(blank, point_icp_before, point_icp_now, cv::Scalar(0, 255, 0), 2, 8, 0);
            cv::line(blank, point_kf_before, point_kf_now, cv::Scalar(255, 0, 0), 2, 8, 0);

            cv::line(blank, cv::Point(0, blank.rows / 2), cv::Point(blank.cols, blank.rows / 2), cv::Scalar(255, 255, 255), 1, 8, 0);

            // save point before
            point_odo_before = point_odo_now;
            point_icp_before = point_icp_now;
            point_kf_before = point_kf_now;

            cv::imshow("kalman_satu", blank);
            cv::waitKey(1);
        }
    }
}

void draw_true_values()
{
    for(int i = 0; i < 10; i++)
    {
        cv::Point point_now = cv::Point(i*50, y_tengah(dd.get_position(i)/10));

        // cout << dd.get_position(i)/10 << endl;

        if(i != 0)
            cv::line(blank_true_value, point_dumm_before, point_now, cv::Scalar(0, 0, 255), 2, 8, 0);

        point_dumm_before = point_now;

        cv::imshow("kalman_satu_true", blank_true_value);
        cv::waitKey(1);
    }
}

void kalman_filter()
{
    int measurement[10] = {30211, 30453, 30906, 30999, 31368, 31978, 32526, 33379, 34698, 36275};

    kf.init(30000, 50, 0, 5, 225, 25);
    kf.predict();

    for(int i = 0; i < 10; i++)
    {
        kf.measure(measurement[i]);
        kf.update();
        
        cv::Point point_now = cv::Point(i*50, y_tengah(int(kf.s_now/100)));

        cout << int(kf.s_now/10) << endl;
        // cout << "kf now " << i << " " << kf.s_now << endl;

        // cout point
        // cout << point_now << endl;
        if(i != 0)
            cv::line(blank_true_value, point_kf_before, point_now, cv::Scalar(0, 255, 0), 2, 8, 0);

        point_kf_before = point_now;

        kf.predict();
    }    

}

void kalman_filter_2()
{

}

int y_tengah(int y)
{
    // get y tengah
    return (blank_true_value.rows) - (y);
}

int y_tengah_1(int y)
{
    return (blank_velocity.rows / 2) - (y);   
}

