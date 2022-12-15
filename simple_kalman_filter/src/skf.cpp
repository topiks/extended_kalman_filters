#include <iostream>
#include "skf.h"
#include "opencv2/opencv.hpp"

int main() {
    SimpleKalmanFilter skf(5, 5, 0.09);
    
    cv::Mat graph = cv::Mat::zeros(500, 500, CV_8UC3);
    cv::Mat graph2 = cv::Mat::zeros(500, 500, CV_8UC3);

    cv::Point2f last_point(0, 250);
    cv::Point2f last_point2(0, 250);

    for (int i = 0; i < 500; i++) {
        double x = i;
        double y = 250 + 100 * sin(i / 10.0) + 10 * rand() / 100000000;
        
        cv::Point2f point(x, y);
        cv::line(graph, last_point, point, cv::Scalar(0, 0, 255));
        last_point = point;

        double y2 = skf.update_estimate(y);
        cv::Point2f point2(x, y2);
        cv::line(graph2, last_point2, point2, cv::Scalar(0, 255, 0));
        last_point2 = point2;
    }

    cv::imshow("graph", graph);
    cv::imshow("graph2", graph2);
    cv::waitKey(0);


    return 0;
}