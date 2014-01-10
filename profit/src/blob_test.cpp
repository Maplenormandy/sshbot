#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace cv;

void detectObjects(Mat& frame);
int key=-1;

int main( int argc, char** argv )
{
    VideoCapture cap(1);
    if (!cap.isOpened())
        return -1;

    namedWindow("Capture", CV_WINDOW_AUTOSIZE);
    namedWindow("Channel", CV_WINDOW_AUTOSIZE);

    while(key!='q') {
        Mat frame;
        cap >> frame;

        // detect objects and display video
        detectObjects (frame);

        if (waitKey(30) > 0) break;
    }

    return 0;
}

void detectObjects(Mat& frame)
{
    Mat broken;
    Mat src_Y(frame.rows, frame.cols, CV_8UC1);
    Mat src_Cr(frame.rows, frame.cols, CV_8UC1);
    Mat src_Cb(frame.rows, frame.cols, CV_8UC1);
    Mat out[] = {src_Y, src_Cr, src_Cb};
    Mat test;

    // std::cout << "cvtColor" << std::endl;

    // convert video image color
    cvtColor(frame, broken, CV_BGR2YCrCb);

    // std::cout << "mixChannels" << std::endl;

    int from_to[] = {0,0, 1,1, 2,2};
    mixChannels(&broken, 1, out, 3, from_to, 3);

    GaussianBlur(src_Cr, test, Size(9, 9), 2, 2);

    vector<Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( test, circles, CV_HOUGH_GRADIENT, 1, 20, 50, 40, 0, 0);

    /// Draw the circles detected
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( frame, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( frame, center, radius, Scalar(255,0,0), 3, 8, 0 );
    }

    imshow("Capture", frame);
    imshow("Channel", test);
}

