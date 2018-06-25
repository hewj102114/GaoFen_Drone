#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;


class NumberDetect{
    public:
    NumberDetect();
    void detect(Mat&,Mat&);
    void colorExteact(Mat&,Mat&);
};


