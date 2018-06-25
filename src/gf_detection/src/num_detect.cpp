#include "num_detect.hpp"

NumberDetect::NumberDetect() { int a = 0; }
void NumberDetect::detect(Mat &raw_img_rgb, Mat &raw_img_depth) {
    Mat img_threshold;
    colorExteact(raw_img_rgb,img_threshold);
    //轮廓提取
    //轮廓筛选
    //四边形拟合 approxPolyDP
    //对应位置深度图
    //深度解算
    
}

void NumberDetect::colorExteact(Mat &img,Mat& img_output) {
    int img_size = img.cols * img.rows;
    const uchar *ptr_begin = img.data;
    const uchar *ptr_src = img.data;
    const uchar *ptr_src_end = img.data + img_size * 3;

    Mat img_gray(img.rows, img.cols, CV_8UC1);
    uchar *ptr_img_gray = img_gray.data;

    for (; ptr_src != ptr_src_end; ++ptr_src) {
        uchar max = 0, min = 255;
        uchar b = *ptr_src;
        if (b > max) max = b;
        if (b < min) min = b;

        uchar g = *(++ptr_src);
        if (g > max) max = g;
        if (g < min) min = g;

        uchar r = *(++ptr_src);
        if (r > max) max = r;
        if (r < min) min = r;

        uchar gray = (r * 30 + g * 59 + b * 11 + 50) / 100;
        if (gray>130 && gray<160 && (max-min)<11)
            *ptr_img_gray = 255;
        else
            *ptr_img_gray=0;

        ptr_img_gray++;
    }
    img_output=img_gray.clone();
    imshow("gray", img_gray);
    waitKey(1);
}