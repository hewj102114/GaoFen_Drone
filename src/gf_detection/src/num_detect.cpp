#include "num_detect.hpp"
#define SHOW_IMAGE

NumberDetect::NumberDetect(ros::NodeHandle *_pnh) {
    int a = 0;
    pnh = _pnh;
}
void NumberDetect::detect(Mat &raw_img_rgb, Mat &raw_img_depth) {
    Mat img_threshold, img_depth_8u, img_fusion;
    //深度图转换
    raw_img_depth.convertTo(img_depth_8u, CV_8U, 1);
    threshold(img_depth_8u, img_depth_8u, 200, 255, CV_THRESH_BINARY_INV);
    // RGB颜色提取
    colorExteact(raw_img_rgb, img_threshold);
    //求与
    bitwise_and(img_threshold, img_depth_8u, img_fusion);
    imshow("dep", img_depth_8u);
    //轮廓提取
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(img_fusion, contours, hierarchy, CV_RETR_EXTERNAL,
                 CV_CHAIN_APPROX_NONE, Point(0, 0));

    Mat img_draw = raw_img_rgb.clone();

    //轮廓筛选  四边形拟合 approxPolyDP
    vector<vector<Point>> target_ploy;
    vector<vector<Point2i>>::const_iterator it = contours.begin();
    while (it != contours.end()) {
        if (it->size() < 60 || it->size() > 10000) {
            it++;
            continue;
        }
        RotatedRect rect = minAreaRect(*it);
        cout << "size" << it->size() << "   area "
             << rect.size.width * rect.size.height << endl;
        // drawContours(img_draw, contours, it - contours.begin(), Scalar(0), 4,
        // 8,
        //  hierarchy);
        vector<Point> hull, poly;
        convexHull(Mat(*it), hull, false);
        approxPolyDP(Mat(hull), poly, it->size() / 10, true);

        if (poly.size() == 4) {
            float avg_len = (getPointDistance(poly[0], poly[1]) +
                             getPointDistance(poly[1], poly[2]) +
                             getPointDistance(poly[2], poly[3]) +
                             getPointDistance(poly[3], poly[0])) *
                            1.0 / 4;
            if (abs(getPointDistance(poly[0], poly[2]) -
                    getPointDistance(poly[1], poly[3])) < avg_len / 2) {
                target_ploy.push_back(poly);
                line(img_draw, poly[0], poly[1], Scalar(255, 0, 0), 1);
                line(img_draw, poly[1], poly[2], Scalar(255, 0, 0), 1);
                line(img_draw, poly[2], poly[3], Scalar(255, 0, 0), 1);
                line(img_draw, poly[3], poly[0], Scalar(255, 0, 0), 1);

                // line(img_fusion, poly[0], poly[1], Scalar(255), 1);
                // line(img_fusion, poly[1], poly[2], Scalar(255), 1);
                // line(img_fusion, poly[2], poly[3], Scalar(255), 1);
                // line(img_fusion, poly[3], poly[0], Scalar(255), 1);
            }
        }
        cout << "p" << hull.size() << "   " << poly.size() << endl;

        it++;
    }
    cout << "next  %d" << target_ploy.size() << endl;

    vector<vector<Point3f>> target_position(target_ploy.size());
    vector<vector<Point2f>> target_img_point(target_ploy.size());
    for (int i = 0; i < target_ploy.size(); i++) {
        for (int j = 0; j < 4; j++) {
            int ptx = target_ploy[i][j].x;
            int pty = target_ploy[i][j].y;
            float ptz = getDepthValue(raw_img_depth, target_ploy[i][j]);
            Point3f position;
            reprojectDepthImage(target_ploy[i][j], ptz, position);
            target_position[i].push_back(position);
            target_img_point[i].push_back(Point2f(ptx,pty));
            char str[30];
            sprintf(str, "%d, %d, %.1f", ptx, pty, ptz);
            putText(img_draw, str, Point(ptx, pty),
                    CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(255), 1);
            sprintf(str, "%.2f, %.2f, %.2f", position.x, position.y,
                    position.z);
            putText(img_draw, str, Point(ptx, pty + 20),
                    CV_FONT_HERSHEY_COMPLEX_SMALL, 0.5, Scalar(255), 1);
        }

        cout<<"ss"<<Mat(target_position[i])<<Mat(target_ploy[i])<<cam_matrix<<cam_distortion<<endl;
        Mat rvec,tvec;
        
        solvePnP(target_position[i], target_img_point[i], cam_matrix, cam_distortion, rvec, tvec);  
        cout<<rvec<<"                 "<<tvec<<endl;
    }

    imshow("dd", img_draw);
    imshow("ddd", img_fusion);
    waitKey(1);
    //对应位置深度图
    //深度解算
}
double NumberDetect::getPointDistance(Point &pt1, Point &pt2) {
    return sqrt(pow((pt1.x - pt2.x), 2) + pow((pt1.y - pt2.y), 2));
}
float NumberDetect::getDepthValue(Mat &img, Point &pt) {
    float ptz = img.at<float>(pt.x, pt.y);
    if (ptz == 255) {
        int sum = 0, count = 0;
        for (int m = 1; m < 5; m++) {
            for (int n = 1; n < 5; n++) {
                if (pt.x < m || pt.x > img.cols - m || pt.y < n ||
                    pt.y > img.rows - n)
                    continue;
                if (img.at<float>(pt.y + n, pt.x + m) != 255) {
                    sum += img.at<float>(pt.y + n, pt.x + m);
                    count++;
                }
                if (img.at<float>(pt.y - n, pt.x - m) != 255) {
                    sum += img.at<float>(pt.y - n, pt.x - m);
                    count++;
                }
            }
        }
        if (sum != 0) ptz = sum * 1.0 / count;
    }
    return ptz;
}
void NumberDetect::colorExteact(Mat &img, Mat &img_output) {
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
        if (gray > 120 && gray < 160 && (max - min) < 11)
            *ptr_img_gray = gray;
        else
            *ptr_img_gray = 0;

        ptr_img_gray++;
    }
    img_output = img_gray.clone();
    imshow("gray", img_output);
    waitKey(1);
}

void NumberDetect::reprojectDepthImage(Point &pt, float depth,
                                       Point3f &position) {
    Point center(cam_matrix.at<double>(0,2), cam_matrix.at<double>(1,2));
    float focal_x = cam_matrix.at<double>(0,0);
    float focal_y = cam_matrix.at<double>(1,1);
    
    float u = pt.x - center.x;
    float v = pt.y - center.y;
    double k = sqrt(1 + pow((u / focal_x), 2) + pow((v / focal_y), 2));

    position.z = depth / k;
    position.x = 1.0 * position.z * u / focal_x;
    position.y = -1.0 * position.z * v / focal_y;
}

void NumberDetect::setCamera(Mat& _matrix,Mat& _dist){
    cam_matrix=_matrix.clone();
    cam_distortion=_dist.clone();
}