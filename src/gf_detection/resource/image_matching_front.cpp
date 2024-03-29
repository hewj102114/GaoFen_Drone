#include <image_matching_front.hpp>
#include <opencv2/opencv.hpp>
#include <ros/package.h>



using namespace cv;
using namespace std;
#define SHOW_IMAGE
#define DATA_FREQ 100000
#define CALC_TIME_START ros::Time start = ros::Time::now();
#define CALC_TIME_END ROS_ERROR("TIME : %f FPS: %f", (ros::Time::now() - start).toSec(), 1.0 / (ros::Time::now() - start).toSec()); \
 start = ros::Time::now();

 int NS=0;

ImageMatchingFront::ImageMatchingFront() : it_(nh_)
{
  image_sub_ = it_.subscribe("/airsim/image/front/rgb", 1,
                             &ImageMatchingFront::imageCb, this);
  object_pub = nh_.advertise<gf_perception::ObjectList>("airsim/object/front", 1);
  campose_sub = nh_.subscribe("airsim/front_camera/pose_state", 1, &ImageMatchingFront::camposeCallback, this);
}

void ImageMatchingFront::camposeCallback(const std_msgs::Int16::ConstPtr& msg)
{
  campose=msg->data;
  
}
void ImageMatchingFront::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  gf_perception::ObjectList object_list_msg;

  imageProcess(cv_ptr->image, object_list_msg);

  
  object_list_msg.header.stamp=ros::Time::now();
  object_pub.publish(object_list_msg);
}

void ImageMatchingFront::imageProcess(cv::Mat &imgRaw, gf_perception::ObjectList &rect_list)
{
  if(campose==0)
  {
   ;
  }
  else if(campose==1)
  {
    Mat dst;  
    transpose(imgRaw, dst); 
    flip(dst, imgRaw, 1);
  }
  else if(campose==2)
  {
    Mat dst;  
    transpose(imgRaw, dst);
    flip(dst, imgRaw, 0); 
  }

  IplImage p = IplImage(imgRaw);
  IplImage *pimgRaw = &p;
  IplImage *input = cvCloneImage(pimgRaw);

  digitSquares(input, 300, 150000, rect_list);
}

//////////////////////////////////////////////////////////////////
//函数功能：采用多边形检测，通过约束条件寻找矩形
//输入：   img 原图像
//          storage 存储
//          minarea，maxarea 检测矩形的最小/最大面积
//////////////////////////////////////////////////////////////////
void ImageMatchingFront::digitSquares(IplImage *img, int minarea, int maxarea, gf_perception::ObjectList &rect_list)
{
  CvSeq *contours; //边缘
  int N = 6;       //阈值分级
  int n_square = 0;
  double x1, y1;
  CvSize sz = cvSize(img->width & -2, img->height & -2);
  IplImage *timg = cvCloneImage(img);                                       //拷贝一次img
  IplImage *gray = cvCreateImage(sz, 8, 1);                                 //img灰度图
  IplImage *pyr = cvCreateImage(cvSize(sz.width / 2, sz.height / 2), 8, 3); //金字塔滤波3通道图像中间变量
  IplImage *tgray = cvCreateImage(sz, 8, 1);
  CvMemStorage *storage = cvCreateMemStorage(0);
  CvSeq *result;

  cvSetImageROI(timg, cvRect(0, 0, sz.width, sz.height));
  //金字塔滤波
  cvPyrDown(timg, pyr, 7);
  cvPyrUp(pyr, timg, 7);
  //在3个通道中寻找矩形
  for (int c = 0; c < 3; c++) //对3个通道分别进行处理
  {
    cvSetImageCOI(timg, c + 1);
    cvCopy(timg, tgray, 0); //依次将BGR通道送入tgray
    for (int l = 0; l < N; l++)
    {
      //不同阈值下二值化
      cvThreshold(tgray, gray, (l + 1) * 255 / N, 255, CV_THRESH_BINARY);

      cvFindContours(gray, storage, &contours, sizeof(CvContour), CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0, 0));
      while (contours)
      { //多边形逼近
        result = cvApproxPoly(contours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, cvContourPerimeter(contours) * 0.02, 0);
        //如果是凸四边形并且面积在范围内
        if (result->total == 4 && fabs(cvContourArea(result, CV_WHOLE_SEQ)) > minarea && fabs(cvContourArea(result, CV_WHOLE_SEQ)) < maxarea && cvCheckContourConvexity(result))
        {
          CvPoint *pt0 = (CvPoint *)cvGetSeqElem(result, 0);
          CvPoint *pt1 = (CvPoint *)cvGetSeqElem(result, 1);
          CvPoint *pt2 = (CvPoint *)cvGetSeqElem(result, 2);
          CvPoint *pt3 = (CvPoint *)cvGetSeqElem(result, 3);


          if(getx(pt2)<getx(pt0))
          {
            pt0=(CvPoint*)cvGetSeqElem( result, 1 );
            pt1=(CvPoint*)cvGetSeqElem( result, 2 );
            pt2=(CvPoint*)cvGetSeqElem( result, 3 );
            pt3=(CvPoint*)cvGetSeqElem( result, 0 );
          }

          if (judgerect(pt0, pt1, pt2, pt3) == 1)
          {
            int num = t_rect(img, pt0, pt1, pt2, pt3);
            if (num != -1)
            {
              vector<Point> pt_list;
              pt_list.push_back(Point(pt0->x, pt0->y));
              pt_list.push_back(Point(pt1->x, pt1->y));
              pt_list.push_back(Point(pt2->x, pt2->y));
              pt_list.push_back(Point(pt3->x, pt3->y));
              RotatedRect rect = minAreaRect(pt_list);
              int repeat_count = 0;
              for (int i = 0; i < rect_list.object.size(); i++)
              {
                if (judgecenter(rect_list.object[i].center.x, rect_list.object[i].center.y, rect.center.x, rect.center.y) == 0)
                {
                  repeat_count++;
                }
              }
              if (repeat_count == 0)
              {
                rect_list.count++;
                gf_perception::Object object_msg;
                object_msg.number=num;
                object_msg.type=2;
                if(campose==0)
                {
                  object_msg.center.x=rect.center.x;
                  object_msg.center.y=rect.center.y;
                  object_msg.size.x=rect.size.width ;
                  object_msg.size.y=rect.size.height;
                }
                else if(campose==1)//右转
                {
                  object_msg.center.x=rect.center.y;
                  object_msg.center.y=480-rect.center.x;
                  object_msg.size.x=rect.size.height ;
                  object_msg.size.y=rect.size.width;
                }
                else if(campose==2)//左转
                {
                  object_msg.center.x=640-rect.center.y;
                  object_msg.center.y=rect.center.x;
                  object_msg.size.x=rect.size.height ;
                  object_msg.size.y=rect.size.width;
                }
                object_msg.size.z=rect.angle; 
                rect_list.object.push_back(object_msg);
                  #ifdef SHOW_IMAGE
                Scalar color(rand() % 255, rand() % 255, rand() % 255);
                cvLine(img, *pt0, *pt1, color, 2);
                cvLine(img, *pt1, *pt2, color, 2);
                cvLine(img, *pt2, *pt3, color, 2);
                cvLine(img, *pt3, *pt0, color, 2);
                #endif
              }
            }
          }
        }
        contours = contours->h_next;
      }
    }
  }
  #ifdef SHOW_IMAGE
  cvShowImage("Image window", img);
  cvWaitKey(3);
  #endif
  cvReleaseImage(&gray);
  cvReleaseImage(&pyr);
  cvReleaseImage(&tgray);
  cvReleaseImage(&timg);
  cvClearMemStorage(storage);
}

//////////////////////////////////////////////////////////////////
//函数功能：用向量来做COSα=两向量之积/两向量模的乘积求两条线段夹角
//输入：   线段3个点坐标pt1,pt2,pt0,最后一个参数为公共点
//输出：   线段夹角，单位为角度
//////////////////////////////////////////////////////////////////
double ImageMatchingFront::angle(CvPoint *pt1, CvPoint *pt2, CvPoint *pt0)
{
  double dx1 = pt1->x - pt0->x;
  double dy1 = pt1->y - pt0->y;
  double dx2 = pt2->x - pt0->x;
  double dy2 = pt2->y - pt0->y;
  double angle_line = (dx1 * dx2 + dy1 * dy2) / sqrt((dx1 * dx1 + dy1 * dy1) * (dx2 * dx2 + dy2 * dy2) + 1e-10); //余弦值
  return acos(angle_line) * 180 / 3.141592653;
}

//////////////////////////////////////////////////////////////////
//函数功能：输出矩形的长宽比
//输入：   线段3个点坐标pt0,pt1,pt2
//输出：   长宽比
//////////////////////////////////////////////////////////////////
int ImageMatchingFront::lentowid(CvPoint *pt0, CvPoint *pt1, CvPoint *pt2)
{
  int n = 0;
  double dx1 = pt0->x - pt1->x;
  double dy1 = pt0->y - pt1->y;
  double dx2 = pt2->x - pt1->x;
  double dy2 = pt2->y - pt1->y;
  double dx = sqrt(dx1 * dx1 + dy1 * dy1);
  double dy = sqrt(dx2 * dx2 + dy2 * dy2);
  double wh = dx / dy;
  if (fabs(wh - 0.6) < 0.2)
  {
    n = 1;
  }
  return n;
}
//////////////////////////////////////////////////////////////////
//函数功能：判断矩形是否重合
//////////////////////////////////////////////////////////////////
int ImageMatchingFront::judgecenter(double x1, double y1, double x2, double y2)
{
  int n = 1;
  if (fabs(x1 - x2) < 50 && fabs(y1 - y2) < 50)
  {
    n = 0;
  }
  return n;
}
//////////////////////////////////////////////////////////////////
//函数功能：判断是否为一个矩形
//////////////////////////////////////////////////////////////////
int ImageMatchingFront::judgerect(CvPoint *pt0, CvPoint *pt1, CvPoint *pt2, CvPoint *pt3)
{
  double t1, t2, t3;
  int n = 0;
  if (lentowid(pt0, pt1, pt2) == 1)
  {
    //判断每一条边
    t1 = fabs(angle(pt2, pt0, pt1));
    t2 = fabs(angle(pt3, pt1, pt2));
    t3 = fabs(angle(pt0, pt2, pt3));
    //这里的S为直角判定条件 单位为角度
    if (fabs(t1 + t2 - 180) < 10 && fabs(t1 - t3) < 10 && fabs(t1 - 90) < 10)
    {
      n = 1;
    }
  }
  return n;
}
//////////////////////////////////////////////////////////////////
//函数功能：返回一个点的x轴坐标
//////////////////////////////////////////////////////////////////
double ImageMatchingFront::getx(CvPoint *pt)
{
  return pt->x;
}
//////////////////////////////////////////////////////////////////
//函数功能：返回一个点的y轴坐标
//////////////////////////////////////////////////////////////////
double ImageMatchingFront::gety(CvPoint *pt)
{
  return pt->y;
}
//////////////////////////////////////////////////////////////////
//函数功能：校正矩形
//////////////////////////////////////////////////////////////////
int ImageMatchingFront::t_rect(IplImage *img, CvPoint *pt0, CvPoint *pt1, CvPoint *pt2, CvPoint *pt3)
{
  //NS++;
  IplImage *img_dst = cvCreateImage(cvSize(50, 50), IPL_DEPTH_8U, 3);
  CvPoint2D32f srcQuad[4], dstQuad[4];
  CvMat *warp_matrix = cvCreateMat(3, 3, CV_32FC1);

  srcQuad[0].x = pt0->x;
  srcQuad[0].y = pt0->y;
  srcQuad[1].x = pt3->x;
  srcQuad[1].y = pt3->y;
  srcQuad[2].x = pt2->x;
  srcQuad[2].y = pt2->y;
  srcQuad[3].x = pt1->x;
  srcQuad[3].y = pt1->y;

  dstQuad[0].x = 0;
  dstQuad[0].y = 0;
  dstQuad[1].x = 50 - 1;
  dstQuad[1].y = 0;
  dstQuad[2].x = 50 - 1;
  dstQuad[2].y = 50 - 1;
  dstQuad[3].x = 0;
  dstQuad[3].y = 50 - 1;

  //计算透视映射矩阵
  cvGetPerspectiveTransform(srcQuad, dstQuad, warp_matrix);
  //密集透视变换
  cvWarpPerspective(img, img_dst, warp_matrix);
  #ifdef SHOW_IMAGE
  cvShowImage("Image window2", img_dst);
  cvWaitKey(3);
  #endif

//   char ad[128]={0};
//   sprintf(ad, "/home/jjt-204u/gaofen/src/gf_detection/resource/num5/%d.jpg", NS);NS++;
//   cvSaveImage(ad, img_dst);

  Mat imgnum = cvarrToMat(img_dst,true);
  float num=hogsvm(imgnum);
  cout<<num<<endl;

//   int num = getDigit(img_dst, img, pt0, pt2);

  //cvShowImage("Image window",img);
  //cvWaitKey(3);
  // getDigit(img_dst,img,pt0,pt2);
  //std::cout<<n;

  cvReleaseImage(&img_dst);
  cvReleaseMat(&warp_matrix);
  return num;
}

int ImageMatchingFront::getDigit(IplImage *img, IplImage *imgsrc, CvPoint *pt0, CvPoint *pt2) //两张图片相减
{
  IplImage *src = cvCloneImage(img);
  
  uchar *tmp;
  int tmp1;
  int min = 1000000;
  int serieNum = -1;
  std::string resource_dir=ros::package::getPath("gf_detection");
  cout<<"############   "<<resource_dir<<endl;

//二值化
  IplImage *src_s = cvCreateImage(cvSize(50, 50), 8, 1);
  cvCvtColor(src, src_s, CV_BGR2GRAY);
  cvThreshold(src_s, src_s, 0, 255, CV_THRESH_OTSU);

  #ifdef SHOW_IMAGE
  cvShowImage("Image window2", src_s);
  cvWaitKey(3);
  #endif


//裁剪数字
  int wid=src_s->width;int heig=src_s->height;
  int ro[heig]={0};int co[wid]={0}; 
  int ct=0,ce=wid-1,rt=0,re=heig-1;

  for(int i=0;i<heig;i++)
  {
    for(int j=0;j<wid;j++)
    {
        uchar *tmp = (uchar *)(src_s->imageData + i * src_s->widthStep + j);
        tmp1 = *tmp;
        if(tmp1==0)
        {
            ro[i]++;
            co[j]++;
        }
    }
  }

  int h_heig=heig/2; int h_wid=wid/2;
  
  for(int i=h_heig;i>=0;i--)
  {
      if(ro[i]==0)
      {
        rt=i+1;
        break;
      }
  }
  for(int i=h_heig;i<=heig-1;i++)
  {
      if(ro[i]==0)
      {
        re=i-1;
        break;
      }
  }
  for(int i=h_wid;i>=0;i--)
  {
      if(co[i]==0)
      {
        ct=i+1;
        break;
      }
  }
  for(int i=h_wid;i<=wid-1;i++)
  {
      if(co[i]==0)
      {
        ce=i-1;
        break;
      }
  }

  if(ct<ce&&rt<re)
  {
      //裁剪
     cvSetImageROI(src_s,cvRect(ct,rt,ce-ct+1, re-rt+1));//设置源图像ROI
     IplImage *cutdst;
     CvSize size;size.width = 15; size.height = 30;
     cutdst = cvCreateImage(size, src_s->depth, src_s->nChannels);
     cvResize(src_s, cutdst, CV_INTER_CUBIC);
     cvResetImageROI(src_s);
     #ifdef SHOW_IMAGE
     cvShowImage("Image window3", cutdst);
     cvWaitKey(3);
     #endif

     // char ad[128]={0};
     // sprintf(ad, "/home/jjt-204u/gaofen/src/gf_detection/resource/num3/%d.jpg", NS);NS++;
     // cvSaveImage(ad, cutdst);


    //识别
    IplImage *result = cvCreateImage(cvSize(15, 30), 8, 1);
    for (int h = 4; h <= 9; h++)
    {
        char name[128];
        sprintf(name, "%s/resource/num4/%d.jpg",resource_dir.c_str(), h);
        IplImage *timg = cvLoadImage(name, CV_LOAD_IMAGE_GRAYSCALE);
        cvThreshold(timg, timg, 0, 255, CV_THRESH_OTSU);
        
        
        cvAbsDiff(timg, cutdst, result); //

        int diff = 0;
        for (int i = 0; i < result->height; i++)
        {
            for (int j = 0; j < result->width; j++)
            {
                uchar *tmp = (uchar *)(result->imageData + i * result->widthStep + j);
                tmp1 = *tmp;
                diff = diff + tmp1;
            }
        }

        if (diff < min)
        {
            min = diff;
            serieNum = h;
        }

        cvReleaseImage(&timg);
    }
    cvReleaseImage(&cutdst);
    cvReleaseImage(&result);

    if (min < 50000)
    {
        //printf("最小距离是%d ", min);
        printf(" 数字是%d  Center %f,%f\n", serieNum, (pt0->x + pt2->x) / 2.0, (pt0->y + pt2->y) / 2.0);

        char num[128];
        sprintf(num, "%d", serieNum);

        // CvFont font;
        // cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8);
        // cvPutText(imgsrc, num, cvPoint((pt0->x + pt2->x) / 2, (pt0->y + pt2->y) / 2), &font, CV_RGB(255, 0, 0));

        // cvShowImage("Image window",imgsrc);
        //cvWaitKey(3);
    }
    else
    {
        serieNum=-1;
    }
    
  }
  else
  {
      serieNum=-1;
  }
  
  cvReleaseImage(&src);
  cvReleaseImage(&src_s);
  
  return serieNum;
}

float ImageMatchingFront::hogsvm(Mat &img)
{
  resize(img,img,Size(64,64));

  Size win_size=Size(64,64);
  Size block_size=Size(8, 8);
  Size block_stride=Size(8, 8);
  Size cell_size=Size(8, 8);
  int nbins=9;
  double win_sigma=4;
  double threshold_L2hys=2.0000000000000001e-01;
  bool gamma_correction=false;
  int nlevels=64;
  HOGDescriptor hog(win_size,block_size,block_stride,cell_size,nbins,win_sigma,threshold_L2hys,gamma_correction,nlevels);

  vector<float> descriptors;
  hog.compute(img,descriptors);

  //Ptr<ml::SVM>svm = ml::SVM::load<ml::SVM>("../resource/model.xml");
  Ptr<ml::SVM> svm = ml::StatModel::load<ml::SVM>("/home/jjt-204u/gaofen/src/gf_detection/resource/model.xml");
  
  float res=svm->predict(descriptors);

//   svm.clear();
//   svm.load("../resource/model.xml");
//   int res=svm->predict(descriptors);

  return res;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_front");
  ImageMatchingFront ic;
  ros::spin();
  return 0;
}
