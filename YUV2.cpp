#include<ros/ros.h>
#include<cv.h>
#include<cxcore.h>
#include<highgui.h>
#include<opencv2/opencv.hpp>

#define  DEFAULT_Y 50    //0~255
#define  DEFAULT_U 127     //-128~127
#define  DEFAULT_V 127     //-128~127

int main(int argc, char **argv){
    char windowNameSource[]= "Source";
    char windowNameYUV[]="YUV";

    IplImage *srcImage = cvLoadImage("image/YUV/59232.jpg",CV_LOAD_IMAGE_ANYDEPTH |CV_LOAD_IMAGE_ANYCOLOR);

    if(srcImage == NULL){
        ROS_INFO("NO IMAGE");//printと秒数表示
        return -1;
    }
    CvSize sizeOfImage = cvGetSize(srcImage);

    IplImage *yuvImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,3);
    IplImage *YImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,1);
    IplImage *UImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,1);
    IplImage *VImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,1);

    IplImage *mergeImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,3);
    IplImage *dstImage = cvCreateImage(sizeOfImage, IPL_DEPTH_8U,3);

    CvScalar valueY = cvScalar(DEFAULT_Y);
    CvScalar valueU = cvScalar(DEFAULT_U);
    CvScalar valueV = cvScalar(DEFAULT_V);

    cvNamedWindow(windowNameSource, CV_WINDOW_AUTOSIZE);
    cvNamedWindow(windowNameYUV, CV_WINDOW_AUTOSIZE);

    cvCvtColor(srcImage,yuvImage,CV_BGR2YUV);

    cvSplit(yuvImage,YImage,UImage,VImage,NULL);

    std::cout <<"Y="<<YImage<< std::endl;//printf
    std::cout <<"U="<<UImage<< std::endl;//printf
    std::cout <<"V="<<VImage<< std::endl;//printf

    //cvSet(YImage,valueY,NULL);
    cvSet(UImage,valueU,NULL);
    cvSet(VImage,valueV,NULL);

    cvMerge(YImage,UImage,VImage,NULL,mergeImage);
    cvCvtColor(mergeImage,dstImage,CV_YUV2BGR);

    cvShowImage(windowNameSource, srcImage);
    cvShowImage(windowNameYUV,dstImage);

    cvWaitKey(0);

    cvReleaseImage(&srcImage);
    cvReleaseImage(&yuvImage);
    cvReleaseImage(&YImage);
    cvReleaseImage(&UImage);
    cvReleaseImage(&VImage);
    cvReleaseImage(&mergeImage);
    cvReleaseImage(&dstImage);

    cvDestroyWindow(windowNameSource);
    cvDestroyWindow(windowNameYUV);







}

