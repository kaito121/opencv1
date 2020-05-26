#define _CRT_SECURE_NO_WARNINGS
#include <iostream>
#include <vector>
#include<ros/ros.h>
#include<opencv2/opencv.hpp>

int main(){
    cv::Mat img_src1, img_src2, img_dst;
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;
  

    img_src1 = cv::imread("img/img_src2.jpg",0);
    img_src2 = cv::imread("img/img_src3.jpg",0);

    std::string file_dst = "img_dst1.jpg";

    cv::Ptr<cv::ORB> detector = cv::ORB::create();

    detector->detectAndCompute(img_src1, cv::noArray(), kpts1, desc1);
    detector->detectAndCompute(img_src2, cv::noArray(), kpts2, desc2);

    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce");
    std::vector<cv::DMatch> matches;
    matcher->match(desc1, desc2, matches);

    cv::drawMatches(img_src1, kpts1, img_src2, kpts2, matches, img_dst);
    cv::imshow("dst",img_dst);
    cv::imwrite(file_dst, img_dst);
    cv::waitKey(0);

    return 0;
}