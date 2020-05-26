//#define _CRT_SECURE_NO_WARNINGS
//#define _USE_MATH_DEFINES
//#include <iostream>
//#include <cmath>
#include<ros/ros.h>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
string win_src = "src";
string win_dst = "dst";

int main()
{
string file_src = "image/59233.jpg";
string file_dst = "image/59233_2tika.jpg";
Mat img_src = imread(file_src, 1);

Mat img_dst;
if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);


int thresh =100;
threshold(img_src,img_dst,thresh,255,THRESH_BINARY);



imshow(win_src, img_src);
imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);

waitKey(0);
return 0;
}
