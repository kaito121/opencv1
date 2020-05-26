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
string file_src = "image/59240.jpg";//元画像
string file_dst = "image/59240_2tika.jpg";//二値化
string file_dst2 = "image/59240after2_2tika.jpg";//二値化エッジ抽出画像
Mat img_src = imread(file_src, 1);

Mat img_dst;
Mat img_dst2;
if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);

//二値化
int thresh =100;
threshold(img_src,img_dst,thresh,255,THRESH_BINARY);


//エッジ抽出
Mat img_tmp;

Sobel(img_dst, img_tmp, CV_32F, 0, 1, 3);
convertScaleAbs(img_tmp, img_dst2, 1, 0);



imshow(win_src, img_src);
imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);

waitKey(0);
return 0;
}
