//#define _CRT_SECURE_NO_WARNINGS
//#define _USE_MATH_DEFINES
//#include <iostream>
//#include <cmath>
#include<opencv2/opencv.hpp>
#include<ros/ros.h>
using namespace std;
using namespace cv;
string win_src1 = "src";
string win_dst1 = "dst";
int main()
{
	string file_src = "droneziku.png";
	string file_dst = "droneziku1.png";
	Mat img_src1 = imread(file_src, 1);

	Mat img_dst1;
	if (!img_src1.data) {
		cout << "error" << endl;
		return -1;
	}

	namedWindow(win_src1, WINDOW_AUTOSIZE);
	namedWindow(win_dst1, WINDOW_AUTOSIZE);


	vector<Mat> img_bgr(3);
	split(img_src1, img_bgr);
	
	merge(vector<Mat>{img_bgr[2], img_bgr[0], img_bgr[1]}, img_dst1);



	imshow(win_src1, img_src1);
	imshow(win_dst1, img_dst1);
	imwrite(file_dst, img_dst1);

	waitKey(0);
	return 0;
}
