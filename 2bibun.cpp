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
// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180


int main()
{
string file_src = "image/59232.jpg";//元画像
string file_dst = "image/Q2_59232_2tika.jpg";//二値化
string file_dst2 = "image/Q2_59232after2_2tika.jpg";//二値化エッジ抽出画像
string file_dst3= "image/Q2_59232after2_2tika_mask.jpg";//mask画像
string file_dst4 = "image/Q2_59232after2_2tika_out.jpg";//カラーフィルタ処理画像
string file_dst5 = "image/Q2_59232after2_2tika_out_UP.jpg";//膨張処理画像
Mat img_src = imread(file_src, 1);

Mat img_dst;
Mat img_dst2;
Mat img_dst3;
Mat img_dst4;
Mat img_dst5;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);

//二値化
int thresh =100;
threshold(img_src,img_dst,thresh,255,THRESH_BINARY);

//膨張縮小処理(一回目)
Mat img_tmp;
Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
morphologyEx(img_dst,img_tmp,MORPH_OPEN,element8,Point(-1,-1),1);
morphologyEx(img_tmp,img_dst,MORPH_CLOSE,element8,Point(-1,-1),1);


//エッジ抽出(一次微分　y軸方向)柵
//Mat img_tmp1;
//Sobel(img_dst, img_tmp1, CV_32F, 1, 0, 3);
//convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//エッジ抽出(一次微分　X軸方向)階段
//Mat img_tmp1;
//Sobel(img_dst, img_tmp1, CV_32F, 0, 1, 3);
//convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_dst, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//カラー抽出（白だけを抽出させる)

// inRangeを用いてフィルタリング
	Scalar s_min = Scalar(B_MIN, G_MIN, R_MIN);
	Scalar s_max = Scalar(B_MAX, G_MAX, R_MAX);
	inRange(img_dst2, s_min, s_max, img_dst3);

	// マスク画像を表示
	namedWindow("mask");
	//imshow("mask", img_dst);
	//imwrite("image/59233after2_2tika_mask.jpg", img_dst3);

	// マスクを基に入力画像をフィルタリング
	img_dst2.copyTo(img_dst4, img_dst3);

	//膨張縮小処理(二回目)
	//Mat img_tmp;
	//Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1,1);
	//morphologyEx(img_dst2,img_tmp,MORPH_OPEN,element8,Point(-1,-1),1);
	//morphologyEx(img_tmp,img_dst2,MORPH_CLOSE,element8,Point(-1,-1),1);

	//膨張処理
	dilate(img_dst4,img_dst5,element8,Point(-1,-1),1);




//imshow(win_src, img_src);
//imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);
imwrite(file_dst3, img_dst3);
imwrite(file_dst4, img_dst4);
imwrite(file_dst5, img_dst5);

waitKey(0);
return 0;
}
