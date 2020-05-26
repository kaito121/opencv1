#include<ros/ros.h>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
string win_src = "src";
string win_dst = "dst";
// 抽出する画像の輝度値の範囲を指定
#define Y_MAX 255
#define Y_MIN 0
#define U_MAX 127
#define U_MIN -128
#define V_MAX 127
#define V_MIN -128


int main()
{
string file_src = "image/YUV/59232.jpg";//元画像
string file_dst = "image/YUV/59232_YUV.jpg";//二値化
string file_mask = "image/YUV/59232_YUV_mask.jpg";//二値化
string file_dst2 = "image/YUV/59232_YUV_masksyori.jpg";//二値化

Mat img_src = imread(file_src, 1);

Mat img_dst;
Mat img_dst2;
Mat img_mask;
Mat img_dst3;
Mat img_dst4;
Mat img_dst5;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);

cvtColor(img_src, img_dst, CV_BGR2YUV);


//カラー抽出（白だけを抽出させる)

// inRangeを用いてフィルタリング
	Scalar s_min = Scalar(Y_MIN+0, U_MIN+0, V_MIN+128);
	Scalar s_max = Scalar(Y_MAX-0, U_MAX-0, V_MAX-0);
	inRange(img_dst, s_min, s_max, img_mask);

	// マスク画像を表示
	namedWindow("mask");
	//imshow("mask", img_dst);
	//imwrite("image/59233after2_2tika_mask.jpg", img_dst3);

	// マスクを基に入力画像をフィルタリング
	img_dst.copyTo(img_dst2, img_mask);


//imshow(win_src, img_src);
//imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);
imwrite(file_mask, img_mask);
// imwrite(file_dst3, img_dst3);
// imwrite(file_dst4, img_dst4);
// imwrite(file_dst5, img_dst5);

waitKey(0);
return 0;
}
