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
string file_dst = "image/HSV_59232_2tika.jpg";//HSV化

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


//HSV化
cvtColor(img_src,img_dst,COLOR_BGR2HSV);

//二値化
int thresh =100;
threshold(img_dst,img_dst2,thresh,255,THRESH_BINARY);


imwrite(file_dst, img_dst2);


waitKey(0);
return 0;
}
