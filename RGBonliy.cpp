#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 100
#define G_MAX 255
#define G_MIN 210
#define R_MAX 255
#define R_MIN 180
string win_src = "src";
string win_dst2 = "dst";

// メイン関数
int main()
{
string file_src = "image/59233after2_2tika.jpg";//元画像
string file_dst = "image/59233after2_2tika_mask.jpg";//mask
string file_dst2 = "image/59233after2_2tika_out.jpg";//output
string file_dst3 = "image/59233after2_2tika_out_UP.jpg";//output
Mat img_src = imread(file_src, 1);

Mat img_dst;
Mat img_dst2;
Mat img_dst3;
if (!img_src.data) {
cout << "error" << endl;
return -1;
}


	// 表示して確認
	namedWindow(win_src, WINDOW_AUTOSIZE);
	namedWindow(win_dst2, WINDOW_AUTOSIZE);

	// inRangeを用いてフィルタリング
	Scalar s_min = Scalar(B_MIN, G_MIN, R_MIN);
	Scalar s_max = Scalar(B_MAX, G_MAX, R_MAX);
	inRange(img_src, s_min, s_max, img_dst);

	// マスク画像を表示
	namedWindow("mask");
	//imshow("mask", img_dst);
	imwrite("image/59233after2_2tika_mask.jpg", img_dst);

	// マスクを基に入力画像をフィルタリング
	img_src.copyTo(img_dst2, img_dst);

	//膨張縮小処理(二回目)
	//Mat img_tmp;
	//Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1,1);
	//morphologyEx(img_dst2,img_tmp,MORPH_OPEN,element8,Point(-1,-1),1);
	//morphologyEx(img_tmp,img_dst2,MORPH_CLOSE,element8,Point(-1,-1),1);

	//膨張処理
	Mat element8 = (Mat_<uchar>(3,3) <<1,1,1,1,1,1,1,1,1);
	dilate(img_dst2,img_dst3,element8,Point(-1,-1),1);


	// 結果の表示と保存
	//imshow(win_src, img_src);
    //imshow(win_dst2, img_dst2);
	imwrite(file_dst, img_dst);
	imwrite(file_dst2, img_dst2);
	imwrite(file_dst3, img_dst3);

	waitKey(0);
	return 0;
}
