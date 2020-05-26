#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
string windoName = "src";
string yuv_window = "YUV";

// 抽出する画像の輝度値の範囲を指定
#define Y_MAX 255       //255
#define Y_MIN 0         //0
#define U_MAX 127       //127  
#define U_MIN -128       //-128
#define V_MAX 127       //127
#define V_MIN -128       //-128


int main(void)
{
  // このなかにプログラムを書いていきます
  VideoCapture capture(0);
// カメラが使えない場合はプログラムを止める
if(!capture.isOpened())
    return -1;


uchar Y, U, V; // Hue, Saturation, Valueを表現する変数
Mat src_video(Size(640,480),CV_8UC1,Scalar::all(255)); // サイズを指定する
Mat smooth_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // ノイズを除去した映像を保存する
Mat yuv_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // YUVに変換した映像を保存する
Mat frame(Size(640, 480), CV_8UC1, Scalar::all(255));
Mat dst_img(Size(640, 480), CV_8UC1, Scalar::all(0)); // 認識結果を表示する
Mat img_dst1(Size(640, 480), CV_8UC1, Scalar::all(255));
Mat img_dst2(Size(640, 480), CV_8UC1, Scalar::all(255));
Mat img_mask(Size(640, 480), CV_8UC1, Scalar::all(255));

//Mat img_dst1;
//Mat img_dst2;
//Mat img_mask;


char windowName[] = "元映像";
namedWindow(windowName, CV_WINDOW_AUTOSIZE);
char yuvwindow[] = "YUV変換結果";
namedWindow(yuvwindow, CV_WINDOW_AUTOSIZE);
char dstwindow[] = "認識結果";
namedWindow(dstwindow, CV_WINDOW_AUTOSIZE);
char maskwindow[] = "マスク画像";
namedWindow(maskwindow, CV_WINDOW_AUTOSIZE);

while (cvWaitKey(1) == -1)
    {
        dst_img = Scalar(0, 0, 0); // この行を追加
        // カメラから1フレーム取得する
        capture >> frame;
        src_video = frame;
        imshow(windowName,src_video);

        medianBlur(src_video, smooth_video, 5);
        cvtColor(smooth_video, yuv_video, CV_BGR2YUV);
        imshow(yuv_window, yuv_video);//YUV画像表示

        for(int y = 0; y < yuv_video.rows; y++) {
        for (int x = 0; x < yuv_video.cols; x++) {
        Y = yuv_video.at<Vec3b>(y, x)[0];
        U = yuv_video.at<Vec3b>(y, x)[1];
        V = yuv_video.at<Vec3b>(y, x)[2];
        // 居留地マップの検出

        //
        if (Y < 255 && Y > 200) {
            dst_img.at<int>(y, x) = 255;}
        else {
            dst_img.at<int>(y, x) = 0;
        }
    //カラー抽出（白だけを抽出させる)

//    // inRangeを用いてフィルタリング
// 	  Scalar s_min = Scalar(Y_MIN, U_MIN, V_MIN);
// 	  Scalar s_max = Scalar(Y_MAX, U_MAX, V_MAX);
// 	  inRange(dst_img, s_min, s_max, img_dst1);

// 	// マスク画像を表示
//      namedWindow("mask");
// 	imshow(maskwindow, img_mask);
// 	//imwrite("image/59233after2_2tika_mask.jpg", img_dst3);

// 	// マスクを基に入力画像をフィルタリング
// 	img_dst1.copyTo(img_dst2, img_mask);
    }
}
imshow(dstwindow,dst_img);
//imshow("認識結果",dst_img);
    }
destroyAllWindows();
return 0;    
}
