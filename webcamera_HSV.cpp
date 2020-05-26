#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;
string windoName = "src";
string hsv_window = "hsv";

//dynamic Reconfigure
    double MAX_HUE;
    double MIN_HUE;
    double MAX_SAT;
    double MIN_SAT;
    double MAX_VAL;
    double MIN_VAL;


/*int main(void)
{
  // このなかにプログラムを書いていきます
  VideoCapture capture(0);
// カメラが使えない場合はプログラムを止める
if(!capture.isOpened())
    return -1;


uchar hue, sat, val; // Hue, Saturation, Valueを表現する変数
Mat src_video(Size(640,480),CV_8UC1,Scalar::all(255)); // サイズを指定する
Mat smooth_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // ノイズを除去した映像を保存する
Mat hsv_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // HSVに変換した映像を保存する
Mat frame(Size(640, 480), CV_8UC1, Scalar::all(255));
Mat dst_img(Size(640, 480), CV_8UC1, Scalar::all(0)); // 認識結果を表示する

char windowName[] = "元映像";
namedWindow(windowName, CV_WINDOW_AUTOSIZE);
char hsvwindow[] = "HSV変換結果";
namedWindow(hsvwindow, CV_WINDOW_AUTOSIZE);
char dstwindow[] = "認識結果";
namedWindow(dstwindow, CV_WINDOW_AUTOSIZE);

while (cvWaitKey(1) == -1)
    {
        dst_img = Scalar(0, 0, 0); // この行を追加
        // カメラから1フレーム取得する
        capture >> frame;
        src_video = frame;
        imshow(windowName,src_video);

        medianBlur(src_video, smooth_video, 5);
        cvtColor(smooth_video, hsv_video, CV_BGR2HSV);
        imshow(hsv_window, hsv_video);

        for(int y = 0; y < hsv_video.rows; y++) {
        for (int x = 0; x < hsv_video.cols; x++) {
        hue = hsv_video.at<Vec3b>(y, x)[0];
        sat = hsv_video.at<Vec3b>(y, x)[1];
        val = hsv_video.at<Vec3b>(y, x)[2];
        // 居留地マップの検出
        /*if ((hue < 360 && hue > 0) && sat > 200) {
            dst_img.at<uchar>(y, x) = 255;}
        else {
            dst_img.at<uchar>(y, x) = 0;
        }

        //白(明度:255〜200)を抽出
        if (val < 255 && val > 200) {
            dst_img.at<uchar>(y, x) = 255;}
        else {
            dst_img.at<uchar>(y, x) = 0;
        }
    }
}
imshow(dstwindow,dst_img);
//imshow("認識結果",dst_img);
    }
destroyAllWindows();
return 0;    
}*/

void callback(){
     // このなかにプログラムを書いていきます
  VideoCapture capture(0);
// カメラが使えない場合はプログラムを止める
if(!capture.isOpened())
    return -1;


uchar hue, sat, val; // Hue, Saturation, Valueを表現する変数
Mat src_video(Size(640,480),CV_8UC1,Scalar::all(255)); // サイズを指定する
Mat smooth_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // ノイズを除去した映像を保存する
Mat hsv_video(Size(640, 480), CV_8UC1, Scalar::all(255)); // HSVに変換した映像を保存する
Mat frame(Size(640, 480), CV_8UC1, Scalar::all(255));
Mat dst_img(Size(640, 480), CV_8UC1, Scalar::all(0)); // 認識結果を表示する

char windowName[] = "元映像";
namedWindow(windowName, CV_WINDOW_AUTOSIZE);
char hsvwindow[] = "HSV変換結果";
namedWindow(hsvwindow, CV_WINDOW_AUTOSIZE);
char dstwindow[] = "認識結果";
namedWindow(dstwindow, CV_WINDOW_AUTOSIZE);

while (cvWaitKey(1) == -1)
    {
        dst_img = Scalar(0, 0, 0); // この行を追加
        // カメラから1フレーム取得する
        capture >> frame;
        src_video = frame;
        imshow(windowName,src_video);

        medianBlur(src_video, smooth_video, 5);
        cvtColor(smooth_video, hsv_video, CV_BGR2HSV);
        imshow(hsv_window, hsv_video);

        for(int y = 0; y < hsv_video.rows; y++) {
        for (int x = 0; x < hsv_video.cols; x++) {
        hue = hsv_video.at<Vec3b>(y, x)[0];
        sat = hsv_video.at<Vec3b>(y, x)[1];
        val = hsv_video.at<Vec3b>(y, x)[2];
        // 居留地マップの検出
        /*if ((hue < 360 && hue > 0) && sat > 200) {
            dst_img.at<uchar>(y, x) = 255;}
        else {
            dst_img.at<uchar>(y, x) = 0;
        }*/

        //白(明度:255〜200)を抽出
        if (val < 255 && val > 200) {
            dst_img.at<uchar>(y, x) = 255;}
        else {
            dst_img.at<uchar>(y, x) = 0;
        }
    }
}
imshow(dstwindow,dst_img);
//imshow("認識結果",dst_img);
    }
destroyAllWindows

}



void dynamicParamsCB(OpenCV1::HSV_webcame &cfg, uint32_t level){
    MAX_HUE = cfg.max_hue;
    MIN_HUE = cfg.min_hue;
    MAX_SAT = cfg.max_sat;
    MIN_SAT = cfg.min_sat;
    MAX_VAL = cfg.max_val;
    MIN_VAL = cfg.min_val;
    }

int main(){
ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録


    dynamic_reconfigure::Server<OpenCV1::Depth_pclConfig> drs_;
    drs_.setCallback(boost::bind(&dynamicParamsCB, _1, _2));
    	
	ros::spin();
	
	return 0;
}
