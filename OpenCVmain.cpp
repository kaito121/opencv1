//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//nameteigi
std::string win_dst = "dst";//nameteigi
std::string file_dst = "testopencv.png";

//コールバック関数
void callback_function(const sensor_msgs::Image::ConstPtr& msg)//画像トピックが更新されたらよばれるやつ//センサーデータがmsgに入る
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeImage;//クラス::型//cv_brigeは画像変換するとこ
	cv::Mat image;//opencvの画像
	ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeImage=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        msg->encoding.c_str());
        return ;
    }

    image = bridgeImage->image.clone();//image変数に変換した画像データを代入

	//ここに処理項目
	cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);

	cv::Mat img_tmp;
	cv::Mat img_dst;


	cv::Sobel(image, img_tmp, CV_32F, 1, 0, 3);
	cv::convertScaleAbs(img_tmp, img_dst, 1, 0);

	//処理結果表示もここで行うこともできる（別関数でもあり
        cv::imshow(win_src, image);
	cv::imshow(win_dst, img_dst);
	cv::imwrite(file_dst, img_dst);
	cv::waitKey(1);


	//


   //ros::spinにジャンプする
}

//メイン関数
int main(int argc,char **argv){

	ros::init(argc,argv,"opencv_main");//rosを初期化
	ros::NodeHandle nh;//ノードハンドル
	
	//subscriber関連
	sub=nh.subscribe("/camera/color/image_raw",1,callback_function);//取得するトピックデータを選択しコールバック関数に登録

	ros::spin();//トピック更新待機
			
	return 0;
}


