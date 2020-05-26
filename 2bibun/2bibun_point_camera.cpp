//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>

ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//nameteigi
std::string win_dst = "dst";//nameteigi
std::string win_dst2 = "dst2";
std::string file_dst = "testopencv.png";

using namespace std;
using namespace cv;

// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180

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
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);

	cv::Mat img_dst;
    cv::Mat img_dst1;
    cv::Mat img_dst2;
    cv::Mat img_dst3;
    cv::Mat img_dst4;
    cv::Mat img_dst5;
    cv::Mat img_dst6;
    cv::Mat img_dst7;
    cv::Mat img_dst8;

//グレースケール化
cvtColor(image,img_dst,COLOR_BGR2GRAY);

equalizeHist(img_dst,img_dst2);//ヒストグラム均一化

//コントラスト軽減
int min=0,max=200;
img_dst2.convertTo(img_dst3,img_dst2.type(),(max-min)/255.0,min);

//膨張縮小処理(一回目)
Mat img_tmp,img_ab,img_tmpp;
Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
morphologyEx(img_dst3,img_tmp,MORPH_CLOSE,element8,Point(-1,-1),4);
morphologyEx(img_tmp,img_ab,MORPH_OPEN,element8,Point(-1,-1),4);

morphologyEx(img_ab,img_tmpp,MORPH_OPEN,element8,Point(-1,-1),1);
morphologyEx(img_tmpp,img_dst4,MORPH_CLOSE,element8,Point(-1,-1),1);

//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_dst4, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst5, 1, 0);

//二値化
int thresh =100;
threshold(img_dst5,img_dst6,thresh,255,THRESH_BINARY);

//膨張処理
dilate(img_dst6,img_dst7,element8,Point(-1,-1),1);


//カラー抽出（白だけを抽出させる)
std::cout<<"test"<<std::endl;
	double siro=0;
    int W=10,H=10;
    img_dst8=image.clone();//コピー作成
    //画像サイズ設定
	std::vector<std::vector<double>> A(image.size().width, std::vector<double>(image.size().height));

std::cout<<"test1"<<std::endl;

    //白の割合を調べる
    for(int x = 0;x < image.size().width-W;x=x+W){
        for(int y = 0;y < image.size().height-H;y=y+H){
            Mat cut_img(img_dst7,Rect(x,y,W,H)); // 30px四方で切り抜く
            siro=0;
                    cut_img.forEach<unsigned char>([&](unsigned char &p, const int  position[]) -> void{
                        if(p>=200){siro=siro+1;} 
                     });
                     A[x][y]=(siro/(W*H))*100;
                     }  }
                     //結果表示
   for(int x=0;x<image.size().width;x=x+W){
	 for(int y=0;y<image.size().height;y=y+H){
		 if(A[x][y]>=10){rectangle(img_dst8,Rect(x,y,W,H),Scalar(0,0,255),1);}//青四角作成

		 std::cout <<A[x][y]<< std::endl;//printf
     }}



	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow(win_src, image);
	cv::imshow(win_dst, img_dst8);
    cv::imshow(win_dst2, img_dst7);
	cv::imwrite(file_dst, img_dst8);
	cv::waitKey(1);



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