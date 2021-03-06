//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>
#include <OpenCV1/Depth_pclConfig.h>


ros::Subscriber sub;//データをsubcribeする奴
std::string win_src = "src";//nameteigi
std::string win_dst = "dst";//nameteigi
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";
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

//dynamic Reconfigure
    double CLUSTER_TOLERANCE;
    int MIN_CLUSTER_SIZE;
    int MAX_CLUSTER_SIZE;
    double X_Y_wariai;
    double WINDOW_SIZE;

//コールバック関数

void callback(const sensor_msgs::Image::ConstPtr& rgb_msg,const sensor_msgs::Image::ConstPtr& depth_msg)
{
	//変数宣言
	cv_bridge::CvImagePtr bridgeRGBImage;//クラス::型//cv_brigeは画像変換するとこ
    cv_bridge::CvImagePtr bridgedepthImage;//クラス::型//cv_brigeは画像変換するとこ
    cv::Mat RGBimage;//opencvの画像
    cv::Mat depthimage;//opencvの画像
	//ROS_INFO("callback_functionが呼ばれたよ");
	
    try{//MAT形式変換
       bridgeRGBImage=cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"RGB_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'BGR8'.",
        rgb_msg->encoding.c_str());
        return ;
    }

    try{//MAT形式変換
       bridgedepthImage=cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);//MAT形式に変える
       ROS_INFO("callBack");//printと秒数表示
    }
	//エラー処理
    catch(cv_bridge::Exception& e) {//エラー処理(失敗)成功ならスキップ
        std::cout<<"depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to '32FC1'.",
        depth_msg->encoding.c_str());
        return ;
    }
    

    RGBimage = bridgeRGBImage->image.clone();//image変数に変換した画像データを代入
    depthimage = bridgedepthImage->image.clone();//image変数に変換した画像データを代入

    // pointcloud を作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointCloud->points.reserve(RGBimage.size().width*RGBimage.size().height);

//ここに処理項目
	cv::namedWindow(win_src, cv::WINDOW_AUTOSIZE);
	cv::namedWindow(win_dst, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst2, cv::WINDOW_AUTOSIZE);
    cv::namedWindow(win_dst3, cv::WINDOW_AUTOSIZE);

	cv::Mat img_dst;
    cv::Mat img_dst1;
    cv::Mat img_dst2;
    cv::Mat img_dst3;
    cv::Mat img_dst4;
    cv::Mat img_dst5;
    cv::Mat img_dst6;
    cv::Mat img_dst7;
    cv::Mat img_dst8;
    cv::Mat img_dst9;

//グレースケール化
cvtColor(RGBimage,img_dst,COLOR_BGR2GRAY);

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
	double siro=0,depth=0,Dmax=0,Dmin=0,kyori,kaisu,Dmax1,Dmin1,P1;
    double positionA,positionB;
    int W=WINDOW_SIZE,H=WINDOW_SIZE;
    img_dst8=RGBimage.clone();//コピー作成
    img_dst9=img_dst8.clone();//コピー作成
    //画像サイズ設定
	std::vector<std::vector<double>> A(RGBimage.size().width, std::vector<double>(RGBimage.size().height));
    std::vector<std::vector<double>> T(RGBimage.size().width, std::vector<double>(RGBimage.size().height));
    std::vector<std::vector<double>> Q(RGBimage.size().width, std::vector<double>(RGBimage.size().height));

std::cout<<"test1"<<std::endl;

/*for(int x = 0;x < RGBimage.size().width;x++){
        for(int y = 0;y < RGBimage.size().height;y++){
std::cout <<"["<<x<<"]["<<y<<"]="<<depthimage.at<float>(y,x)<< std::endl;//printf
}}*/



    //白の割合を調べる

    for(int x = 0;x < RGBimage.size().width-W;x=x+W){
        for(int y = 0;y < RGBimage.size().height-H;y=y+H){
            Mat cut_img(img_dst7,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            Mat cutdepth_img(depthimage,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            siro=0,kaisu=0,P1=0;
                    cut_img.forEach<unsigned char>([&](unsigned char &p, const int  position[]) -> void{
                        if(p>=200){siro=siro+1;} });//白を検出(200以上なら白)
                        Dmax=-100;Dmin=1000;depth=0;//初期値代入のため
                    cutdepth_img.forEach<float>([&](float &p, const int  position[]) -> void{
                        kaisu=kaisu+1;P1=p/1000;
                        //Q[position[1]][position[0]]=p;
                        //if(kaisu==1){positionA=position[1];positionB=position[0];}
                        if(Dmax<=(p/1000)){Dmax=p/1000;}
                        if(Dmin>=(p/1000)){Dmin=p/1000;}
                        depth=depth+p/1000;//ピクセル内の距離の合計
                        if(kaisu==W*H){T[x][y]=(depth-(Dmax+Dmin))/(W*H-2);}//最大値と最小値を除いて平均化
                        //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<P1<< std::endl;//printf
                        //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<cutdepth_img.at<float>(position[0],position[1])<< std::endl;//printf
                     });
                     A[x][y]=(siro/(W*H))*100;//ピクセル内の白の割合
                      }}


                     //結果表示
   for(int x=0;x<RGBimage.size().width;x=x+W){
	 for(int y=0;y<RGBimage.size().height;y=y+H){
         
		 if(A[x][y]>=10){                                           //白の割合が10%以上なら発動
             rectangle(img_dst8,Rect(x,y,W,H),Scalar(0,0,255),1);//赤四角作成

             pointCloud->points.emplace_back(pcl::PointXYZ((float)x/X_Y_wariai,(float)y/X_Y_wariai,(float)T[x][y]));//ポイントクラウドに座標データを移動
             //std::cout <<"B["<<x<<"]["<<y<<"]="<<T[x][y]<<"_m"<< std::endl;}
         }
     }}

    //pointcloudサイズ設定
    pointCloud -> width = pointCloud -> points.size();
    pointCloud -> height = 1;
    pointCloud -> is_dense = true;

     // クラスタリングの設定
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pointCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (CLUSTER_TOLERANCE);//同じクラスタとみなす距離
  	ec.setMinClusterSize (MIN_CLUSTER_SIZE);//クラスタを構成する最小の点数
  	ec.setMaxClusterSize (MAX_CLUSTER_SIZE);//クラスタを構成する最大の点数
	ec.setSearchMethod (tree);//s探索方法
	ec.setInputCloud (pointCloud);//クラスタリングするポイントクラウドの指定

    // クラスタリング実行
    std::vector<pcl::PointIndices> indices;
	ec.extract (indices);//結果

    int R[12]={255,255,255,125,125,125,50,50,50,0,0,0};
    int G[12]={50,125,0,50,125,0,255,50,125,0,50,125};
    int B[12]={50,50,50,125,125,125,255,255,255,0,0,0};
    int j=0;
  
     // クラスタリング の結果を色々できるところ(配列にアクセス)
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){ // グループにアクセスするループ(it=グループ番号)
        j=j+1;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // グループ中の点にアクセスするループ(pit=グループ内の点番号)
            // 点へのアクセス
           // pointCloud -> points[*pit].x;
           // pointCloud -> points[*pit].y;
          //  pointCloud -> points[*pit].z;

            rectangle(img_dst9,Rect((int)(pointCloud -> points[*pit].x*X_Y_wariai),(int)(pointCloud -> points[*pit].y*X_Y_wariai),W,H),Scalar(B[j%12],G[j%12],R[j%12]),1);//四角作成


        }


    }



	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow(win_src, RGBimage);
	cv::imshow(win_dst, img_dst8);
    cv::imshow(win_dst2, img_dst7);
    cv::imshow(win_dst3, img_dst9);
	//cv::imwrite(file_dst, img_dst8);
	cv::waitKey(1);



   //ros::spinにジャンプする
   }

void dynamicParamsCB(OpenCV1::Depth_pclConfig &cfg, uint32_t level){
    CLUSTER_TOLERANCE = cfg.cluster_tolerance;
    MIN_CLUSTER_SIZE = cfg.min_cluster_size;
    MAX_CLUSTER_SIZE = cfg.max_cluster_size;
    X_Y_wariai = cfg.X_Y_wariai;
    WINDOW_SIZE=cfg.window_size;
    }

//メイン関数

int main(int argc,char **argv){
	ros::init(argc,argv,"opencv_main");
    	
	ros::NodeHandle nhSub;
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/camera/depth/image_rect_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    dynamic_reconfigure::Server<OpenCV1::Depth_pclConfig> drs_;
    drs_.setCallback(boost::bind(&dynamicParamsCB, _1, _2));
    	
	ros::spin();
	
	return 0;
}
