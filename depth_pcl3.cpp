//rosのヘッダ
#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>//reconfig用
#include <OpenCV1/Depth_pclConfig.h>
#include<mutex>
#include<OpenCV1/wakuhairetu.h>//自作メッセージ用ヘッダ
#include<algorithm>//並び替え用


ros::Subscriber sub;//データをsubcribeする奴
ros::Publisher  pub;
ros::Publisher  waku_pub;
std::string win_src = "src";//nameteigi
std::string win_dst = "dst";//nameteigi
std::string win_dst2 = "dst2";
std::string win_dst3 = "dst3";
std::string win_nitika = "nitika";
std::string win_edge = "edge";
std::string win_open = "open";
//std::string file_dst = "testopencv.png";

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
    double X_Y_pcl;
    double Z_pcl;
    double Z_Z=1000;
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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pointCloud->points.reserve(RGBimage.size().width*RGBimage.size().height);//点の数

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
	double siro=0,depth=0,Dmax=0,Dmin=0,kyori,kaisu,Dmax1,Dmin1,P1,zero;
    double positionA,positionB;
    int W=WINDOW_SIZE,H=WINDOW_SIZE;
    img_dst8=RGBimage.clone();//コピー作成
    img_dst9=img_dst8.clone();//コピー作成
	std::vector<std::vector<double>> A(RGBimage.size().width, std::vector<double>(RGBimage.size().height));//画像サイズ設定
    std::vector<std::vector<double>> T(RGBimage.size().width, std::vector<double>(RGBimage.size().height));
    std::vector<std::vector<double>> Q(RGBimage.size().width, std::vector<double>(RGBimage.size().height));

std::cout<<"test1"<<std::endl;

    //白の割合を調べる

    for(int x = 0;x < RGBimage.size().width-W;x=x+W){
        for(int y = 0;y < RGBimage.size().height-H;y=y+H){
            Mat cut_img(img_dst7,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            Mat cutdepth_img(depthimage,Rect(x,y,W,H)); // W*Hpx四方で切り抜く
            siro=0,kaisu=0,P1=0,zero=0;
            std::mutex mut1;
                    cut_img.forEach<unsigned char>([&](unsigned char &p, const int  position[]) -> void{
                        std::lock_guard<std::mutex> lock(mut1); //// ここ
                        if(p>=200){siro=siro+1;} });//白を検出(200以上なら白)
                        Dmax=-100;Dmin=1000;depth=0;//初期値代入のため
                        std::mutex mut2; //ここ
                    cutdepth_img.forEach<float>([&](float &p, const int  position[]) -> void{
                        std::lock_guard<std::mutex> lock(mut2);//ここ
                        kaisu=kaisu+1;P1=p/1000;
                        //Q[position[1]][position[0]]=p;
                        //if(kaisu==1){positionA=position[1];positionB=position[0];}
                        if(Dmax<=(p/1000)){Dmax=p/1000;}
                        if(Dmin>=(p/1000)){Dmin=p/1000;}
                        if((p/1000)<=0.2){zero=zero+1;}
                        depth=depth+p/1000;//ピクセル内の距離の合計
                        if(W*H-zero<=2){}
                        else{
                        if(kaisu==W*H){T[x][y]=(depth-(Dmax+Dmin))/(W*H-2-zero);}}//最大値と最小値を除いて平均化
                        //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<P1<< std::endl;//printf
                        //std::cout <<"["<<position[1]<<"]["<<position[0]<<"]=P="<<cutdepth_img.at<float>(position[0],position[1])<< std::endl;//printf
                     });
                     A[x][y]=(siro/(W*H))*100;//ピクセル内の白の割合
                      }}


                     //結果表示
   for(int x=0;x<RGBimage.size().width;x=x+W){
	 for(int y=0;y<RGBimage.size().height;y=y+H){
         
		 if(A[x][y]>=10 && T[x][y]!=0){                                           //白の割合が10%以上なら発動
             rectangle(img_dst8,Rect(x,y,W,H),Scalar(0,0,255),1);//赤四角作成
             pcl::PointXYZRGB jk;

             jk.x=(float)x/X_Y_wariai;
             jk.y=(float)y/X_Y_wariai;
             jk.z=(float)T[x][y];

             pointCloud->points.emplace_back(jk);//ポイントクラウドに座標データを移動
             std::cout <<"B["<<x<<"]["<<y<<"]="<<pointCloud->points.back().z<<"_m"<< std::endl;
         }
     }}

    //pointcloudサイズ設定
    pointCloud -> width = pointCloud -> points.size();
    pointCloud -> height = 1;
    pointCloud -> is_dense = true;

     // クラスタリングの設定
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (pointCloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
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
    double MAXPX,MINPX,MAXPY,MINPY,PZ,leftz,rightz,centerz,centerx;

    OpenCV1::wakuhairetu pclP;
    //ROS_INFO("いちばん");//printと秒数表示

     // クラスタリング の結果を色々できるところ(配列にアクセス)
    for (std::vector<pcl::PointIndices>::iterator it = indices.begin (); it != indices.end (); ++it){ // グループにアクセスするループ(it=グループ番号)
        std::sort(it->indices.begin (),it->indices.end (),[&] (int const& a,int const& b){//並び替え
        return pointCloud -> points[a].x > pointCloud -> points[b].x;//並び替えの条件
        });
    }
        //ROS_INFO("なか ");//printと秒数表示
    for (std::vector<pcl::PointIndices>::const_iterator it = indices.begin (); it != indices.end (); ++it){ 
        j=j+1; MAXPX=-10000,MINPX=10000,MAXPY=-10000,MINPY=10000;
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // グループ中の点にアクセスするループ(pit=グループ内の点番号)
            // 点へのアクセス
      
            pointCloud -> points[*pit].r = R[j%12];//ポイントクラウドのマスの色付け
            pointCloud -> points[*pit].g = G[j%12];
            pointCloud -> points[*pit].b = B[j%12];

            rectangle(img_dst9, Rect((int)(pointCloud -> points[*pit].x*X_Y_wariai), (int)(pointCloud -> points[*pit].y*X_Y_wariai),W,H), Scalar(B[j%12],G[j%12],R[j%12]),1);//四角作成
            //std::cout <<"X["<<pointCloud -> points[*pit].x<<"]["<<pointCloud -> points[*pit].y<<"]="<<pointCloud -> points[*pit].z<<"_m"<< std::endl;
            
            if(MAXPY<=(pointCloud -> points[*pit].y * X_Y_wariai)){MAXPY = pointCloud -> points[*pit].y * X_Y_wariai;}//yの最大値
            if(MINPY>=(pointCloud -> points[*pit].y * X_Y_wariai)){MINPY = pointCloud -> points[*pit].y * X_Y_wariai;}

            pointCloud -> points[*pit].x = ((pointCloud -> points[*pit].x*X_Y_wariai)-(RGBimage.size().width/2))/X_Y_pcl; //ピクセル座標からpcl上のX座標
            pointCloud -> points[*pit].y = ((pointCloud -> points[*pit].y*X_Y_wariai)-(RGBimage.size().height/2))/X_Y_pcl;//ピクセル座標からpcl上のY座標
            pointCloud -> points[*pit].z *= Z_pcl;

        }
        //ROS_INFO("おわり");//printと秒数表示
    //itの回数が大枠の個数
    //
        double CENPX,minix=1000;
         MINPX = (pointCloud -> points[*(it->indices.end ()-1)].x * X_Y_pcl +(RGBimage.size().width/2));//並び替えたので最大の値がXの最小値となる
         MAXPX = (pointCloud -> points[*(it->indices.begin ())].x * X_Y_pcl +(RGBimage.size().width/2));
         CENPX = ((pointCloud -> points[*(it->indices.end ()-1)].x+pointCloud -> points[*(it->indices.begin ())].x)/2);
         for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){ // 中心点にアクセスするループ(pit=グループ内の点番号
       
        if(minix>=std::abs(CENPX-pointCloud -> points[*pit].x)){minix = std::abs(CENPX-pointCloud -> points[*pit].x);
            centerx = (pointCloud -> points[*pit].x * X_Y_pcl +(RGBimage.size().width/2));
            centerz = pointCloud -> points[*pit].z;
            }
        }//X_Y_wariaiでかけるとピクセル値になる
         //centerx = (pointCloud -> points[(*(it->indices.end ()-1) + *it->indices.begin ())/2].x * X_Y_pcl +(RGBimage.size().width/2));
         rightz = pointCloud -> points[*(it->indices.end ()-1)].z;
         leftz = pointCloud -> points[*it->indices.begin ()].z;
         //centerz = pointCloud -> points[(*(it->indices.end ()-1) + *it->indices.begin ())/2].z;

    //std::cout <<"MINPX["<<MINPX<<"]  MINPY["<<MINPY<<"]"<< std::endl;
    //std::cout <<"MAXPX["<<MAXPX<<"]  MAXPY["<<MAXPY<<"]"<< std::endl;
    //ROS_INFO("おわりaaaaaaaaaaaaaaaaaaa");//printと秒数表示
    rectangle(img_dst9,Rect(MINPX,MINPY,MAXPX-MINPX+W,MAXPY-MINPY+H),Scalar(24,248,159),1.5);//四角作成
    putText(img_dst9,"No."+std::to_string(j-1),Point(MINPX,MINPY+30),0,0.5,Scalar(0,255,255),1);
    putText(img_dst9,"Left_Z="+std::to_string(leftz),Point(MINPX,MINPY+45),0,0.5,Scalar(0,255,255),1);
    putText(img_dst9,"center_z="+std::to_string(centerz),Point(MINPX,MINPY+60),0,0.5,Scalar(0,255,255),1);
    putText(img_dst9,"right_z="+std::to_string(rightz),Point(MINPX,MINPY+75),0,0.5,Scalar(0,255,255),1);
    line(img_dst9,Point(RGBimage.size().width/2,0),Point(RGBimage.size().width/2,RGBimage.size().height),Scalar(255,0,255),2);
    std::cout <<"Left_Z["<<j<<"]="<<leftz<< std::endl;
    std::cout <<"center_Z["<<j<<"]="<<centerz<< std::endl;
    std::cout <<"right_Z["<<j<<"]="<<rightz<< std::endl;
    std::cout <<"Left_X["<<j<<"]="<<MINPX<< std::endl;
    std::cout <<"CENPX["<<j<<"]="<<CENPX<< std::endl;
    std::cout <<"center_X["<<j<<"]="<<centerx<< std::endl;
    std::cout <<"Right_X["<<j<<"]="<<MAXPX<< std::endl;

    OpenCV1::waku waku;
    waku.point_left.x = (MINPX-(RGBimage.size().width/2))/X_Y_pcl;//送るデータ作成(大枠の右下と左下の座標)
    waku.point_left.y = (MAXPY-(RGBimage.size().height/2))/X_Y_pcl;
    waku.point_left.z = leftz;
    waku.point_right.x = (MAXPX-(RGBimage.size().width/2))/X_Y_pcl;
    waku.point_right.y = (MAXPY-(RGBimage.size().height/2))/X_Y_pcl;
    waku.point_right.z = rightz;
    waku.point_center.x = (centerx-(RGBimage.size().width/2))/X_Y_pcl;
    waku.point_center.z = centerz;

    pclP.pclP.emplace_back(waku);//配列に送るデータを追加
    }
    pclP.header.stamp = ros::Time::now();//トピックにパブリッシュした（データを送った
    waku_pub.publish(pclP);

    sensor_msgs::PointCloud2 depth_pcl;
        pcl::toROSMsg (*pointCloud, depth_pcl);
        depth_pcl.header.stamp = ros::Time::now();
        depth_pcl.header.frame_id = rgb_msg->header.frame_id;
        pub.publish(depth_pcl);

	//処理結果表示もここで行うこともできる（別関数でもあり
    cv::imshow(win_src, RGBimage);
	cv::imshow(win_dst, img_dst8);
    cv::imshow(win_dst2, img_dst7);
    cv::imshow(win_dst3, img_dst9);
    cv::imshow(win_edge, img_dst5);
    cv::imshow(win_nitika, img_dst6);
    cv::imshow(win_open, img_dst4);
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
    X_Y_pcl = cfg.X_Y_pcl;
    Z_pcl = cfg.Z_pcl;
    }

//メイン関数

int main(int argc,char **argv){
	ros::init(argc,argv,"opencv_main");
    	
	ros::NodeHandle nhSub;
    //sub設定(データ受け取り)
	message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nhSub, "/robot1/camera/color/image_raw", 1);
	message_filters::Subscriber<sensor_msgs::Image> depth_sub(nhSub, "/robot1/camera/depth/image_rect_raw", 1);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),rgb_sub, depth_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    //reconfigure設定
    dynamic_reconfigure::Server<OpenCV1::Depth_pclConfig> drs_;
    drs_.setCallback(boost::bind(&dynamicParamsCB, _1, _2));

    ros::NodeHandle nhPub;
    pub=nhPub.advertise<sensor_msgs::PointCloud2>("depth_pcl", 1000);
    waku_pub=nhPub.advertise<OpenCV1::wakuhairetu>("wakuhairetu", 1000);//パブリッシュ設定
    	
	ros::spin();
	
	return 0;
}


