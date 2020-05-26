#include <ros/ros.h>
#include <sensor_msgs/Image.h>//センサーデータ形式ヘッダ
#include <cv_bridge/cv_bridge.h>//画像変換のヘッダ
#include <sensor_msgs/image_encodings.h>//エンコードのためのヘッダ
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
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

ros::Publisher  laser_pub;

void callback(const OpenCV1::wakuhairetu::ConstPtr& waku_msg,const sensor_msgs::LaserScan::ConstPtr& laser_msg){
ROS_INFO("callBack1");
sensor_msgs::LaserScan laserScan_src = *laser_msg; //データ取り出し、変数定義
//waku_msg -> pclP[0].point_left.x;//pclPという名の配列の0番目のpoint_left.xの値
//waku_msg -> pclP[0].point_center.x;
//waku_msg -> pclP.size();//pclPの配列の個数(大枠の個数)
ROS_INFO("callBack");
std::cout <<"waku_size="<<waku_msg -> pclP.size()<<"_m"<< std::endl;
int i,p,renge,waku_size=waku_msg -> pclP.size();

// double left_l[waku_size],right_l[waku_size],left_s[waku_size],right_s[waku_size];
// double kosu[waku_size],henka_l[waku_size];

std::vector<double> left_l(waku_size);//リサイズ可能な配列を定義
std::vector<double> right_l(waku_size);
std::vector<double> center_l(waku_size);
std::vector<double> centerleft_l(waku_size);
std::vector<double> centerright_l(waku_size);
std::vector<double> left_s(waku_size);
std::vector<double> right_s(waku_size);
std::vector<double> center_s(waku_size);
std::vector<double> centerleft_s(waku_size);
std::vector<double> centerright_s(waku_size);
std::vector<int> kosuR1(waku_size);
std::vector<int> kosuR2(waku_size);
std::vector<int> kosuL1(waku_size);
std::vector<int> kosuL2(waku_size);
std::vector<double> henka_Ll_1(waku_size);
std::vector<double> henka_Ll_2(waku_size);
std::vector<double> henka_Rl_1(waku_size);
std::vector<double> henka_Rl_2(waku_size);
std::vector<std::vector<float>> LL(waku_size);//leftL
std::vector<std::vector<float>> LCL(waku_size);//LeftCenterL
std::vector<std::vector<float>> RL(waku_size);//RightL
std::vector<std::vector<float>> RCL(waku_size);//RightCenterL
std::vector<std::vector<double>> LS(waku_size);//
std::vector<std::vector<double>> LCS(waku_size);
std::vector<std::vector<double>> RS(waku_size);
std::vector<std::vector<double>> RCS(waku_size);

for(i=0;i<laserScan_src.ranges.size();i++){
  if(laserScan_src.ranges[i]==0){
   laserScan_src.ranges[i]=std::numeric_limits<float>::quiet_NaN();
   std::cout <<"i="<<i<< std::endl;
   }
}


for(p=0;p<waku_msg -> pclP.size();p++){//枠の個数

  left_l[p]=std::sqrt(waku_msg -> pclP[p].point_left.x * waku_msg -> pclP[p].point_left.x + waku_msg -> pclP[p].point_left.z * waku_msg -> pclP[p].point_left.z);//カメラからのLeftXの距離Lを求める
  right_l[p]=std::sqrt(waku_msg -> pclP[p].point_right.x * waku_msg -> pclP[p].point_right.x + waku_msg -> pclP[p].point_right.z * waku_msg -> pclP[p].point_right.z);//カメラからのRightXの距離を求める
  center_l[p]=std::sqrt(waku_msg -> pclP[p].point_center.x * waku_msg -> pclP[p].point_center.x + waku_msg -> pclP[p].point_center.z * waku_msg -> pclP[p].point_center.z);
  centerleft_l[p]=std::sqrt(waku_msg -> pclP[p].point_centerleft.x * waku_msg -> pclP[p].point_centerleft.x + waku_msg -> pclP[p].point_centerleft.z * waku_msg -> pclP[p].point_centerleft.z);
  centerright_l[p]=std::sqrt(waku_msg -> pclP[p].point_centerright.x * waku_msg -> pclP[p].point_centerright.x + waku_msg -> pclP[p].point_centerright.z * waku_msg -> pclP[p].point_centerright.z);
  left_s[p]=std::atan(waku_msg -> pclP[p].point_left.x/-waku_msg -> pclP[p].point_left.z);//カメラからLeftXまでの角度θを求める
  right_s[p]=std::atan(waku_msg -> pclP[p].point_right.x/-waku_msg -> pclP[p].point_right.z);
  center_s[p]=std::atan(waku_msg -> pclP[p].point_center.x/-waku_msg -> pclP[p].point_center.z);
  centerleft_s[p]=std::atan(waku_msg -> pclP[p].point_centerleft.x/-waku_msg -> pclP[p].point_centerleft.z);
  centerright_s[p]=std::atan(waku_msg -> pclP[p].point_centerright.x/-waku_msg -> pclP[p].point_centerright.z);
  
  std::cout <<"Left_L["<<p<<"]="<<left_l[p]<<"_m"<< std::endl;
  std::cout <<"Right_L["<<p<<"]="<<right_l[p]<<"_m"<< std::endl;
    std::cout <<"centerleft_l["<<p<<"]="<<centerleft_l[p]<<"_m"<< std::endl;
  std::cout <<"centerright_l["<<p<<"]="<<centerright_l[p]<<"_m"<< std::endl;
  std::cout <<"Left_S["<<p<<"]="<<left_s[p]<<"_m"<< std::endl;
  std::cout <<"center_S["<<p<<"]="<<center_s[p]<<"_m"<< std::endl;
  std::cout <<"Right_S["<<p<<"]="<<right_s[p]<<"_m"<< std::endl;
  std::cout <<"center_S["<<p<<"]="<<center_s[p]<<"_m"<< std::endl;
  std::cout <<"centerleft_S["<<p<<"]="<<centerleft_s[p]<<"_m"<< std::endl;
  std::cout <<"centerright_S["<<p<<"]="<<centerright_s[p]<<"_m"<< std::endl;


  kosuL1[p]=1+std::abs(left_s[p]-centerleft_s[p])/laser_msg -> angle_increment;//LeftXからCenterXの角度θを1サンプリング角度で割ることでLeftからCenterまでの個数を求める
  kosuL2[p]=1+std::abs(centerleft_s[p]-center_s[p])/laser_msg -> angle_increment;
  kosuR1[p]=1+std::abs(center_s[p]-centerright_s[p])/laser_msg -> angle_increment;
  kosuR2[p]=1+std::abs(centerright_s[p]-right_s[p])/laser_msg -> angle_increment;
  
  //henka_l[p]=(left_l[p]-right_l[p])/kosu[p];
  henka_Ll_1[p]=(left_l[p]-centerleft_l[p])/kosuL1[p];//距離の差をLeftからCenterまでの個数で割ることで1サンプリングあたりの変化量を求める
  henka_Ll_2[p]=(centerleft_l[p]-center_l[p])/kosuL2[p];
  henka_Rl_1[p]=(center_l[p]-centerright_l[p])/kosuR1[p];
  henka_Rl_2[p]=(centerright_l[p]-right_l[p])/kosuR2[p];

   std::cout <<"kosuL1["<<p<<"]="<<kosuL1[p]<< std::endl;
   std::cout <<"kosuL2["<<p<<"]="<<kosuL2[p]<< std::endl;
   std::cout <<"kosuR1["<<p<<"]="<<kosuR1[p]<< std::endl;
   std::cout <<"kosuR2["<<p<<"]="<<kosuR2[p]<< std::endl;
  // std::cout <<"henka_Ll["<<p<<"]="<<henka_Ll[p]<<std::endl;
  // std::cout <<"henka_Rl["<<p<<"]="<<henka_Rl[p]<<std::endl;

  LL[p].resize(kosuL1[p]);
  LCL[p].resize(kosuL2[p]);
  RCL[p].resize(kosuR1[p]);
  RL[p].resize(kosuR2[p]);
  LS[p].resize(kosuL1[p]);
  LCS[p].resize(kosuL2[p]);
  RCS[p].resize(kosuR1[p]);
  RS[p].resize(kosuR2[p]);

  // LL[p].resize(laserScan_src.ranges.size());
  // LCL[p].resize(laserScan_src.ranges.size());
  // RL[p].resize(laserScan_src.ranges.size());
  // RCL[p].resize(laserScan_src.ranges.size());
  // LS[p].resize(laserScan_src.ranges.size());
  // LCS[p].resize(laserScan_src.ranges.size());
  // RS[p].resize(laserScan_src.ranges.size());
  // RCS[p].resize(laserScan_src.ranges.size());

  ROS_INFO("koko1");

for(i=0;i<(kosuL1[p]);i++){
    ROS_INFO("koko2");

    LL[p][i] = left_l[p] + henka_Ll_1[p] * i;//LB番号の距離の値 
    LS[p][i] = left_s[p]-laser_msg -> angle_increment*i;//LB番号の角度の値

    int LB,RB;
    LB=(LS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//rengesの要素番号を求める(LBは要素番号)

    if(LB>=laserScan_src.ranges.size()||LB<0){}
     else{ 
       laserScan_src.ranges[LB] = LL[p][i];
       std::cout <<"LB="<<LB<< std::endl; }//rengesの要素(距離データ)書き換え 
     ROS_INFO("koko3");
}//for文i

for(i=0;i<(kosuL2[p]);i++){

    LCL[p][i] = centerleft_l[p] + henka_Ll_2[p] * i;//LB番号の距離の値 
    LCS[p][i] = centerleft_s[p]-laser_msg -> angle_increment*i;//LB番号の角度の値

    int LCB,RB;
    LCB=(LCS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//rengesの要素番号を求める(LBは要素番号)

    if(LCB>=laserScan_src.ranges.size()||LCB<0){}
     else{ laserScan_src.ranges[LCB] = LCL[p][i];std::cout <<"LCB="<<LCB<< std::endl; }//rengesの要素(距離データ)書き換え 
     ROS_INFO("koko4");
}//for文i

for(i=0;i<kosuR1[p];i++){
    RCL[p][i] = centerright_l[p] + henka_Rl_1[p] * i;//RB番号の距離
    RCS[p][i]=centerright_s[p]-laser_msg -> angle_increment*i;//RB番号の角度

    int RCB;
    RCB=(RCS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//要素番号

    if(RCB>=laserScan_src.ranges.size()||RCB<0){}
    else{ laserScan_src.ranges[RCB] = RCL[p][i]; std::cout <<"RCB="<<RCB<< std::endl;}//rengesの要素(距離データ)書き換え

    ROS_INFO("koko5");

}//for文i

for(i=0;i < kosuR2[p];i++){
    RL[p][i] = right_l[p] + henka_Rl_2[p] * i;//RB番号の距離
    RS[p][i]= right_s[p]-laser_msg -> angle_increment*i;//RB番号の角度

    int RB;
    RB=(RS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//要素番号

    if(RB>=laserScan_src.ranges.size()||RB<0){}
    else{ laserScan_src.ranges[RB] = RL[p][i]; std::cout <<"RB="<<RB<< std::endl;}//rengesの要素(距離データ)書き換え

    ROS_INFO("koko6");
}//for文i



}//for文p
ROS_INFO("pub1");
laserScan_src.header.stamp = ros::Time::now();//pubされた現在時刻を記録
ROS_INFO("pub2");
laser_pub.publish(laserScan_src);//データ送信
ROS_INFO("pub3");

cv::waitKey(1);

}

int main(int argc,char **argv){//int argc=引数の個数 char **argv=引数の配列
	ros::init(argc,argv,"deptu_pcl4_sub"/*ノードの名前*/);

    //データ受け取り設定	
	ros::NodeHandle nhSub;//topic設定で使う
	message_filters::Subscriber<OpenCV1::wakuhairetu> waku_sub(nhSub, "wakuhairetu", 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nhSub, "/robot1/scan", 1);

    
	typedef message_filters::sync_policies::ApproximateTime<OpenCV1::wakuhairetu,sensor_msgs::LaserScan> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),waku_sub, laser_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

  ros::NodeHandle nhPub;//topic設定で使う
  laser_pub/*pub変数名*/=nhPub.advertise<sensor_msgs::LaserScan/*変数型*/>("Laser_dst"/*トピック名*/, 1000);

    	
	ros::spin();
	
	return 0;
}

