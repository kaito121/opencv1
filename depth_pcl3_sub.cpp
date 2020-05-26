
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
#include <limits>
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
int i,p,waku_size=waku_msg -> pclP.size();

// double left_l[waku_size],right_l[waku_size],left_s[waku_size],right_s[waku_size];
// double kosu[waku_size],henka_l[waku_size];
std::vector<double> left_l(waku_size);
std::vector<double> right_l(waku_size);
std::vector<double> center_l(waku_size);
std::vector<double> left_s(waku_size);
std::vector<double> right_s(waku_size);
std::vector<double> center_s(waku_size);
std::vector<int> kosuR(waku_size);
std::vector<int> kosuL(waku_size);
std::vector<double> henka_Ll(waku_size);
std::vector<double> henka_Rl(waku_size);
std::vector<std::vector<float>> LL(waku_size);
std::vector<std::vector<float>> RL(waku_size);
std::vector<std::vector<double>> LS(waku_size);
std::vector<std::vector<double>> RS(waku_size);

for(i=0;i<laserScan_src.ranges.size();i++){
  if(laserScan_src.ranges[i]==0){
   laserScan_src.ranges[i]=std::numeric_limits<float>::quiet_NaN();
   std::cout <<"i="<<i<< std::endl;
   }
}

for(p=0;p<waku_msg -> pclP.size();p++){//枠の個数

  left_l[p]=std::sqrt(waku_msg -> pclP[p].point_left.x * waku_msg -> pclP[p].point_left.x + waku_msg -> pclP[p].point_left.z * waku_msg -> pclP[p].point_left.z);
  right_l[p]=std::sqrt(waku_msg -> pclP[p].point_right.x * waku_msg -> pclP[p].point_right.x + waku_msg -> pclP[p].point_right.z * waku_msg -> pclP[p].point_right.z);
  center_l[p]=std::sqrt(waku_msg -> pclP[p].point_center.x * waku_msg -> pclP[p].point_center.x + waku_msg -> pclP[p].point_center.z * waku_msg -> pclP[p].point_center.z);
  left_s[p]=std::atan(waku_msg -> pclP[p].point_left.x/-waku_msg -> pclP[p].point_left.z);
  right_s[p]=std::atan(waku_msg -> pclP[p].point_right.x/-waku_msg -> pclP[p].point_right.z);
  center_s[p]=std::atan(waku_msg -> pclP[p].point_center.x/-waku_msg -> pclP[p].point_center.z);
  
  std::cout <<"Left_L["<<p<<"]="<<left_l[p]<<"_m"<< std::endl;
  std::cout <<"Right_L["<<p<<"]="<<right_l[p]<<"_m"<< std::endl;
  std::cout <<"Left_S["<<p<<"]="<<left_s[p]<<"_m"<< std::endl;
  std::cout <<"center_S["<<p<<"]="<<center_s[p]<<"_m"<< std::endl;
  std::cout <<"Right_S["<<p<<"]="<<right_s[p]<<"_m"<< std::endl;

  kosuL[p]=1+std::abs(left_s[p]-center_s[p])/laser_msg -> angle_increment;
  kosuR[p]=1+std::abs(center_s[p]-right_s[p])/laser_msg -> angle_increment;
  //henka_l[p]=(left_l[p]-right_l[p])/kosu[p];
  henka_Ll[p]=(left_l[p]-center_l[p])/kosuL[p];
  henka_Rl[p]=(center_l[p]-right_l[p])/kosuR[p];

  std::cout <<"kosuL["<<p<<"]="<<kosuL[p]<< std::endl;
  std::cout <<"kosuR["<<p<<"]="<<kosuR[p]<< std::endl;
  std::cout <<"henka_Ll["<<p<<"]="<<henka_Ll[p]<<std::endl;
  std::cout <<"henka_Rl["<<p<<"]="<<henka_Rl[p]<<std::endl;

  LL[p].resize(kosuL[p]);
  RL[p].resize(kosuR[p]);
  LS[p].resize(kosuL[p]);
  RS[p].resize(kosuR[p]);

for(i=0;i<(kosuL[p]);i++){

    LL[p][i] = left_l[p] + henka_Ll[p] * i;//LB番号の距離の値
    LS[p][i] = left_s[p]-laser_msg -> angle_increment*i;//LB番号の角度の値

   //std::cout <<"LL=["<<p<<"]["<<i<<"]="<<LL[p][i]<<"_m"<< std::endl;
    //std::cout <<"LS=["<<p<<"]["<<i<<"]="<<LS[p][i]<<"_rad"<< std::endl;
    int LB,RB;
    LB=(LS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//rengesの要素番号を求める(LBは要素番号)
    //std::cout <<"laserScan_src.angle_min="<<laserScan_src.angle_min<<"_rad"<< std::endl;
    std::cout <<"LB="<<LB<< std::endl;
    std::cout <<"range.size="<<laserScan_src.ranges.size()<<"_kosu"<< std::endl;
    //if(LS[p][i]>=laserScan_src.angle_max||LL[p][i]>laserScan_src.range_max||LL[p][i]<laserScan_src.range_min){}
      if(LB>=laserScan_src.ranges.size()||LB<0){}
     else{ laserScan_src.ranges[LB] = LL[p][i]; }//rengesの要素(距離データ)書き換え 
}//for文i

for(i=0;i<(kosuR[p]);i++){
    RL[p][i] = center_l[p] + henka_Rl[p] * i;//RB番号の距離
    RS[p][i]=center_s[p]-laser_msg -> angle_increment*i;//RB番号の角度

  //  std::cout <<"RL=["<<p<<"]["<<i<<"]="<<RL[p][i]<<"_m"<<std::endl;
  //  std::cout <<"RS=["<<p<<"]["<<i<<"]="<<RS[p][i]<<"_rad"<< std::endl;
    int LB,RB;
    RB=(RS[p][i]-laserScan_src.angle_min)/laserScan_src.angle_increment;//要素番号
    // std::cout <<"laserScan_src.angle_min="<<laserScan_src.angle_min<< std::endl;
    // std::cout <<"RB="<<RB<< std::endl;
    // std::cout <<"range.size="<<laserScan_src.ranges.size()<<"_kosu"<< std::endl;
    // std::cout <<"range.min="<<laserScan_src.range_min<<"_m"<< std::endl;
    // std::cout <<"range.max="<<laserScan_src.range_max<<"_m"<< std::endl;
//     if(RS[p][i]<=laserScan_src.angle_min||RL[p][i]>laserScan_src.range_max||RL[p][i]<laserScan_src.range_min){}
      if(RB>=laserScan_src.ranges.size()||RB<0){}
    else{ laserScan_src.ranges[RB] = RL[p][i]; }//rengesの要素(距離データ)書き換え

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
	ros::init(argc,argv,"deptu_pcl3_sub"/*ノードの名前*/);

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
