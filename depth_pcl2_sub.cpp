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

void callback(const OpenCV1::wakuhairetu::ConstPtr& waku_msg,const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
waku_msg -> pclP[0].point_left.x;//pclPという名の配列の0番目のpoint_left.xの値
waku_msg -> pclP.size();//pclPの配列の個数(大枠の個数)
ROS_INFO("callBack");
std::cout <<"B="<<waku_msg -> pclP.size()<<"_m"<< std::endl;
int i,p,waku_size=waku_msg -> pclP.size();

// double left_l[waku_size],right_l[waku_size],left_s[waku_size],right_s[waku_size];
// double kosu[waku_size],henka_l[waku_size];
std::vector<double> left_l(waku_size);
std::vector<double> right_l(waku_size);
std::vector<double> left_s(waku_size);
std::vector<double> right_s(waku_size);
std::vector<int> kosu(waku_size);
std::vector<double> henka_l(waku_size);
std::vector<std::vector<double>> L(waku_size);
std::vector<std::vector<double>> S(waku_size);

for(p=0;p<waku_msg -> pclP.size();p++){//枠の個数

  left_l[p]=std::sqrt(waku_msg -> pclP[p].point_left.y * waku_msg -> pclP[p].point_left.y + waku_msg -> pclP[p].point_left.z * waku_msg -> pclP[p].point_left.z);
  right_l[p]=std::sqrt(waku_msg -> pclP[p].point_right.y * waku_msg -> pclP[p].point_right.y + waku_msg -> pclP[p].point_right.z * waku_msg -> pclP[p].point_right.z);
  left_s[p]=std::atan(waku_msg -> pclP[p].point_left.y/waku_msg -> pclP[p].point_left.z);
  right_s[p]=std::atan(waku_msg -> pclP[p].point_right.y/waku_msg -> pclP[p].point_right.z);
  
  std::cout <<"Left_L=["<<p<<"]="<<left_l[p]<<"_m"<< std::endl;
  std::cout <<"Right_L=["<<p<<"]="<<right_l[p]<<"_m"<< std::endl;
    std::cout <<"Left_S=["<<p<<"]="<<left_s[p]<<"_m"<< std::endl;
    std::cout <<"Right_S=["<<p<<"]="<<right_s[p]<<"_m"<< std::endl;

  kosu[p]=1+std::abs(left_s[p]-right_s[p])/laser_msg -> angle_increment;
  henka_l[p]=(left_l[p]-right_l[p])/kosu[p];

  std::cout <<"kosu=["<<p<<"]="<<kosu[p]<<"_m"<< std::endl;
  std::cout <<"henka_l=["<<p<<"]="<<henka_l[p]<<"_m"<< std::endl;

  L[p].resize(kosu[p]);
  S[p].resize(kosu[p]);

for(i=0;i<kosu[p];i++){

    L[p][i]=left_l[p] + henka_l[p]*i;
    S[p][i]=left_s[p]-laser_msg -> angle_increment*i;

    std::cout <<"L=["<<p<<"]["<<i<<"]="<<L[p][i]<<"_m"<< std::endl;
    std::cout <<"S=["<<p<<"]["<<i<<"]="<<S[p][i]<<"_m"<< std::endl;

  
}

}


}

int main(int argc,char **argv){
	ros::init(argc,argv,"deptu_pcl2_sub");

    //データ受け取り設定	
	ros::NodeHandle nhSub;
	message_filters::Subscriber<OpenCV1::wakuhairetu> waku_sub(nhSub, "wakuhairetu", 1);
	message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nhSub, "scan", 1);

    
	typedef message_filters::sync_policies::ApproximateTime<OpenCV1::wakuhairetu,sensor_msgs::LaserScan> MySyncPolicy;	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),waku_sub, laser_sub);
	sync.registerCallback(boost::bind(&callback,_1, _2));

    	
	ros::spin();
	
	return 0;
}
