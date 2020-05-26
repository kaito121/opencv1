#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;
string win_src = "src";
string win_dst = "dst";
// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180


int main()
{
string name = "59232";
string file = "Edge_2tika";
string file_src = "image/"+name+".jpg";//元画像
//string file_cont = "image2/"+file+"/DSC_"+name+"contdown.jpg";//コントラスト
//string file_meido = "image2/"+file+"/DSC_"+name+"contdown_meido0.jpg";//コントラスト
string file_dst = "image2/"+file+"/"+name+"Edge.jpg";//二値化
string file_dst2 = "image2/"+file+"/"+name+"Edge_2tika.jpg";//二値化エッジ抽出画像

Mat img_src = imread(file_src, 1);

Mat img_dst;//二値化処理
Mat img_dst2;//エッジ抽出
Mat img_dst3;//マスク作成
Mat img_dst4;//RGBフィルター
Mat img_dst5;//膨張処理
Mat img_dst6;
Mat img_dst7;
Mat img_src1;
Mat img_cont;
Mat img_meido;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);

//コントラスト軽減
//int min=0,max=200;
//img_src.convertTo(img_cont,img_src.type(),(max-min)/255.0,min);

//明度調整
//double shift =0;
//img_cont.convertTo(img_meido,img_cont.type(),1.0,shift);

//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_src, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst, 1, 0);


//二値化
int thresh =100;
threshold(img_dst,img_dst2,thresh,255,THRESH_BINARY);

//膨張縮小処理(一回目)
//Mat img_tmp;
//Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
//morphologyEx(img_dst,img_tmp,MORPH_OPEN,element8,Point(-1,-1),1);
//morphologyEx(img_tmp,img_dst,MORPH_CLOSE,element8,Point(-1,-1),1);


//エッジ抽出(二次微分)
//Mat img_tmp1;
//Laplacian(img_dst, img_tmp1, CV_32F, 3);
//convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//カラー抽出（白だけを抽出させる)




//imshow(win_src, img_src);
//imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);

//imwrite(file_cont, img_cont);
//imwrite(file_meido, img_meido);



waitKey(0);
return 0;
}
