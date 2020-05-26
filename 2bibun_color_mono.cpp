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
string file = "1008";
string file_src = "image2/DSC_"+file+".jpg";//元画像
string file_dst = "image2/1DSC_"+file+"after2.jpg";//二値化
string file_dst2 = "image2/1DSC_"+file+"after2_2tika.jpg";//二値化エッジ抽出画像
string file_dst3= "image2/1DSC_"+file+"after2_2tika_mask.jpg";//mask画像
string file_dst4 = "image2/1DSC_"+file+"after2_2tika_out.jpg";//カラーフィルタ処理画像
string file_dst5 = "image2/1DSC_"+file+"after2_2tika_out_UP.jpg";//膨張処理画像
string file_dst6 = "image2/1DSC_"+file+"after2_2tika_out_UP_mono.jpg";//密度検出画像
string file_dst7 = "image2/1DSC_"+file+"after2_2tika_out_UP_mono_mitudo.jpg";//密度検出画像
Mat img_src = imread(file_src, 1);

Mat img_dst;//二値化処理
Mat img_dst2;//エッジ抽出
Mat img_dst3;//マスク作成
Mat img_dst4;//RGBフィルター
Mat img_dst5;//膨張処理
Mat img_dst6;
Mat img_dst7;
Mat img_src1;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);


//二値化
int thresh =100;
threshold(img_src,img_dst,thresh,255,THRESH_BINARY);

//膨張縮小処理(一回目)
Mat img_tmp;
Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
morphologyEx(img_dst,img_tmp,MORPH_OPEN,element8,Point(-1,-1),1);
morphologyEx(img_tmp,img_dst,MORPH_CLOSE,element8,Point(-1,-1),1);


//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_dst, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//カラー抽出（白だけを抽出させる)

// inRangeを用いてフィルタリング
	Scalar s_min = Scalar(B_MIN, G_MIN, R_MIN);
	Scalar s_max = Scalar(B_MAX, G_MAX, R_MAX);
	inRange(img_dst2, s_min, s_max, img_dst3);

	// マスク画像を表示
	namedWindow("mask");
	//imshow("mask", img_dst);
	//imwrite("image/59233after2_2tika_mask.jpg", img_dst3);

	// マスクを基に入力画像をフィルタリング
	img_dst2.copyTo(img_dst4, img_dst3);

	//膨張処理
	dilate(img_dst4,img_dst5,element8,Point(-1,-1),1);
	
	cvtColor(img_dst5,img_dst6,COLOR_BGR2GRAY);


std::cout<<"test"<<std::endl;
	double i,j,x,y;
	double siro=0;
	int W=30,H=30;
    //画像サイズ設定
	std::vector<std::vector<double>> A(img_src.size().width, std::vector<double>(img_src.size().height));

std::cout<<"test1"<<std::endl;
    //白の割合を調べる
   for(i=0;i<=img_src.size().width-W;i=i+W){
	 for(j=0;j<=img_src.size().height-H;j=j+H){ 
		 siro=0; std::cout<<"test2"<<std::endl;
		 for(x=i;x<=i+H;x=x+3){
			 for(y=j;y<=j+W;y=y+3){
	            unsigned char intensity = img_dst6.at<unsigned char>(y, x);//白か黒かを調べる
	            std::cout <<"i:"<<i<<"j:"<<j<<"X:"<<x<<"y:"<<y<<"a:"<<(unsigned int)intensity<< std::endl;//表示
	            if(intensity>=200){siro=siro+1;} 
                 }}
	 A[i][j]=(siro/100)*100;}}
std::cout<<"test3"<<std::endl;

	 img_dst7=img_dst5.clone();



std::cout<<"test4"<<std::endl;

   for(i=0;i<img_src.size().width;i=i+W){
	 for(j=0;j<img_src.size().height;j=j+H){
		 if(A[i][j]>=30){rectangle(img_dst7,Rect(i,j,W,H),Scalar(0,0,255),2);}

		 std::cout <<A[i][j]<< std::endl;//printf
     }}

std::cout<<"test5"<<std::endl;


//imshow(win_src, img_src);
//imshow(win_dst, img_dst);
imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);
imwrite(file_dst3, img_dst3);
imwrite(file_dst4, img_dst4);
imwrite(file_dst5, img_dst5);
imwrite(file_dst6, img_dst6);
imwrite(file_dst7, img_dst7);




waitKey(0);
return 0;
}
