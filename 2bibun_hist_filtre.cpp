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


int main(int argc, char *argv[])
{
string name = "IMG_0297";
string file = "filter";
string file_src = "image3/"+name+".jpg";//元画像
string file_grey = "image3/"+file+"/B0_"+name+"grey.jpg";//グレースケール
string file_hist = "image3/"+file+"/B0_"+name+"hist.jpg";//ヒストグラム均一化
string file_cont = "image3/"+file+"/B0_"+name+"hist_cont.jpg";//コントラスト
//string file_meido = "image2/"+file+"/DSC_"+name+"hist_contup_meido0.jpg";//明度調整
string file_dst = "image3/"+file+"/B0_"+name+"hist_cont_after2.jpg";//二値化
string file_dst2 = "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika.jpg";//二値化エッジ抽出画像
string file_dst3= "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika_mask.jpg";//mask画像
string file_dst4 = "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika_out.jpg";//カラーフィルタ処理画像
string file_dst5 = "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika_out_UP.jpg";//膨張処理画像
string file_dst6 = "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika_out_UP_mono.jpg";//密度検出画像
string file_dst7 = "image3/"+file+"/B0_"+name+"hist_cont_after2_2tika_out_UP_mono_mitudo.jpg";//密度検出画像
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
Mat img_hist;
Mat img_grey;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);

//グレースケール化
cvtColor(img_src,img_grey,COLOR_BGR2GRAY);

equalizeHist(img_grey,img_hist);//ヒストグラム均一化

//コントラスト軽減
int min=0,max=200;
img_hist.convertTo(img_cont,img_hist.type(),(max-min)/255.0,min);

//明度調整
//double shift =0;
//img_cont.convertTo(img_meido,img_cont.type(),1.0,shift);


//二値化
//int thresh =100;
//threshold(img_hist,img_dst,thresh,255,THRESH_BINARY);

//膨張縮小処理(一回目)
Mat img_tmp,img_ab,img_tmpp,img_ab1,img_ab2,img_aba;
Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
morphologyEx(img_cont,img_tmp,MORPH_CLOSE,element8,Point(-1,-1),2);
morphologyEx(img_tmp,img_ab,MORPH_OPEN,element8,Point(-1,-1),2);

morphologyEx(img_ab,img_tmpp,MORPH_OPEN,element8,Point(-1,-1),2);
morphologyEx(img_tmpp,img_ab1,MORPH_CLOSE,element8,Point(-1,-1),2);

GaussianBlur(img_ab1,img_ab2,Size(11,11),1);
bilateralFilter(img_ab2,img_aba,11,50,100);


//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_aba, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//二値化
int thresh =100;
threshold(img_dst2,img_dst3,thresh,255,THRESH_BINARY);



	//膨張処理
	dilate(img_dst3,img_dst5,element8,Point(-1,-1),1);
	//cvtColor(img_dst5,img_dst6,COLOR_GRAY2BGR);



std::cout<<"test"<<std::endl;
	double i,j,x,y;
	double siro=0;
    int W=20,H=20;
    //画像サイズ設定
	std::vector<std::vector<double>> A(img_src.size().width, std::vector<double>(img_src.size().height));

std::cout<<"test1"<<std::endl;
    //白の割合を調べる
   for(i=0;i<=img_src.size().width-W;i=i+W){
	 for(j=0;j<=img_src.size().height-H;j=j+H){ 
		 siro=0; std::cout<<"test2"<<std::endl;
		 for(x=i;x<=i+W;x=x+2){
			 for(y=j;y<=j+H;y=y+2){
	            unsigned char intensity = img_dst5.at<unsigned char>(y, x);//白か黒かを調べる
	            std::cout <<"i:"<<i<<"j:"<<j<<"X:"<<x<<"y:"<<y<<"a:"<<(unsigned int)intensity<< std::endl;//表示
	            if(intensity>=200){siro=siro+1;} 
                 }}
	 A[i][j]=(siro/100)*100;}}

std::cout<<"test3"<<std::endl;

	 img_dst7=img_src.clone();//コピー作成

std::cout<<"test4"<<std::endl;

//結果表示
   for(i=0;i<img_src.size().width;i=i+W){
	 for(j=0;j<img_src.size().height;j=j+H){
		 if(A[i][j]>=30){rectangle(img_dst7,Rect(i,j,W,H),Scalar(255,0,0),2);}//赤四角作成

		 std::cout <<A[i][j]<< std::endl;//printf
     }}

//スキャン２回め
/*int W2=40,H2=40;
   for(i=0;i<=img_src.size().width-W2;i=i+W2){
	 for(j=0;j<=img_src.size().height-H2;j=j+H2){ 
		 siro=0; std::cout<<"test2"<<std::endl;
		 for(x=i;x<=i+W2;x=x+4){
			 for(y=j;y<=j+H2;y=y+4){
	            unsigned char intensity = img_dst5.at<unsigned char>(y, x);//白か黒かを調べる
	            std::cout <<"i:"<<i<<"j:"<<j<<"X:"<<x<<"y:"<<y<<"a:"<<(unsigned int)intensity<< std::endl;//表示
	            if(intensity>=200){siro=siro+1;} 
                 }}
	 A[i][j]=(siro/100)*100;}}

std::cout<<"test3"<<std::endl;


std::cout<<"test4"<<std::endl;

//結果表示
   for(i=0;i<img_src.size().width;i=i+W2){
	 for(j=0;j<img_src.size().height;j=j+H2){
		 if(A[i][j]>=10){rectangle(img_dst7,Rect(i,j,W2,H2),Scalar(255,0,0),2);}//青四角作成

		 std::cout <<A[i][j]<< std::endl;//printf
     }}*/


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
//imwrite(file_cont, img_cont);
//imwrite(file_meido, img_meido);
imwrite(file_hist, img_hist);
imwrite(file_grey, img_grey);




waitKey(0);
return 0;
}


