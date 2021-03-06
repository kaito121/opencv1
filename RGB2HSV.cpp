#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
using namespace std;
using namespace cv;
string win_src = "src";
string win_dst = "dst";
string win_HSV = "HSV";
// 抽出する画像の輝度値の範囲を指定
#define B_MAX 255
#define B_MIN 150
#define G_MAX 255
#define G_MIN 180
#define R_MAX 255
#define R_MIN 180


int main(int argc,char** argv)
{
string name = "IMG_0301";
string file = "hist/0301";
string file_src = "image/"+name+".jpg";//元画像
/*string file_grey = "image3/"+file+"/B8_1"+name+"grey.jpg";//グレースケール
string file_hist = "image3/"+file+"/B8_1"+name+"hist.jpg";//ヒストグラム均一化
string file_cont = "image3/"+file+"/B8_1"+name+"hist_cont.jpg";//コントラスト
//string file_meido = "image2/"+file+"/DSC_"+name+"hist_contup_meido0.jpg";//明度調整
string file_dst = "image3/"+file+"/B8_1"+name+"hist_cont_after2.jpg";//二値化
string file_dst2 = "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika.jpg";//二値化エッジ抽出画像
string file_dst3= "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika_mask.jpg";//mask画像
string file_dst4 = "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika_out.jpg";//カラーフィルタ処理画像
string file_dst5 = "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika_out_UP.jpg";//膨張処理画像
string file_dst6 = "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika_out_UP_mono.jpg";//密度検出画像
string file_dst7 = "image3/"+file+"/B8_1"+name+"hist_cont_after2_2tika_out_UP_mono_mitudo.jpg";//密度検出画像*/
//strin file_HSV = "image3/"+name+"_HSV.jpg";
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
Mat img_HSV;

if (!img_src.data) {
cout << "error" << endl;
return -1;
}

namedWindow(win_src, WINDOW_AUTOSIZE);
namedWindow(win_dst, WINDOW_AUTOSIZE);
namedWindow(win_HSV, WINDOW_AUTOSIZE);

//HSV化
cvtColor(img_src,img_HSV,COLOR_BGR2HSV);

//グレースケール化
//cvtColor(img_src,img_grey,COLOR_BGR2GRAY);

//equalizeHist(img_grey,img_hist);//ヒストグラム均一化

//コントラスト軽減
//int min=0,max=200;
//img_hist.convertTo(img_cont,img_hist.type(),(max-min)/255.0,min);

//膨張縮小処理(一回目)
/*Mat img_tmp,img_ab,img_tmpp,img_ab1;
Mat element8=(Mat_<uchar>(3,3)<<1,1,1,1,1,1,1,1);
morphologyEx(img_cont,img_tmp,MORPH_CLOSE,element8,Point(-1,-1),4);
morphologyEx(img_tmp,img_ab,MORPH_OPEN,element8,Point(-1,-1),4);

morphologyEx(img_ab,img_tmpp,MORPH_OPEN,element8,Point(-1,-1),1);
morphologyEx(img_tmpp,img_ab1,MORPH_CLOSE,element8,Point(-1,-1),1);


//エッジ抽出(二次微分)
Mat img_tmp1;
Laplacian(img_ab1, img_tmp1, CV_32F, 3);
convertScaleAbs(img_tmp1, img_dst2, 1, 0);

//二値化
int thresh =100;
threshold(img_dst2,img_dst3,thresh,255,THRESH_BINARY);

//膨張処理
dilate(img_dst3,img_dst5,element8,Point(-1,-1),1);*/


/*std::cout<<"test"<<std::endl;
	//double i,j,x,y;
	double siro=0;
    int W=30,H=30;
    img_dst7=img_src.clone();//コピー作成
    //画像サイズ設定
	std::vector<std::vector<double>> A(img_src.size().width, std::vector<double>(img_src.size().height));

std::cout<<"test1"<<std::endl;

    //白の割合を調べる
    Mat src_img = cv::imread("input.png",0);
    for(int x = 0;x < img_src.size().width-W;x=x+W){
        for(int y = 0;y < img_src.size().height-H;y=y+H){
            Mat cut_img(img_dst5,Rect(x,y,W,H)); // 30px四方で切り抜く
            siro=0;
                    cut_img.forEach<unsigned char>([&](unsigned char &p, const int  position[]) -> void{
                        if(p>=200){siro=siro+1;} 
                     });
                     A[x][y]=(siro/(W*H))*100;
                     }  }
                     //結果表示
   for(int x=0;x<img_src.size().width;x=x+W){
	 for(int y=0;y<img_src.size().height;y=y+H){
		 if(A[x][y]>=10){rectangle(img_dst7,Rect(x,y,W,H),Scalar(0,0,255),2);}//青四角作成

		 std::cout <<A[x][y]<< std::endl;//printf
     }}*/



std::cout<<"test3"<<std::endl;


imshow(win_src, img_src);
imshow(win_HSV, img_HSV);
//imshow(win_dst, img_dst);
/*imwrite(file_dst, img_dst);
imwrite(file_dst2, img_dst2);
imwrite(file_dst3, img_dst3);
imwrite(file_dst4, img_dst4);
imwrite(file_dst5, img_dst5);
imwrite(file_dst6, img_dst6);
imwrite(file_dst7, img_dst7);
//imwrite(file_cont, img_cont);
//imwrite(file_meido, img_meido);
imwrite(file_hist, img_hist);
imwrite(file_grey, img_grey);*/




waitKey(0);
return 0;
}
