#include<cv.h>
#include<highgui.h>

int main(int argc,char **argv)
{
    double min_val,max_val;
    CvPoint min_loc,max_loc;
    CvSize dst_size;
    IplImage *src_img,*tmp_img,*dst_img;

    if(argc != 3  ||
    (src_img = cvLoadImage (argv[1],CV_LOAD_IMAGE_CLODE))==0||
    (tmp_img = cvLoadImage (argv[2],CV_LOAD_IMAGE_COLOR))==0) 
    return -1;

    dst_size = cvSize (src_img->width - tmp_img->width +1,src_img->heigh - tmp_img->height+1);
    dst_img= cvCreateImage(dst_size,IPL_DEPTH_32F,1);
    cvMatchTemplate(src_img,tmp_img,dst_img,CV_TM_CCOEFF_NORMED);
    cvMinMaxLoc(dst_img,&min_val,&max_val,&min_loc,&max_loc,NULL);
    
    cvRectangle(src_img,max_loc,
                cvPoint(max_loc.x+tmp_img->width,max_loc.y+tmp_img->height),CV_RGB(255,0,0),3);
    cvNameWindow("Image",1);
    cvShowImage("Image",src_img);
    cvWaitKey(0);

    cvDestroyWindow("Image");
    cvReleaseImage(&src_img);
    cvReleaseImage(&tmp_img);
    cvReleaseImage(&dst_img);

    return 0;
                
}
