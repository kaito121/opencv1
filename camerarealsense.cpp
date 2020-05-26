#include <opencv2/opencv.hpp>
#include <ros/ros.h>

int main( int argc, char* argv[] )
{
    // (1) Open cv::VideoCapture() with RealSense
    cv::VideoCapture capture( cv::VideoCaptureAPIs::CAP_INTELPERC );
    if( !capture.isOpened() ){
        return -1;
    }

    while( true ){
        // (2) Grab All Frames
        capture.grab();

        // (3) Retrieve Each Frames
        // (3.1) Color Frame
        cv::Mat color_frame;
        capture.retrieve( color_frame, cv::CAP_INTELPERC_IMAGE );

        // (3.2) Depth Frame
        cv::Mat depth_frame;
        capture.retrieve( depth_frame, cv::CAP_INTELPERC_DEPTH_MAP );

        // (3.3) Infrared Frame
        cv::Mat infrared_frame;
        capture.retrieve( infrared_frame, cv::CAP_INTELPERC_IR_MAP );

        // (4) Show Image
        cv::imshow( "Color", color_frame );
        depth_frame.convertTo( depth_frame, CV_8U, -255.0 / 10000.0, 255.0 ); // Scaling
        cv::imshow( "Depth", depth_frame );
        cv::imshow( "Infrared", infrared_frame );

        const int32_t key = cv::waitKey( 33 );
        if( key == 'q' ){
            break;
        }
    }

    cv::destroyAllWindows();

    return 0;
}
