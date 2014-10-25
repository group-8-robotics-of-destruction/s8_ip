#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/synchronizer.h>
#include <signal.h>
#include <iostream>


using namespace cv;
using namespace std;



#define HZ 10
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_GRAY = "Grayscale Image Windows";
static const std::string OPENCV_WINDOW_DEPTH = "Depth Image Windows";
static const std::string OPENCV_WINDOW_CONTROL = "Control Window";
//static const std::string OPENCV_WINDOW_CC = "Connected Component Window";

class BallDectector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Subscriber image_depth_sub_;
    image_transport::Publisher image_pub_;
    int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, iErode;
    int maxDist;
    bool isInitializedDepth, isInitializedColor;
    cv_bridge::CvImagePtr cv_depth_ptr;
    cv_bridge::CvImagePtr cv_ptr;

public:
    BallDectector()
    : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
          &BallDectector::imageColorCb, this);
        image_depth_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
          &BallDectector::imageDepthCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        iLowH = 0;
        iHighH = 50;
        iLowS = 80;
        iHighS = 255;
        iLowV = 36;
        iHighV = 255;
        iErode = 15;
        maxDist = 3;
        isInitializedDepth = false;
        isInitializedColor = false;

        cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow(OPENCV_WINDOW_CONTROL);
        //cv::namedWindow(OPENCV_WINDOW_CC);
        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", OPENCV_WINDOW_CONTROL.c_str(), &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", OPENCV_WINDOW_CONTROL.c_str(), &iHighH, 179);

        cvCreateTrackbar("LowS", OPENCV_WINDOW_CONTROL.c_str(), &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", OPENCV_WINDOW_CONTROL.c_str(), &iHighS, 255);

        cvCreateTrackbar("LowV", OPENCV_WINDOW_CONTROL.c_str(), &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", OPENCV_WINDOW_CONTROL.c_str(), &iHighV, 255);
        cvCreateTrackbar("Erode", OPENCV_WINDOW_CONTROL.c_str(), &iErode, 29); //Value (0 - 20)
        cvCreateTrackbar("maxDist", OPENCV_WINDOW_CONTROL.c_str(), &maxDist, 10); //Value (0 - 20)
    }

    ~BallDectector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
        cv::destroyWindow(OPENCV_WINDOW_CONTROL);
        cv::destroyWindow(OPENCV_WINDOW_GRAY);
        cv::destroyWindow(OPENCV_WINDOW_DEPTH);
        //cv::destroyWindow(OPENCV_WINDOW_CC);
    }

    void imageDepthCb(const sensor_msgs::ImageConstPtr& msgDepth)
    {
        try
        {
            cv_depth_ptr = cv_bridge::toCvCopy(msgDepth, sensor_msgs::image_encodings::TYPE_32FC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        isInitializedDepth = true;
    }


    void imageColorCb(const sensor_msgs::ImageConstPtr& msgColor)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msgColor, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        isInitializedColor = true;
    }

    void depthPreProcess(cv::Mat *normalizedDepth)
    {
        double max = 0.0;
        cv::minMaxLoc(cv_depth_ptr->image, 0, &max, 0, 0);
        cv::Mat Depth;
        Depth = cv_depth_ptr->image;
        Depth.convertTo(Depth, CV_32FC1, 1.0/max, 0);
        //medianBlur( Depth, Depth, 5 );
        GaussianBlur( Depth, Depth, Size(9, 9), 2, 2 );
        *normalizedDepth = Depth;
    }

    void colorPreProcess(cv::Mat *HSVImage, cv::Mat *RGBImage)
    {
        cv::Mat tempHSV, tempRGB;
        //Convert the captured frame from BGR to HSV
        cvtColor(cv_ptr->image, tempHSV, COLOR_BGR2HSV);
        tempRGB = cv_ptr->image;
        *HSVImage = tempHSV;
        *RGBImage = tempRGB;


        // Blur the image
        //GaussianBlur( tmpHSVImage, tmpHSVImage, Size(5, 5), 2, 2 );
        //inRange(tmpHSVImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), tmpgrayScaleImage);
        //*HSVImage = tmpHSVImage;
        //*grayScaleImage = tmpgrayScaleImage;
    }

    void getDirection()
    {
        if(isInitializedDepth == false) return;
        if(isInitializedColor == false) return;

        Mat normalizedDepth;
        Mat HSVImage;
        Mat RGBImage;
        // Create a normalized depth image, gaussian and mean filters used as well.
        depthPreProcess(&normalizedDepth);
        // Convert from BRG to HSV and use gaussian blur.
        colorPreProcess(&HSVImage, &RGBImage);



        //cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


        Mat imgThresholded;

        inRange(HSVImage, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );



        Mat cimg;
        Mat maskedimg;
        RGBImage.copyTo(maskedimg, imgThresholded);
        medianBlur(maskedimg, maskedimg, 5);

        cvtColor(maskedimg, cimg, COLOR_BGR2GRAY);

        vector<Vec3f> circles;
        HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, 10,
                     100, 30, 1, 200 // change the last two parameters
                                    // (min_radius & max_radius) to detect larger circles
                     );
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            circle( maskedimg, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, CV_AA);
            circle( maskedimg, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, CV_AA);
        }

        cv::imshow(OPENCV_WINDOW, maskedimg);
    }
};

bool stop = false;
void handler(int) {
    std::cout << "will exit..." << std::endl;
    stop = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ball_detector");
    BallDectector ic;
    ros::Rate loop_rate(HZ);

    while(stop == false){
        signal(SIGINT, &handler);
        ros::spinOnce();
        ic.getDirection();
        cv::waitKey(3);
        loop_rate.sleep();
    }
    ic.~BallDectector();
    return 0;
}


