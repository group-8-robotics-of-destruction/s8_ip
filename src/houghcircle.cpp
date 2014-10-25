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

static void help()
{
    cout << "\nThis program demonstrates circle finding with the Hough transform.\n"
            "Usage:\n"
            "./houghcircles <image_name>, Default is pic1.png\n" << endl;
}

int main(int argc, char** argv)
{
    const char* filename = argc >= 2 ? argv[1] : "/home/weilun/catkin_ws/src/group8/testimage/colorballs.png";


    Mat img = imread(filename, 3);

    if(img.empty())
    {
        help();
        cout << "can not open " << filename << endl;
        return -1;
    }

    Mat imgHSV;

    cvtColor(img, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


    Mat imgThresholded;

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH = 0;
    int iHighH = 179;

     int iLowS = 0;
    int iHighS = 255;

     int iLowV = 0;
    int iHighV = 255;

    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

/*    // setting HSV threshold for color red
    int iLowH = 0;
    int iHighH = 55;

    int iLowS = 100;
    int iHighS = 255;

    int iLowV =100;
    int iHighV = 255;
*/
     inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

    //morphological opening (remove small objects from the foreground)
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

     //morphological closing (fill small holes in the foreground)
    dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );



    Mat cimg;
    Mat maskedimg;
    img.copyTo(maskedimg, imgThresholded);
    medianBlur(maskedimg, maskedimg, 5);

    cvtColor(maskedimg, cimg, COLOR_BGR2GRAY);

    vector<Vec3f> circles;
    HoughCircles(cimg, circles, CV_HOUGH_GRADIENT, 1, 10,
                 100, 30, 1, 100 // change the last two parameters
                                // (min_radius & max_radius) to detect larger circles
                 );
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Vec3i c = circles[i];
        circle( maskedimg, Point(c[0], c[1]), c[2], Scalar(0,0,255), 3, CV_AA);
        circle( maskedimg, Point(c[0], c[1]), 2, Scalar(0,255,0), 3, CV_AA);
    }

    imshow("detected circles", maskedimg);
    waitKey();

    return 0;
}
