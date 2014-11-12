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
#include <cmath>


using namespace cv;
using namespace std;



#define HZ 10
static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_CONTROL = "Control Window";
//static const std::string OPENCV_WINDOW_CC = "Connected Component Window";

class Shapedetector
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

private:
    static double angle(cv::Point pt1, cv::Point pt2, cv::Point pt0)
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
    }

    void setLabel(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
    {
        int fontface = cv::FONT_HERSHEY_SIMPLEX;
        double scale = 0.4;
        int thickness = 1;
        int baseline = 0;

        cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
        cv::Rect r = cv::boundingRect(contour);

        cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
        cv::rectangle(im, pt + cv::Point(0, baseline), pt + cv::Point(text.width, -text.height), CV_RGB(255,255,255), CV_FILLED);
        cv::putText(im, label, pt, fontface, scale, CV_RGB(0,0,0), thickness, 8);
    }

public:
    Shapedetector()
    : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
          &Shapedetector::imageColorCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        isInitializedColor = false;

        cv::namedWindow(OPENCV_WINDOW);
        cv::namedWindow(OPENCV_WINDOW_CONTROL);
    }

    ~Shapedetector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
        cv::destroyWindow(OPENCV_WINDOW_CONTROL);
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



    void getDirection()
    {
        if(isInitializedColor == false) return;

        // Convert to grayscale
        cv::Mat src;
        cv::Mat gray;
        src = cv_ptr->image;
        cv::cvtColor(src, gray, CV_BGR2GRAY);

        // Use Canny instead of threshold to catch squares with gradient shading
        cv::Mat bw;
        cv::Canny(gray, bw, 0, 50, 5);

        // Find contours
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(bw.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

        std::vector<cv::Point> approx;
        cv::Mat dst = src.clone();

        for (int i = 0; i < contours.size(); i++)
        {
            // Approximate contour with accuracy proportional
            // to the contour perimeter
            cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true)*0.02, true);

            // Skip small or non-convex objects
            if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
                continue;

            if (approx.size() == 3)
            {
                setLabel(dst, "TRI", contours[i]);    // Triangles
            }
            else if (approx.size() >= 4 && approx.size() <= 6)
            {
                // Number of vertices of polygonal curve
                int vtc = approx.size();

                // Get the cosines of all corners
                std::vector<double> cos;
                for (int j = 2; j < vtc+1; j++)
                    cos.push_back(angle(approx[j%vtc], approx[j-2], approx[j-1]));

                // Sort ascending the cosine values
                std::sort(cos.begin(), cos.end());

                // Get the lowest and the highest cosine
                double mincos = cos.front();
                double maxcos = cos.back();

                // Use the degrees obtained above and the number of vertices
                // to determine the shape of the contour
                if (vtc == 4 && mincos >= -0.1 && maxcos <= 0.3)
                    setLabel(dst, "RECT", contours[i]);
                else if (vtc == 5 && mincos >= -0.34 && maxcos <= -0.27)
                    setLabel(dst, "PENTA", contours[i]);
                else if (vtc == 6 && mincos >= -0.55 && maxcos <= -0.45)
                    setLabel(dst, "HEXA", contours[i]);
            }
            else
            {
                // Detect and label circles
                double area = cv::contourArea(contours[i]);
                cv::Rect r = cv::boundingRect(contours[i]);
                int radius = r.width / 2;

                if (std::abs(1 - ((double)r.width / r.height)) <= 0.2 &&
                    std::abs(1 - (area / (CV_PI * std::pow(radius, 2)))) <= 0.2)
                    setLabel(dst, "CIR", contours[i]);
            }
        }

        cv::imshow(OPENCV_WINDOW, src);
        cv::imshow(OPENCV_WINDOW_CONTROL, dst);
        cv::waitKey(0);
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
    Shapedetector ic;
    ros::Rate loop_rate(HZ);

    while(stop == false){
        signal(SIGINT, &handler);
        ros::spinOnce();
        ic.getDirection();
        cv::waitKey(3);
        loop_rate.sleep();
    }
    ic.~Shapedetector();
    return 0;
}