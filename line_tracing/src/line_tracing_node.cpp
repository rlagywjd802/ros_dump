#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <line_tracing/hsv_thresh.h>
#include <line_tracing/theta_stamped.h>

#include <ctime>
#include <string>
#include <sstream>

#include <math.h>

// integer to string
std::string ToString(int val)
{
  std::stringstream stream;
  if(val/10 == 0){
    stream <<"0"<< val;
  }
  else{
    stream << val;
  }
  
  return stream.str();
}

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_filtered;
  image_transport::Publisher img_pub_thresh;
  image_transport::Publisher image_pub_eroded;
  image_transport::Publisher image_pub_dilated;
  ros::Publisher theta_pub_;
  ros::Subscriber thresh_sub_;
  // int lowH = 0, lowS = 0, lowV = 0;
  // int highH = 255, highS = 255, highV = 255;
  int lowH = 30, lowS = 130, lowV = 25;
  int highH = 140, highS = 255, highV = 255;
  int k_size = 3;
  int e_size = 0;
  int d_size = 0;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_filtered = it_.advertise("/output/filtered", 1);
    img_pub_thresh = it_.advertise("/output/thresholded", 1);
    image_pub_eroded = it_.advertise("/output/eroded", 1);
    image_pub_dilated = it_.advertise("/output/dilated", 1);
    thresh_sub_ = nh_.subscribe("/hsv_thresh", 1, &ImageConverter::threshCb, this);
    theta_pub_ = nh_.advertise<line_tracing::theta_stamped>("/line_angle", 1);
  }
  void threshCb(line_tracing::hsv_thresh msg)
  {
    k_size = msg.k_size;
    e_size = msg.e_size;
    d_size = msg.d_size;
    lowH = msg.lowH;
    highH = msg.highH;
    lowS = msg.lowS;
    highS = msg.highS;
    lowV = msg.lowV;
    highV = msg.highV;
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr cv_ptr_new(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_new1(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_new2(new cv_bridge::CvImage);
    cv_bridge::CvImagePtr cv_ptr_new3(new cv_bridge::CvImage);
    cv::Mat img_orgin;
    cv::Mat img_filtered;
    cv::Mat img_hsv;
    cv::Mat img_thresh;
    cv::Mat img_eroded;
    cv::Mat img_dilated;
    cv::Mat img_contour;

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;


    cv::Mat element_erode = getStructuringElement(cv::MORPH_ELLIPSE,
          cv::Size(2 * e_size + 1, 2 * e_size + 1),
          cv::Point(e_size, e_size) );

    cv::Mat element_dilate = getStructuringElement(cv::MORPH_ELLIPSE,
      cv::Size(2 * d_size + 1, 2 * d_size + 1),
      cv::Point(d_size, d_size) );

    static int seq_i = 0;

    try
    {
      // msg to CVImagePtr
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // image processing
      cv::GaussianBlur(cv_ptr->image, img_filtered, cv::Size(k_size, k_size), 0, 0); // change kernel size
      cv::cvtColor(img_filtered, img_hsv, cv::COLOR_BGR2HSV);
      cv::inRange(img_hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), img_thresh);
      //cv::erode(img_thresh, img_eroded, element_erode);
      //cv::dilate(img_eroded, img_dilated, element_dilate);
      // find contours
      cv::findContours(img_thresh, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
      // draw largest contours
      cv::cvtColor(img_thresh, img_contour, cv::COLOR_GRAY2BGR);
      double largest_area = 0;
      int largest_index = 0;
      for(int i=0; i<contours.size(); i++){
        double a = contourArea(contours[i], false);
        if(a>largest_area){
          largest_area = a;
          largest_index = i;
        }
      }     
      cv::drawContours(img_contour, contours, largest_index, cv::Scalar(0, 0, 255), 5, 8, hierarchy);
      
      // get two point p1, p2
      int rows = img_contour.rows;
      int cols = img_contour.cols;
      int x1 = 0, y1 = 0, n1 = 0;
      int x2 = 0, y2 = 0, n2 = 0; 
      
      for (int i=0; i<contours[largest_index].size(); i++){
        if (contours[largest_index][i].y < (180)){
          x1 += contours[largest_index][i].x;
          y1 += contours[largest_index][i].y;
          n1 += 1;
        }
        else{
          x2 += contours[largest_index][i].x;
          y2 += contours[largest_index][i].y;
          n2 += 1;
        }
      }

      if(n1 == 0 or n2 == 0)
      {
        std::cout<<"divided by zero"<<std::endl;
      }
      else
      {
        x1 /= n1;   y1 /= n1;
        x2 /= n2;   y2 /= n2;
        
        // draw two point p1, p2
        cv::Point p1(x1, y1);
        cv::Point p2(x2, y2);
        cv::circle(img_contour, p1, 8, cv::Scalar(255, 0, 0), -1);
        cv::circle(img_contour, p2, 8, cv::Scalar(255, 0, 0), -1);

        // calculate line angle
        float theta = atan2(float(x1-x2), float(y2-y1));
        theta = theta*180/3.141592; 
        std::cout<<"theta:"<<theta<<std::endl;
        
        // publish line angle
        line_tracing::theta_stamped msg;
        msg.header.seq = seq_i++;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/line_angle";
        msg.data = theta;
        theta_pub_.publish(msg);
      }    
      
      // Mat to msg
      cv_ptr_new->encoding = "bgr8";
      cv_ptr_new->header.stamp = ros::Time::now();
      cv_ptr_new->header.frame_id = "/output";
      cv_ptr_new->image = img_filtered;
      
      cv_ptr_new1->encoding = "mono8";
      cv_ptr_new1->header.stamp = ros::Time::now();
      cv_ptr_new1->header.frame_id = "/output1";
      cv_ptr_new1->image = img_thresh;
      
      cv_ptr_new2->encoding = "mono8";
      cv_ptr_new2->header.stamp = ros::Time::now();
      cv_ptr_new2->header.frame_id = "/output2";
      cv_ptr_new2->image = img_dilated;

      cv_ptr_new3->encoding = "bgr8";
      cv_ptr_new3->header.stamp = ros::Time::now();
      cv_ptr_new3->header.frame_id = "/output3";
      cv_ptr_new3->image = img_contour;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Output modified video stream
    image_pub_filtered.publish(cv_ptr_new->toImageMsg());
    img_pub_thresh.publish(cv_ptr_new1->toImageMsg());
    image_pub_eroded.publish(cv_ptr_new2->toImageMsg());
    image_pub_dilated.publish(cv_ptr_new3->toImageMsg());
  }
};

std::string get_time(){
  time_t now = time(0);
  tm *ltm = localtime(&now);
  int year = 1900+ltm->tm_year;
  int month = 1+ltm->tm_mon;
  int day = ltm->tm_mday;
  int hour = ltm->tm_hour;
  int min = ltm->tm_min;
  int sec = ltm->tm_sec;

  std::string s = ToString(year)+"_"+ToString(month)+"_"+ToString(day)+"_"+ToString(hour)+"_"+ToString(min)+"_"+ToString(sec);
  return s; 
}

int main(int argc, char** argv)
{
  std::cout<<get_time()<<std::endl;
  ros::init(argc, argv, "line_tracing");
  ImageConverter ic;
  ros::spin();
  return 0;
}