#include <memory>
#include <stdlib.h>
#include <functional>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
using std::placeholders::_1;

class ImageSubscriber : public rclcpp::Node
{
  private:
     const std::string OPENCV_WINDOW= "Blured Image Window";
     const std::string OPENCV_WINDOW2= "Original Image Window";
    image_transport::Subscriber sub_;
    image_transport::Publisher pub_;
     void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        cv_bridge::CvImagePtr cv_ptr;
    	try
    	{
    	  cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    	 }
    	catch (cv_bridge::Exception& e)
    	{
    	   RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    	   return ;
    	 }
    	 
    	 cv::Mat image = cv_ptr->image;
    	 cv::Mat kernel = cv::Mat::ones(5,5, CV_64F);
    	 
    	 kernel = kernel / 25;
    	 cv::Mat img;
    	 filter2D(image, img, -100 , kernel, cv::Point(-1,-1), 0, 4);
    	 cv::Mat rs_og;
    	 cv::Mat rs_bl;
        cv::resize(cv_ptr->image, rs_og, cv::Size(800,800), cv::INTER_LINEAR);
        cv::resize(img, rs_bl, cv::Size(800,800), cv::INTER_LINEAR); 
        cv::imshow(OPENCV_WINDOW, rs_bl);
        cv::imshow(OPENCV_WINDOW2, rs_og);
        cv::waitKey(3);

        pub_.publish(cv_ptr->toImageMsg());
      }
      
  public:
    ImageSubscriber() : Node("ImageSubscriber")
    {
      cv::namedWindow(OPENCV_WINDOW);
      
      cv::namedWindow(OPENCV_WINDOW2);

      rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
      pub_ = image_transport::create_publisher(this, "image_rec", custom_qos);
      sub_ = image_transport::create_subscription(this, "/camera/image_raw",
              std::bind(&ImageSubscriber::imageCallback, this, _1), "raw", custom_qos);
     }
     
     ~ImageSubscriber()
     {
     	cv::destroyWindow(OPENCV_WINDOW);
     	cv::destroyWindow(OPENCV_WINDOW2);
     }
};
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageSubscriber>());
  rclcpp::shutdown();
  return 0;
}
     
