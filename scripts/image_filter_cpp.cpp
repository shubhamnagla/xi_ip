/* 
 The Xillinx Image Proccessing Task 

1.) Geting the image from the robot that is spawned in gazebo
2.) Visualize the robot and the image in Rviz. 
3.) Convert the image from sensory data format to the OpenCV format. 
4.) Run the 2Dfilter convolution and blur the image received. 
5.) View the original image in one window of OpenCv and view the blured image on one window of OpenCv.  

*/

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

class ImageSubscriber : public rclcpp::Node     // Creating the node class by inheriting from rclcpp
{
  private:

    const std::string OPENCV_WINDOW= "Blured Image Window";   // Naming the OpenCV Windows
    const std::string OPENCV_WINDOW2= "Original Image Window";  
    image_transport::Subscriber sub_;   // Subscribing to a topic using image_transport 
    image_transport::Publisher pub_;    // Publishing to a topic using image_transport
    // Creating call back function and implementing OpenCV filter
     void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {   
        /* Pointer used for converting ROS Message
	* to an OpenCV Compatible Image */
        cv_bridge::CvImagePtr cv_ptr; 
    	try
    	{
    	 // Converting the ROS Message.
    	  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	 }
    	catch (cv_bridge::Exception& e)
    	{
    	// If there is a malformed or unsupported encoding we will get an exception.
    	   RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what()); 
    	   return ;
    	 }
    	 
    	 /* Store the values of the OpenCV-compatible image
    	   into the image variable */
    	cv::Mat image = cv_ptr->image;
    	 
    	/* Blurred using kernel
         Initialize matrix with all ones */
    	cv::Mat kernel = cv::Mat::ones(5,5, CV_64F);
    	// Normalize the elements 
    	kernel = kernel / 25;
    	 // creating a image variable to store the blured image.
    	cv::Mat img;
    	filter2D(image, img, -50, kernel, cv::Point(-1,-1), 0, 4); // Applying the 2D filter on to the image 
    	 // Creating seprate variables for resizing the OpenCV Window
    	cv::Mat rs_og;
    	cv::Mat rs_bl;
    	 // Resizing the Display Windows
        cv::resize(cv_ptr->image, rs_og, cv::Size(900,800), cv::INTER_LINEAR);
        cv::resize(img, rs_bl, cv::Size(900,800), cv::INTER_LINEAR); 
        // Creating the OpenCV Display Windows while startup. 
        cv::imshow(OPENCV_WINDOW, rs_bl);
        cv::imshow(OPENCV_WINDOW2, rs_og);
        cv::waitKey(3);

        pub_.publish(cv_ptr->toImageMsg());
      }
      
  public:
    ImageSubscriber() : Node("ImageSubscriber")
    {
    // Naming the OpenCV Windows
      cv::namedWindow(OPENCV_WINDOW);
      cv::namedWindow(OPENCV_WINDOW2);
	
      rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
      // Subscribing to input video feed and Publishing output video 
      pub_ = image_transport::create_publisher(this, "image_rec", custom_qos);
      sub_ = image_transport::create_subscription(this, "/camera/image_raw",
              std::bind(&ImageSubscriber::imageCallback, this, _1), "raw", custom_qos);
     }
     
     ~ImageSubscriber()
     {
     	//Destroying the created windows once the node is shutdown.
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
     
