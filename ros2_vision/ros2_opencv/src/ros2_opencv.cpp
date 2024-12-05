#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using namespace std::chrono_literals;
using namespace cv;

class ImageProcessor : public rclcpp::Node{
public:
    ImageProcessor() : Node("image_processor"){
        publisher_ =
            this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);

        // timer_ = this->create_wall_timer(
        //     500ms, std::bind(&MinimalImagePublisher::timer_callback, this));

        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/videocamera",10,
            std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));

    }

private:


    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Immagine ricevuta dal topic /videocamera.");
        received_image_ = msg;
        processImage();

    }

    void processImage(){
        Mat img = cv_bridge::toCvCopy(received_image_, "bgr8")->image;

        SimpleBlobDetector::Params params;

        // Change thresholds
	    params.minThreshold = 10;
	    params.maxThreshold = 200;

        // Filter by Circularity
	    params.filterByCircularity = true;
	    params.minCircularity = 0.9;

        // Filter by Convexity
	    params.filterByConvexity = true;
	    params.minConvexity = 0.87;

        std::vector<KeyPoint> keypoints;

        //CV2
        // SimpleBlobDetector detector(params);

        // detector.detect(img, keypoints);

        //CV3
        // Set up detector with params
	    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);   

	    // Detect blobs
	    detector->detect( img, keypoints);

        Mat img_with_keypoints;
        
        drawKeypoints( img, keypoints, img_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
        
        sensor_msgs::msg::Image::SharedPtr msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_with_keypoints)
               .toImageMsg();
 
        // Publish the image to the topic defined in the publisher
        publisher_->publish(*msg_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    sensor_msgs::msg::Image::SharedPtr received_image_;


};