#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.
 
using namespace std::chrono_literals;
 
class MinimalImageProcessor : public rclcpp::Node {
public:
  MinimalImageProcessor() : Node("opencv_image_processor") {
    // Subscriber per il topic della telecamera
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/videocamera", 10,
        std::bind(&MinimalImageProcessor::image_callback, this, std::placeholders::_1));

    // Publisher per il topic dell'immagine processata
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("/videocamera/processed_image", 10);
  }
 
private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Convertire il messaggio ROS in un'immagine OpenCV
    cv::Mat input_image;
    try {
      input_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Errore nella conversione dell'immagine: %s", e.what());
      return;
    }

    // Convertire in scala di grigi per il rilevamento dei blob
    cv::Mat gray_image;
    cv::cvtColor(input_image, gray_image, cv::COLOR_BGR2GRAY);

    // Configurare il rilevatore di blob
    cv::SimpleBlobDetector::Params params;
    params.minThreshold = 0;
    params.maxThreshold = 255;
    params.filterByArea = false;
    params.minArea = 100;
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
    params.filterByConvexity = true;
    params.minConvexity = 0.9;
    params.filterByInertia = true;
    params.minInertiaRatio = 0.5;

    // Crea il rilevatore
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    // Rileva i blob
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray_image, keypoints);

    // Disegna i blob rilevati
    cv::Mat output_image;
    cv::drawKeypoints(input_image, keypoints, output_image, cv::Scalar(0, 0, 255),
                      cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
                      
    imshow("keypoints", output_image);

    // Converti l'immagine processata in un messaggio ROS
    auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", output_image).toImageMsg();

    // Pubblica l'immagine processata
    publisher_->publish(*processed_msg);
    RCLCPP_INFO(this->get_logger(), "Immagine processata e pubblicata con %lu blob rilevati", keypoints.size());
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalImageProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
