#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    // Specify the relative path to the image
    std::string image_path = "image.png"; // Adjust this based on your directory structure

    // Load the image
    cv::Mat img = cv::imread(image_path);
    if (img.empty()) {
        std::cerr << "Error: Could not open or find the image!" << std::endl;
        return -1;
    }

    // Convert the image to grayscale
    cv::Mat gray_img;
    cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

    // Apply Gaussian blur to reduce noise and improve circle detection
    cv::GaussianBlur(gray_img, gray_img, cv::Size(9, 9), 2);

    // Vector to store detected circles
    std::vector<cv::Vec3f> circles;

    // Detect circles using Hough Circle Transform
    cv::HoughCircles(
        gray_img,
        circles,
        cv::HOUGH_GRADIENT,
        1,            // Inverse ratio of the accumulator resolution to the image resolution
        20,           // Minimum distance between detected centers
        50,           // Higher threshold for the Canny edge detector
        30,           // Lower threshold for center detection
        0,            // Minimum circle radius (0 means no minimum)
        0             // Maximum circle radius (0 means no maximum)
    );

    // Draw the detected circles on the original image
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3f c = circles[i];
        // Draw the outer circle
        cv::circle(img, cv::Point(cvRound(c[0]), cvRound(c[1])), cvRound(c[2]), cv::Scalar(0, 255, 0), 2);
        // Draw the center of the circle
        cv::circle(img, cv::Point(cvRound(c[0]), cvRound(c[1])), 2, cv::Scalar(0, 0, 255), 3);
    }

    // Display the results
    cv::imshow("Detected Circles", img);
    cv::waitKey(0); // Wait for a keystroke in the window

    return 0;
}






// #include <memory>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>

// class ShapeDetectionNode : public rclcpp::Node {
// public:
//     ShapeDetectionNode() : Node("shape_detection_node") {
//         // Subscription to the Image topic
//         subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/camera_sensor/image_raw",  // Replace with your image topic
//             10,
//             std::bind(&ShapeDetectionNode::listener_callback, this, std::placeholders::_1)
//         );
//     }

// private:
//     void listener_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
//         // Convert ROS Image message to OpenCV format
//         cv_bridge::CvImagePtr cv_ptr;
//         try {
//             cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
//         } catch (cv_bridge::Exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//             return;
//         }

//         // Get the OpenCV image
//         cv::Mat img = cv_ptr->image;

//         // Convert the image to grayscale
//         cv::Mat gray_img;
//         cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);

//         // Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) to enhance contrast
//         cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(2.0, cv::Size(8, 8));
//         cv::Mat enhanced_img;
//         clahe->apply(gray_img, enhanced_img);

//         // Convert the grayscale image back to BGR for color display
//         cv::Mat cimg;
//         cv::cvtColor(enhanced_img, cimg, cv::COLOR_GRAY2BGR);

//         // --- Circle Detection ---
//         std::vector<cv::Vec3f> circles;
//         cv::HoughCircles(
//             enhanced_img, 
//             circles, 
//             cv::HOUGH_GRADIENT, 
//             1, 
//             20, 
//             50, 
//             30, 
//             0, 
//             0
//         );

//         // If circles are detected, draw them with green color
//         if (!circles.empty()) {
//             for (size_t i = 0; i < circles.size(); i++) {
//                 cv::Vec3i c = circles[i];
//                 // Draw the outer circle
//                 cv::circle(cimg, cv::Point(c[0], c[1]), c[2], cv::Scalar(0, 255, 0), 2);
//                 // Draw the center of the circle
//                 cv::circle(cimg, cv::Point(c[0], c[1]), 2, cv::Scalar(0, 0, 255), 3);
//             }
//         }

//         // --- Rectangle Detection ---
//         // Detect edges using Canny edge detector
//         cv::Mat edges;
//         cv::Canny(enhanced_img, edges, 50, 150);

//         // Find contours in the edge-detected image
//         std::vector<std::vector<cv::Point>> contours;
//         cv::findContours(edges, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

//         for (const auto& contour : contours) {
//             // Approximate the contour to reduce the number of points
//             std::vector<cv::Point> approx;
//             cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);
//             if (approx.size() == 4) {  // If the approximated contour has 4 points, it's likely a rectangle
//                 cv::Rect rect = cv::boundingRect(approx);
//                 // Draw the bounding rectangle with blue color
//                 cv::rectangle(cimg, rect, cv::Scalar(255, 0, 0), 2);
//             }
//         }

//         // Display the result
//         cv::imshow("Detected Shapes", cimg);
//         cv::waitKey(1);
//     }

//     // ROS2 subscription object
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
// };

// int main(int argc, char *argv[]) {
//     // Initialize ROS2
//     rclcpp::init(argc, argv);

//     // Create the node
//     auto shape_detection_node = std::make_shared<ShapeDetectionNode>();

//     // Keep spinning the node until it is shut down
//     rclcpp::spin(shape_detection_node);

//     // Shutdown ROS2
//     rclcpp::shutdown();
//     return 0;
// }
