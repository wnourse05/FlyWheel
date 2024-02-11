#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>

std::string gstreamer_pipeline(int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method, int sensor_id) {
    return "nvarguscamerasrc sensor-id=" + std::to_string(sensor_id) + " ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("/combined_camera/image", 1);

    int capture_width = 1640;
    int capture_height = 1232;
    int display_width = 1640;
    int display_height = 1232;
    int framerate = 30;
    int flip_method = 0;
    int final_width = 64;
    int final_height = 24;
    
    // Sensor IDs for the two cameras (Assuming they are different)
    int sensor_id1 = 0; // Replace with actual sensor ID for camera 1
    int sensor_id2 = 1; // Replace with actual sensor ID for camera 2

    std::string pipeline1 = gstreamer_pipeline(capture_width,
                                                capture_height,
                                                display_width,
                                                display_height,
                                                framerate,
                                                flip_method,
                                                sensor_id1);
    std::cout << "Using pipeline for camera 1: \n\t" << pipeline1 << "\n";

    cv::VideoCapture cap1(pipeline1, cv::CAP_GSTREAMER);
    if (!cap1.isOpened()) {
        std::cout << "Failed to open camera 1." << std::endl;
        return (-1);
    }

    std::string pipeline2 = gstreamer_pipeline(capture_width,
                                                capture_height,
                                                display_width,
                                                display_height,
                                                framerate,
                                                flip_method,
                                                sensor_id2);
    std::cout << "Using pipeline for camera 2: \n\t" << pipeline2 << "\n";

    cv::VideoCapture cap2(pipeline2, cv::CAP_GSTREAMER);
    if (!cap2.isOpened()) {
        std::cout << "Failed to open camera 2." << std::endl;
        return (-1);
    }

    //cv::namedWindow("Combined Camera View", cv::WINDOW_NORMAL);
    cv::Mat img_combined(display_height, display_width * 2, CV_8UC3);
    cv::Mat green_channel;
    cv::Mat img_combined_resized;

    //std::cout << "Hit ESC to exit" << "\n";
    while (ros::ok()) {
    	std::cout << "Initialize " << std::endl;
    	cv::Mat img1, img2;
        if (!cap1.read(img1)) {
            std::cout << "Capture from camera 1 read error" << std::endl;
            break;
        }

        if (!cap2.read(img2)) {
            std::cout << "Capture from camera 2 read error" << std::endl;
            break;
        }
        //if (img1.empty()) {
	//    std::cout << "Empty image received from camera 1" << std::endl;
	//    break;
	//}

	//if (img2.empty()) {
	//    std::cout << "Empty image received from camera 2" << std::endl;
	//    break;
	//}

	// Record timestamp just after capturing the image
        //auto capture_time = std::chrono::high_resolution_clock::now();
        
    	std::cout << "Copy" << std::endl;
        // Copy camera 1 frame to the left side of the combined image
        img2.copyTo(img_combined(cv::Rect(0, 0, display_width, display_height)));

        // Copy camera 2 frame to the right side of the combined image
        img1.copyTo(img_combined(cv::Rect(display_width, 0, display_width, display_height)));
        
    	std::cout << "Split " << std::endl;
        // Split the combined image into its color channels
        std::vector<cv::Mat> channels;
        cv::split(img_combined, channels);
        
    	std::cout << "Green " << std::endl;
        // Extract the green channel
        green_channel = channels[1]; // Green channel is index 1 (BGR order)
        
    	std::cout << "Resize " << std::endl;
    	// Resize the combined image
        //cv::resize(green_channel, img_combined_resized, cv::Size(final_width, final_height), 0, 0, cv::INTER_AREA);
        cv::resize(green_channel, img_combined_resized, cv::Size(final_width, final_height), 0, 0, cv::INTER_NEAREST);
        
        // Convert OpenCV image to ROS message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_combined_resized).toImageMsg();
        
        // Publish the ROS message
        pub.publish(msg);

	std::cout << "Spin " << std::endl;
        ros::spinOnce();
        
        // Record timestamp just before displaying the image
        //auto end_time = std::chrono::high_resolution_clock::now();

        // Calculate the time taken for image processing and display
        //auto capture_to_display_latency = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - capture_time).count();

        std::cout << "Live " << std::endl;
        
        // Display the combined image
        //cv::imshow("Combined Camera View", img_combined_resized);
        
        //int keycode = cv::waitKey(10) & 0xff;
        //if (keycode == 27) break;
    }

    cap1.release();
    cap2.release();
    //cv::destroyAllWindows();
    return 0;
}

