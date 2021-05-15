#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <std_msgs/Bool.h>
#include <chrono>
#include <iostream>

class TrafficSignDetector {
private:
    ros::NodeHandle n;
    ros::Subscriber img_sub;
    ros::Publisher bool_pub;
    image_transport::Publisher img_pub_1;
    image_transport::Publisher img_pub_2;

    bool pub_speed_limit = false;
    tesseract::TessBaseAPI ocr;
    std::chrono::steady_clock::time_point begin, end;

    const int min_interval = 3;
    const float crop_scale = 0.75;
    const float sign_scale = 0.5;
    const int morph_ksize = 7;
    const int median_ksize = 5;

    void callback(const sensor_msgs::ImageConstPtr &msg) {
        // Convert the image into something opencv can handle.
        cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        if (normal_speed_limit_detection(frame)) {
            begin = std::chrono::steady_clock::now();
            pub_speed_limit = true;
        } else if (pub_speed_limit) {
            end = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(end - begin).count() > min_interval &&
                remove_speed_limit_detection(frame)) {
                pub_speed_limit = false;
            }
        }
        std_msgs::Bool bool_msg;
        bool_msg.data = pub_speed_limit;
        bool_pub.publish(bool_msg);
    }

    bool normal_speed_limit_detection(cv::Mat &frame) {
        // get red mask
        cv::Mat hsv, mask0, mask1, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask0);
        cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(179, 255, 255), mask1);
        mask = mask0 | mask1;

        // get masked image
        cv::Mat masked_frame, masked_gray;
        cv::bitwise_and(frame, frame, masked_frame, mask);
        cv::cvtColor(masked_frame, masked_gray, cv::COLOR_BGR2GRAY);

        // detect circles
        std::vector <cv::Vec3f> circles;
        cv::HoughCircles(masked_gray, circles, cv::HOUGH_GRADIENT, 1, 100, 200, 30, 30, 300);

        // crop image
        if (!circles.empty()) {
            cv::Mat gray, thresh;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
            cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
            crop_image(circles, thresh, thresh);

            // check if the car is close to the sign
            if ((float) thresh.rows > (float) frame.rows * sign_scale) {
                // option: visualize the final image
                img_pub_1.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", thresh).toImageMsg());

                // digit detection
                std::string digits = digit_detection(thresh);

                // check result
                if (digits.find("10") != std::string::npos
                    || digits.find("TD") != std::string::npos
                    || digits.find("40") != std::string::npos
                    || digits.find("T0") != std::string::npos) {
                    return true;
                }
            }
        }
        return false;
    }

    bool remove_speed_limit_detection(cv::Mat &frame) {
        // detect circles
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        std::vector <cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 100, 200, 150, 30, 300);

        // crop image
        if (!circles.empty()) {
            cv::Mat thresh;
            cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU);
            crop_image(circles, thresh, thresh);

            // check if the car is close to the sign
            if ((float) thresh.rows > (float) frame.rows * sign_scale) {
                // remove lines
                cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(morph_ksize, morph_ksize));
                cv::Mat opening, inv_img, blur_img;
                cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel);
                cv::bitwise_not(opening, inv_img);
                cv::medianBlur(inv_img, blur_img, median_ksize);

                // option: publish final image
                img_pub_2.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", blur_img).toImageMsg());

                // digit detection
                std::string digits = digit_detection(blur_img);

                // check result
                if (digits.find("10") != std::string::npos
                    || digits.find("TD") != std::string::npos
                    || digits.find("40") != std::string::npos
                    || digits.find("T0") != std::string::npos) {
                    return true;
                }
            }
        }
        return false;
    }

    void crop_image(std::vector <cv::Vec3f> &circles, cv::Mat &input_img, cv::Mat &output_img) const {
        int max_ypr = 0, max_xpr = 0, min_ymr = 100000, min_xmr = 100000;
        for (auto &circle : circles) {
            cv::Vec3i c = circle;
            int x = c[0], y = c[1], r = c[2];
            int tmp = y - int(crop_scale * float(r));
            if (0 < tmp && tmp < min_ymr) {
                min_ymr = tmp;
            }
            tmp = x - int(crop_scale * float(r));
            if (0 < tmp && tmp < min_xmr) {
                min_xmr = tmp;
            }
            tmp = y + int(crop_scale * float(r));
            if (max_ypr < tmp && tmp < input_img.size[0]) {
                max_ypr = tmp;
            }
            tmp = x + int(crop_scale * float(r));
            if (max_xpr < tmp && tmp < input_img.size[1]) {
                max_xpr = tmp;
            }
        }
        output_img = input_img(cv::Range(min_ymr, max_ypr), cv::Range(min_xmr, max_xpr));
    }

    std::string digit_detection(cv::Mat &img) {
        // convert mat to pix
        cv::Mat final_img;
        cv::resize(img, final_img, cv::Size(100, 100));

        // get digits
        Pix *pix = pixCreate(final_img.size().width, final_img.size().height, 8);
        for (int y = 0; y < final_img.rows; ++y) {
            for (int x = 0; x < final_img.cols; ++x) {
                pixSetPixel(pix, x, y, (l_uint32) final_img.at<uchar>(y, x));
            }
        }
        cv::cvtColor(final_img, final_img, CV_GRAY2BGRA);
        ocr.SetImage(pix);
        std::string output = ocr.GetUTF8Text();

        std::cout << output << std::endl;

        delete pix;
        std::string digits;
        std::locale loc;
        for (char &i : output) {
            if (isdigit(i, loc)) {
                digits += i;
            }
        }
        return digits;
    }

public:
    TrafficSignDetector() {
        // subscriber and publisher
        img_sub = n.subscribe("/camera/image_raw", 1, &TrafficSignDetector::callback, this);
        bool_pub = n.advertise<std_msgs::Bool>("/traffic_sign/speed_limit", 1);
        image_transport::ImageTransport it(n);
        img_pub_1 = it.advertise("/traffic_sign/image1", 1);
        img_pub_2 = it.advertise("/traffic_sign/image2", 1);

        // Initialize tesseract-ocr
        if (ocr.Init(nullptr, "eng")) {
            fprintf(stderr, "Could not initialize tesseract.\n");
            exit(1);
        }
        ocr.SetPageSegMode(tesseract::PSM_CIRCLE_WORD);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "traffic_sign_detector");
    TrafficSignDetector trafficSignDetector;
    ros::spin();
    return 0;
}

