#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <tesseract/baseapi.h>
#include <leptonica/allheaders.h>
#include <string>
#include <std_msgs/String.h">
#include <locale>

class TrafficSignDetector {
    ros::Publisher string_pub;
    bool speed_limit;
    tesseract::TessBaseAPI api;

    void TrafficSignDetector(ros::NodeHandle &n){
        string_pub = n.advertise<std_msgs::Bool>("/traffic_sign/speed_limit", 1000);
        // Initialize tesseract-ocr with English, without specifying tessdata path
        if (api.Init(NULL, "eng")) {
            fprintf(stderr, "Could not initialize tesseract.\n");
            exit(1);
        }
    }



    void callback(const sensor_msgs::ImageConstPtr &msg) {
        // Convert the image into something opencv can handle.
        cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;
        if (normal_speed_limit_detection(frame)){
            speed_limit = true;

        } else if (remove_speed_limit_detection(frame)) {
            speed_limit = false;
        }
        std::msgs::String msg;
        mgs.data = speed_limit;
        string_pub.publish(speed_limit);
    }

    bool normal_speed_limit_detection(cv::Mat &frame){
        bool is_detected = false;

        // get red mask
        cv::Mat hsv, mask0, mask1, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_RGB2HSV);
        cv::inRange(hsv, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask0);
        cv::inRange(hsv, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask1);
        mask = mask0 + mask1;

        // get masked image
        cv::Mat masked_frame, masked_gray;
        cv::bitwise_and(frame, mask, masked_frame);
        cv::cvtColor(masked_frame, masked_gray, cv::COLOR_RGB2GRAY);

        // detect circles
        vector <cv::Vec3f> circles;
        cv::HoughCircles(masked_gray, circles, cv::HOUGH_GRADIENT, 1, 100, 200, 40, 30, 300);

        // crop image
        if (circles.size()) {
            cv::Mat gray, thresh;
            cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
            cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU)
            crop_image(circles, thresh, thresh);

            // digit detection
            std::string result = digit_detection(thresh);
            std::string digits;
            std::locale loc;
            for (int i=0; i<result.length; ++i){
                if (isdigit(result[i], loc)){
                    digits.append(result[i]);
                }
            }

            // check result
            if (digits.find("10") != std::string::npos) {
                is_detected = true;
            }
        }
        return is_detected;
    }

    bool remove_speed_limit_detection(cv::Mat &frame){
        bool is_detected = false;

        // detect circles
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
        vector <cv::Vec3f> circles;
        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, 100, 200, 150, 30, 300);

        // crop image
        if (circles.size()) {
            cv::Mat thresh;
            cv::threshold(gray, thresh, 0, 255, cv::THRESH_BINARY_INV | cv::THRESH_OTSU)
            crop_image(circles, thresh, thresh);

            // remove lines
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(13, 13));
            cv::Mat opening, inv_img, blur_img;
            cv::morphologyEx(thresh, opening, cv::MORPH_OPEN, kernel);
            cv::bitwise_not(opening, inv_img);
            cv::medianBlur(inv_img, blur_img, 5);

            // digit detection
            std::string result = digit_detection(blur_img);
            std::string digits;
            std::locale loc;
            for (int i=0; i<result.length; ++i){
                if (isdigit(result[i], loc)){
                    digits.append(result[i]);
                }
            }

            // check result
            if (digits.find("10") != std::string::npos) {
                is_detected = true;
            }
        }
        return is_detected;
    }

    void crop_image(vector <cv::Vec3f> &circles, cv::Mat &input_img, cv::Mat &output_img) {
        int max_ypr, max_xpr, min_ymr = 1000, min_xmr = 1000;
        float scale = 0.7;
        for (size_t i = 0; i < circles.size(); i++) {
            cv::Vec3i c = circles[i];
            int x = c[0], y = c[1], r = c[2];
            float tmp = y - scale * r;
            if (0 < tmp) && (tmp < min_ymr) {
                min_ymr = tmp;
            }
            tmp = x - scale * r;
            if (0 < tmp) && (tmp < min_xmr) {
                min_xmr = tmp;
            }
            tmp = y + scale * r;
            if (max_ypr < tmp) && (tmp < input_img.size[0]) {
                max_ypr = tmp;
            }
            tmp = x + scale * r;
            if (max_xpr < tmp) && (tmp < input_img.size[1]) {
                max_xpr = tmp;
            }
        }
        cv::Rect roi(min_xmr, min_ymr, max_xpr-min_xmr, max_ypr-min_ymr);
        output_img = input_img(roi);
    }

    std::string digit_detection(cv::Mat &img){
        cv::Mat final_img;
        cv::cvtColor(img, final_img, CV_BGR2RGBA);
        cv::resize(final_img, final_img, cv::Size(100, 100));
        api.SetPageSegMode(tesseract::PSM_CIRCLE_WORD);
        api.SetImage(img.data, img.cols, img.rows, 4, 4*img.cols);
        std::string output = api.GetUTF8Text();
        return output;
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "traffic_sign_detector");
    ros::NodeHandle n;
    ros::spin();
}

