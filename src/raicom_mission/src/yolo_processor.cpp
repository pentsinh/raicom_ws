#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Int32.h>

class YoloProcessor {
public:
    YoloProcessor(ros::NodeHandle& nh) : nh_(nh), current_cam_index_(0) {
        
        // 1. 订阅相机切换指令
        cam_index_sub_ = nh_.subscribe("/cam_index", 1, &YoloProcessor::camIndexCallback, this);

        // 2. 订阅两个相机的原始图像
        cam0_sub_ = nh_.subscribe("/usb_cam0/image_raw", 1, &YoloProcessor::cam0Callback, this);
        cam1_sub_ = nh_.subscribe("/usb_cam1/image_raw", 1, &YoloProcessor::cam1Callback, this);

        // 3. 发布者：将选中的图像转发给 YOLO 节点
        input_image_pub_ = nh_.advertise<sensor_msgs::Image>("/yolo/input_image", 1);

        ROS_INFO("YoloProcessor Node Initialized with Dynamic Camera Switching.");
    }

private:
    void publishBgrImage(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(
                msg, sensor_msgs::image_encodings::BGR8);
            input_image_pub_.publish(cv_ptr->toImageMsg());
        } catch (const cv_bridge::Exception& e) {
            ROS_ERROR("Failed to convert image to bgr8 for YOLO input: %s", e.what());
        }
    }

    // 处理相机切换指令
    void camIndexCallback(const std_msgs::Int32::ConstPtr& msg) {
        if (msg->data == 0 || msg->data == 1) {
            if (current_cam_index_ != msg->data) {
                current_cam_index_ = msg->data;
                ROS_INFO("Switched to camera index: %d", current_cam_index_);
            }
        } else {
            ROS_WARN("Invalid camera index: %d. Must be 0 or 1.", msg->data);
        }
    }

    // 处理相机 0 图像
    void cam0Callback(const sensor_msgs::ImageConstPtr& msg) {
        if (current_cam_index_ == 0) {
            publishBgrImage(msg);
        }
    }

    // 处理相机 1 图像
    void cam1Callback(const sensor_msgs::ImageConstPtr& msg) {
        if (current_cam_index_ == 1) {
            publishBgrImage(msg);
        }
    }

    ros::NodeHandle nh_;
    int current_cam_index_;
    
    ros::Subscriber cam_index_sub_;
    ros::Subscriber cam0_sub_;
    ros::Subscriber cam1_sub_;
    ros::Publisher input_image_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_processor_node");
    ros::NodeHandle nh;
    YoloProcessor processor(nh);
    ros::spin();
    return 0;
}
