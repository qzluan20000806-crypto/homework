#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "chrono"

class VirtualCamNode : public rclcpp::Node{
public:
    VirtualCamNode(const rclcpp::NodeOptions & options) : Node("virtual_cam_node", options){
        image_pub_ = image_transport::create_publisher(this, "/virtual_cam/image_raw");
        cap_.open("/home/as/video.mp4");
        if(!cap_.isOpened()){
            RCLCPP_ERROR(this->get_logger(), "没打开video.mp4");
            rclcpp::shutdown();
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&VirtualCamNode::publish_frame, this)
        );
    }
private:
    void publish_frame(){
        cv::Mat frame;
        cap_ >> frame;
        if(frame.empty()){
            return ;
        }
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", frame
        ).toImageMsg();
        msg->header.frame_id = "camera_link";
        msg->header.stamp = this->get_clock()->now();
        image_pub_.publish(msg);
    }
    cv::VideoCapture cap_;
    image_transport::Publisher image_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(VirtualCamNode);

// int main(int argc, char **argv){
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<VirtualCamNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }

