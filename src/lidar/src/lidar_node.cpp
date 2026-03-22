#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <memory>
#include <chrono>
#include <thread>
#include <random>
#include <iostream>
#include <algorithm>

class RosPointCloudBagPublisher : public rclcpp::Node
{
public:
    RosPointCloudBagPublisher(const rclcpp::NodeOptions & options)
        : Node("ros_pointcloud_bag_publisher",options)
    {
        pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "cloud_topic", 100);
        
        bag_thread_ = std::thread(&RosPointCloudBagPublisher::read_and_publish_bag, this);
    }

private:
    void read_and_publish_bag()
    {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_file_path_;
        storage_options.storage_id = "sqlite3";

        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";

        rosbag2_cpp::readers::SequentialReader reader;
        reader.open(storage_options, converter_options);
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> pc_serializer;
        int frame_count = 0;

        while (reader.has_next() && rclcpp::ok())
        {
            auto bag_msg = reader.read_next();

            sensor_msgs::msg::PointCloud2::SharedPtr ros_pc_msg = 
                std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
            pc_serializer.deserialize_message(&serialized_msg, ros_pc_msg.get());

            pc_publisher_->publish(*ros_pc_msg);

            auto start = std::chrono::steady_clock::now();
            std::chrono::duration<double> delay(1000.0/30/1000);

            while(std::chrono::steady_clock::now()-start < delay){
                
            }

            frame_count++;

        }

        RCLCPP_INFO(this->get_logger(), "bag读取完成 | 共发布%d帧点云", frame_count);
    }

    std::string bag_file_path_ = "/home/as/hw_lidar/hw_lidar_0.db3";
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    std::thread bag_thread_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosPointCloudBagPublisher)

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<RosPointCloudBagPublisher>("/home/as/hw_lidar/hw_lidar_0.db3");
//     rclcpp::spin(node);

//     rclcpp::shutdown();
//     return 0;
// }