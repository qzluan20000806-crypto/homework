#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include <memory>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>
#include <flann/flann.h>
#include <algorithm>
#include <chrono>

struct VoxelIndex {
    int x;
    int y;
    int z;
    bool operator==(const VoxelIndex& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelIndex2D {
    int x;
    int y;
    bool operator==(const VoxelIndex2D& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<> struct hash<VoxelIndex> {
        size_t operator()(const VoxelIndex& idx) const {
            size_t seed = 0;
            seed ^= hash<int>()(idx.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash<int>()(idx.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash<int>()(idx.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
    template<> struct hash<VoxelIndex2D> {
        size_t operator()(const VoxelIndex2D& idx) const {
            size_t seed = 0;
            seed ^= hash<int>()(idx.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= hash<int>()(idx.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            return seed;
        }
    };
}

struct VoxelPoints {
    std::vector<float> x_list;
    std::vector<float> y_list;
    std::vector<float> z_list;
    void getMean(float& x, float& y, float& z) const {
        x = y = z = 0.0f;
        int n = x_list.size();
        if (n == 0) return;
        for (float v : x_list) x += v;
        for (float v : y_list) y += v;
        for (float v : z_list) z += v;
        x /= n; y /= n; z /= n;
    }
};

class DBSCAN_component : public rclcpp::Node{
public:
    DBSCAN_component(const rclcpp::NodeOptions &options) : Node("DBSCAN_Node", options){
        RCLCPP_INFO(this->get_logger(), "DBSCAN component started (single thread)");
        rclcpp::QoS qos(1);
        qos.keep_last(1).durability_volatile();
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/patchworkpp/nonground", qos, 
            std::bind(&DBSCAN_component::callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dbscan", qos);
    }

private:
    double eps = 1;
    int min_pts = 3;
    double voxel_size = 0.01;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    bool check(std::vector<float>& z){
        std::sort(z.begin(), z.end());
        bool consistent = 1;
        for(int i = 1; i < z.size(); ++i){
            if(z[i] - z[i-1] > 0.2f){
                consistent = 0;
                break;
            }
        }
        return (z[z.size()-1] <= -1.0f && z[0] <= -1.0f);
    }

    void xyzcheck(const std::vector<float>& input, std::vector<float>& output, double voxel_s) {
        int point_num = input.size() / 3;
        if (point_num == 0) return;

        std::unordered_map<VoxelIndex2D, VoxelPoints> voxel_map;
        voxel_map.reserve(std::max(16, point_num / 8));

        for (int i = 0; i < point_num; ++i) {
            int idx = i * 3;
            float x = input[idx], y = input[idx+1], z = input[idx+2];
            VoxelIndex2D vi{static_cast<int>(std::floor(x/voxel_s)),
                        static_cast<int>(std::floor(y/voxel_s))};
            auto& vp = voxel_map[vi];
            vp.x_list.push_back(x);
            vp.y_list.push_back(y);
            vp.z_list.push_back(z);
        }

        output.resize(voxel_map.size() * 3);
        int cnt = 0;
        for (auto& pair : voxel_map) {
            //RCLCPP_INFO(this->get_logger(), "%d,%d", pair.first.x, pair.first.y);
            if(check(pair.second.z_list)){
                for(int i = 0; i < pair.second.x_list.size(); ++i){
                    if(pair.second.z_list[i] > -1.0f) continue;
                    output.push_back(pair.second.x_list[i]);
                    output.push_back(pair.second.y_list[i]);
                    output.push_back(pair.second.z_list[i]);
                }
            }
        }
    }

    void voxelDownsample(const std::vector<float>& input, std::vector<float>& output) {
        int point_num = input.size() / 3;
        if (point_num == 0) return;

        std::unordered_map<VoxelIndex, VoxelPoints> voxel_map;
        voxel_map.reserve(std::max(16, point_num / 8));

        for (int i = 0; i < point_num; ++i) {
            int idx = i * 3;
            float x = input[idx], y = input[idx+1], z = input[idx+2];
            VoxelIndex vi{static_cast<int>(std::floor(x/voxel_size)),
                        static_cast<int>(std::floor(y/voxel_size)),
                        static_cast<int>(std::floor(z/voxel_size))};
            auto& vp = voxel_map[vi];
            vp.x_list.push_back(x);
            vp.y_list.push_back(y);
            vp.z_list.push_back(z);
        }

        output.resize(voxel_map.size() * 3);
        int cnt = 0;
        for (const auto& pair : voxel_map) {
            //RCLCPP_INFO(this->get_logger(), "%d,%d,%d", pair.first.x, pair.first.y, pair.first.z);
            float x, y, z;
            pair.second.getMean(x, y, z);
            output[cnt++] = x;
            output[cnt++] = y;
            output[cnt++] = z;
        }
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<float> a;
        a.reserve(msg->width * 3);
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            if (std::sqrt((*iter_x)*(*iter_x) + (*iter_y)*(*iter_y)) > 20.0f || *iter_z > 1.0f) continue;
            a.push_back(*iter_x);
            a.push_back(*iter_y);
            a.push_back(*iter_z);
        }
        if (a.empty()) return;
        std::vector<float> input;
        xyzcheck(a, input, 1);
        std::vector<float> input_cloud;
        voxelDownsample(input, input_cloud);
        int cnt = input_cloud.size();
        if (cnt == 0) return;
        int point_num = cnt / 3;
        RCLCPP_INFO(this->get_logger(), "%d", point_num);

        flann::Matrix<float> dataset(input_cloud.data(), point_num, 3);
        flann::KDTreeIndexParams index_params(4); 
        flann::Index<flann::L2<float>> index(dataset, index_params);
        index.buildIndex();

        std::vector<int> label(point_num, -2);

        const int MAX_N = 512; 
        std::vector<int> indices_buf(MAX_N);
        std::vector<float> dists_buf(MAX_N);
        flann::Matrix<int> indices_mat(indices_buf.data(), 1, MAX_N);
        flann::Matrix<float> dists_mat(dists_buf.data(), 1, MAX_N);

        flann::SearchParams search_params(10); 
        int cluster_id = 0;

        for (int i = 0; i < point_num; ++i) {
            if (label[i] != -2) continue;

            flann::Matrix<float> query(input_cloud.data() + i*3, 1, 3);
            int found = index.radiusSearch(query, indices_mat, dists_mat, eps, search_params);
            int neigh_cnt = found;
            if (neigh_cnt <= 0) {
                label[i] = -1;
                continue;
            }
            if (neigh_cnt < min_pts) {
                label[i] = -1;
                continue;
            }

            label[i] = cluster_id;
            std::vector<int> neighbors;
            neighbors.reserve(neigh_cnt * 2);
            for (int k = 0; k < neigh_cnt; ++k) neighbors.push_back(indices_buf[k]);

            size_t q_idx = 0;
            while (q_idx < neighbors.size()) {
                int p = neighbors[q_idx++];
                if (p < 0 || p >= point_num) continue;
                if (label[p] == -1) label[p] = cluster_id;
                if (label[p] != -2) continue;
                label[p] = cluster_id;

                flann::Matrix<float> q_p(input_cloud.data() + p*3, 1, 3);
                int found_p = index.radiusSearch(q_p, indices_mat, dists_mat, eps, search_params);
                if (found_p > 0 && found_p >= min_pts) {
                    for (int kk = 0; kk < found_p; ++kk) neighbors.push_back(indices_buf[kk]);
                }
            }
            ++cluster_id;
        }
        auto end = std::chrono::high_resolution_clock::now();

        if (cluster_id == 0) {
            return;
        }
        std::vector<float> maxz(cluster_id, -1e9f), minz(cluster_id, 1e9f);
        std::vector<float> maxy(cluster_id, -1e9f), miny(cluster_id, 1e9f);
        std::vector<float> maxx(cluster_id, -1e9f), minx(cluster_id, 1e9f);
        for (int i = 0; i < point_num; ++i) {
            if (label[i] < 0) continue;
            int cid = label[i];
            float px = input_cloud[i*3];
            float py = input_cloud[i*3+1];
            float pz = input_cloud[i*3+2];
            if (pz > maxz[cid]) maxz[cid] = pz;
            if (pz < minz[cid]) minz[cid] = pz;
            if (py > maxy[cid]) maxy[cid] = py;
            if (py < miny[cid]) miny[cid] = py;
            if (px > maxx[cid]) maxx[cid] = px;
            if (px < minx[cid]) minx[cid] = px;
        }
        std::vector<char> is_valid(cluster_id, 0);
        for (int i = 0; i < cluster_id; ++i) {
            is_valid[i] = ((maxz[i] - minz[i]) <= 0.4f) && ((maxy[i] - miny[i]) <= 0.5f) && ((maxx[i] - minx[i]) <= 0.5f);
        }

        int valid_cnt = 0;
        for (int i = 0; i < point_num; ++i) if (label[i] >= 0 && is_valid[label[i]]) ++valid_cnt;
        if (valid_cnt == 0) return;

        auto cloud_msg = sensor_msgs::msg::PointCloud2();
        cloud_msg.header = msg->header;
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3,
            "x",1,sensor_msgs::msg::PointField::FLOAT32,
            "y",1,sensor_msgs::msg::PointField::FLOAT32,
            "z",1,sensor_msgs::msg::PointField::FLOAT32
        );
        modifier.resize(valid_cnt);
        //modifier.resize(point_num);

        sensor_msgs::PointCloud2Iterator<float> iter_xx(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_yy(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_zz(cloud_msg, "z");
        for (int i = 0; i < point_num; ++i) {
            if (label[i] >= 0 && is_valid[label[i]]) {
                *iter_xx = input_cloud[i*3];
                *iter_yy = input_cloud[i*3+1];
                *iter_zz = input_cloud[i*3+2];
                //RCLCPP_INFO(this->get_logger(), "(%f,%f,%f)", *iter_xx, *iter_yy, *iter_zz);
                ++iter_xx; ++iter_yy; ++iter_zz;
            }
        }

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end-start);
        RCLCPP_INFO(this->get_logger(), "用时:%ldms", duration.count());
        publisher_->publish(cloud_msg);
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(DBSCAN_component);