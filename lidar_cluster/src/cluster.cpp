#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Bool.h>

struct Point {
    float x;
    float y;
};

struct Cluster {
    std::vector<Point> points;
};

// Function to convert LiDAR data from polar to Cartesian coordinates
std::vector<Point> convertToCartesian(const sensor_msgs::LaserScan::ConstPtr& msg) {
    std::vector<Point> cartesian_points;
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        float angle = msg->angle_min + i * msg->angle_increment;
        float range = msg->ranges[i];
        
        // Ignore invalid ranges (can happen with LiDAR)
        if (range < msg->range_min || range > msg->range_max) {
            continue;
        }

        // Convert to Cartesian coordinates
        Point p;
        p.x = range * cos(angle);
        p.y = range * sin(angle);
        cartesian_points.push_back(p);
    }
    return cartesian_points;
}

// Function to filter points within the ROI
std::vector<Point> filterROI(const std::vector<Point>& points, float min_x, float max_x, float min_y, float max_y) {
    std::vector<Point> filtered_points;
    for (const auto& p : points) {
        if (p.x >= min_x && p.x <= max_x && p.y >= min_y && p.y <= max_y) {
            filtered_points.push_back(p);
        }
    }
    return filtered_points;
}

// Function to perform clustering on the filtered points
std::vector<Cluster> performClustering(const std::vector<Point>& points, float cluster_threshold) {
    std::vector<Cluster> clusters;
    std::vector<bool> visited(points.size(), false);
    std::vector<int> labels(points.size(), -1);  // -1: 노이즈, 0 이상의 값: 클러스터 ID
    int cluster_id = 0;
    int minPts = 20;  // 한 클러스터를 형성하기 위한 최소한의 점 개수

    // 모든 포인트에 대해 클러스터링 시도
    for (size_t i = 0; i < points.size(); ++i) {
        if (visited[i]) {
            continue;  // 이미 방문한 점은 건너뜀
        }

        visited[i] = true;
        std::vector<size_t> neighbors;

        // 현재 포인트 주변의 이웃 찾기
        for (size_t j = 0; j < points.size(); ++j) {
            if (std::hypot(points[i].x - points[j].x, points[i].y - points[j].y) <= cluster_threshold) {
                neighbors.push_back(j);
            }
        }

        // 이웃 점의 개수가 minPts보다 적으면 노이즈로 처리
        if (neighbors.size() < minPts) {
            labels[i] = -1;  // 노이즈로 설정
        } else {
            // 새로운 클러스터 형성
            cluster_id++;
            Cluster current_cluster;

            // 클러스터 확장
            size_t k = 0;
            while (k < neighbors.size()) {
                size_t pt_idx = neighbors[k];

                // 아직 방문하지 않은 점이라면 방문 처리
                if (!visited[pt_idx]) {
                    visited[pt_idx] = true;

                    // 해당 점 주변의 이웃 추가 검색
                    std::vector<size_t> new_neighbors;
                    for (size_t m = 0; m < points.size(); ++m) {
                        if (std::hypot(points[pt_idx].x - points[m].x, points[pt_idx].y - points[m].y) <= cluster_threshold) {
                            new_neighbors.push_back(m);
                        }
                    }

                    // 이웃 점의 개수가 minPts 이상이면 이웃 리스트에 추가
                    if (new_neighbors.size() >= minPts) {
                        neighbors.insert(neighbors.end(), new_neighbors.begin(), new_neighbors.end());
                    }
                }

                // 아직 클러스터에 속하지 않은 점이라면 클러스터에 추가
                if (labels[pt_idx] == -1) {
                    labels[pt_idx] = cluster_id;
                    current_cluster.points.push_back(points[pt_idx]);
                }

                k++;
            }

            clusters.push_back(current_cluster);  // 완성된 클러스터 추가
        }
    }

    return clusters;
}

ros::Publisher point_cloud_pub;
ros::Publisher cluster_detected_pub;

// LiDAR callback to process data
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Step 1: Convert polar coordinates to Cartesian coordinates
    std::vector<Point> cartesian_points = convertToCartesian(msg);
    
    // Step 2: Filter points within the ROI
    float min_x = 0.2; // Define your ROI x-axis limits
    float max_x = 2.5;
    float min_y = -1.0; // Define your ROI y-axis limits
    float max_y = 1.0;
    
    std::vector<Point> filtered_points = filterROI(cartesian_points, min_x, max_x, min_y, max_y);
    
    // Step 3: Publish filtered points as a PointCloud
    sensor_msgs::PointCloud cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = "laser"; // Change frame ID to match your setup

    for (const auto& p : filtered_points) {
        geometry_msgs::Point32 point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0.0; // Since it's 2D LiDAR data, z is 0
        cloud_msg.points.push_back(point);
    }
    
    point_cloud_pub.publish(cloud_msg);

    // Step 4: Perform clustering and check for detected clusters
    float cluster_threshold = 0.05;
    std::vector<Cluster> clusters = performClustering(filtered_points, cluster_threshold);

    std_msgs::Bool cluster_detected_msg;
    cluster_detected_msg.data = !clusters.empty();  // true if clusters are detected
    cluster_detected_pub.publish(cluster_detected_msg);

    if (!clusters.empty()) {
        ROS_INFO("Cluster detected within ROI");
    } else {
        ROS_INFO("No cluster detected within ROI");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_roi_clustering_node");
    ros::NodeHandle nh;
    ROS_INFO("detection_node is ON");

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("filtered_point_cloud", 1000);
    cluster_detected_pub = nh.advertise<std_msgs::Bool>("lidar_detected", 1000);

    ros::Subscriber lidar_sub = nh.subscribe("/scan", 1000, lidarCallback);
    ros::spin();
    return 0;
}
