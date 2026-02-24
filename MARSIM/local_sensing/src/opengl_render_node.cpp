#include "opengl_sim.hpp"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>
#include <string>
#include <tr1/unordered_map>

using namespace Eigen;
using namespace std;

using PointType = pcl::PointXYZI;

#define MAX_INTENSITY 1
#define MIN_INTENSITY 0.1

class OpenGLRenderNode : public rclcpp::Node
{
public:
    OpenGLRenderNode(const std::string& map_file_path) : Node("opengl_render_node"), map_file_path_(map_file_path)
    {
        // Declare parameters
        this->declare_parameter("quadrotor_name", "quadrotor");
        this->declare_parameter("is_360lidar", 1);
        this->declare_parameter("sensing_horizon", 30.0);
        this->declare_parameter("sensing_rate", 10.0);
        this->declare_parameter("estimation_rate", 10.0);
        this->declare_parameter("polar_resolution", 0.2);
        this->declare_parameter("yaw_fov", 70.4);
        this->declare_parameter("vertical_fov", 77.2);
        this->declare_parameter("min_raylength", 1.0);
        this->declare_parameter("downsample_res", 0.1);
        this->declare_parameter("livox_linestep", 1);
        this->declare_parameter("use_avia_pattern", 1);
        this->declare_parameter("curvature_limit", 0.1);
        this->declare_parameter("hash_cubesize", 0.1);
        this->declare_parameter("use_vlp32_pattern", 0);
        this->declare_parameter("use_minicf_pattern", 0);
        this->declare_parameter("use_os128_pattern", 0);
        this->declare_parameter("use_gaussian_filter", 0);
        this->declare_parameter("dynobj_enable", 0);
        this->declare_parameter("dynobject_size", 0.5);
        this->declare_parameter("dynobject_num", 5);
        this->declare_parameter("dyn_mode", 1);
        this->declare_parameter("dyn_velocity", 1.0);
        this->declare_parameter("use_uav_extra_model", 0);
        this->declare_parameter("collisioncheck_enable", 0);
        this->declare_parameter("collision_range", 0.5);
        this->declare_parameter("output_pcd", 0);
        this->declare_parameter("uav_num", 1);
        this->declare_parameter("drone_id", 0);

        // Get parameters
        quad_name_ = this->get_parameter("quadrotor_name").as_string();
        is_360lidar_ = this->get_parameter("is_360lidar").as_int();
        sensing_horizon_ = this->get_parameter("sensing_horizon").as_double();
        sensing_rate_ = this->get_parameter("sensing_rate").as_double();
        estimation_rate_ = this->get_parameter("estimation_rate").as_double();
        polar_resolution_ = this->get_parameter("polar_resolution").as_double();
        yaw_fov_ = this->get_parameter("yaw_fov").as_double();
        vertical_fov_ = this->get_parameter("vertical_fov").as_double();
        min_raylength_ = this->get_parameter("min_raylength").as_double();
        downsample_res_ = this->get_parameter("downsample_res").as_double();
        livox_linestep_ = this->get_parameter("livox_linestep").as_int();
        use_avia_pattern_ = this->get_parameter("use_avia_pattern").as_int();
        curvature_limit_ = this->get_parameter("curvature_limit").as_double();
        hash_cubesize_ = this->get_parameter("hash_cubesize").as_double();
        use_vlp32_pattern_ = this->get_parameter("use_vlp32_pattern").as_int();
        use_minicf_pattern_ = this->get_parameter("use_minicf_pattern").as_int();
        use_os128_pattern_ = this->get_parameter("use_os128_pattern").as_int();
        use_gaussian_filter_ = this->get_parameter("use_gaussian_filter").as_int();
        dynobj_enable_ = this->get_parameter("dynobj_enable").as_int();
        dynobject_size_ = this->get_parameter("dynobject_size").as_double();
        dynobject_num_ = this->get_parameter("dynobject_num").as_int();
        dyn_mode_ = this->get_parameter("dyn_mode").as_int();
        dyn_velocity_ = this->get_parameter("dyn_velocity").as_double();
        use_uav_extra_model_ = this->get_parameter("use_uav_extra_model").as_int();
        collisioncheck_enable_ = this->get_parameter("collisioncheck_enable").as_int();
        collision_range_ = this->get_parameter("collision_range").as_double();
        output_pcd_ = this->get_parameter("output_pcd").as_int();
        drone_num_ = this->get_parameter("uav_num").as_int();
        drone_id_ = this->get_parameter("drone_id").as_int();

        // Initialize variables
        has_odom_ = false;
        has_global_map_ = false;
        render_ready_ = false;
        comp_time_count_ = 0;
        
        // Publishers
        pub_cloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
        pub_intercloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensor_cloud", 10);
        pub_dyncloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dyn_cloud", 10);
        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("sensor_pose", 10);
        pub_uavcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("uav_cloud", 10);
        depth_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("depth_img", 10);
        comp_time_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("simulator_compute_time", 10);
        pub_collisioncloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("collision_cloud", 10);

        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry", 50, std::bind(&OpenGLRenderNode::rcvOdometryCallbck, this, std::placeholders::_1));
        
        global_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "global_map", 1, std::bind(&OpenGLRenderNode::rcvGlobalPointCloudCallBack, this, std::placeholders::_1));

        // Timers
        double sensing_duration = 1.0 / sensing_rate_;
        
        local_sensing_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(sensing_duration),
            std::bind(&OpenGLRenderNode::renderSensedPoints, this));
            
        dynobj_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(sensing_duration),
            std::bind(&OpenGLRenderNode::dynobjGenerate, this));

        // Initialize OpenGL renderer
        initializeRenderer();
        
        t_init_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "OpenGL render node initialized for drone %d", drone_id_);
    }

private:
    void initializeRenderer()
    {
        // Calculate image dimensions based on LiDAR parameters
        int image_width, image_height;
        if (is_360lidar_) {
            image_width = ceil(360.0 / polar_resolution_);
        } else {
            image_width = ceil(yaw_fov_ / polar_resolution_);
        }
        image_height = ceil(vertical_fov_ / polar_resolution_);
        image_width_ = image_width;
        image_height_ = image_height;
        
        // Initialize OpenGL renderer with parameters
        render_.setParameters(image_width, image_height, 250, 250, downsample_res_, 
                             polar_resolution_, yaw_fov_, vertical_fov_, 0.1, 
                             sensing_horizon_, sensing_rate_, use_avia_pattern_, 
                             use_os128_pattern_, use_minicf_pattern_);
        
        // Load the map file
        RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_file_path_.c_str());
        render_.read_pointcloud_fromfile(map_file_path_);
        render_ready_ = render_.is_renderer_ready();

        // Fallback: if OpenGL context fails, run a CPU radius-crop LiDAR model
        // using the same MARSIM map so downstream mapping still works headlessly.
        if (!render_ready_) {
            RCLCPP_WARN(
                this->get_logger(),
                "OpenGL renderer unavailable, switching to CPU fallback local sensing");
            pcl::PointCloud<PointType> loaded_map;
            int status = pcl::io::loadPCDFile<PointType>(map_file_path_, loaded_map);
            if (status != 0 || loaded_map.points.empty()) {
                pcl::PointCloud<pcl::PointXYZ> xyz_map;
                if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_path_, xyz_map) == 0 && !xyz_map.points.empty()) {
                    loaded_map.points.reserve(xyz_map.points.size());
                    for (const auto& pt : xyz_map.points) {
                        PointType q;
                        q.x = pt.x;
                        q.y = pt.y;
                        q.z = pt.z;
                        q.intensity = 1.0f;
                        loaded_map.points.push_back(q);
                    }
                }
            }
            if (loaded_map.points.empty()) {
                has_global_map_ = false;
                RCLCPP_ERROR(this->get_logger(), "Failed to load fallback map cloud from %s", map_file_path_.c_str());
                return;
            }
            global_map_ = loaded_map;
            global_kdtree_.setInputCloud(global_map_.makeShared());
            has_global_map_ = true;
            RCLCPP_INFO(this->get_logger(), "CPU fallback map loaded with %zu points", global_map_.points.size());
        } else {
            has_global_map_ = true;
            RCLCPP_INFO(this->get_logger(), "Map loaded from file, enabling LiDAR simulation");
        }
        
        // Initialize dynamic objects if enabled
        if (dynobj_enable_) {
            initializeDynamicObjects();
        }
    }

    void initializeDynamicObjects()
    {
        // Initialize dynamic objects
        dyn_obs_pos_vec_.resize(dynobject_num_);
        dyn_obs_vel_vec_.resize(dynobject_num_);
        dyn_obs_dir_.resize(dynobject_num_);
        dyn_start_time_vec_.resize(dynobject_num_);
        
        for (int i = 0; i < dynobject_num_; i++) {
            // Random position within sensing range
            double x = (rand() / double(RAND_MAX) - 0.5) * sensing_horizon_;
            double y = (rand() / double(RAND_MAX) - 0.5) * sensing_horizon_;
            double z = rand() / double(RAND_MAX) * 3.0;
            
            dyn_obs_pos_vec_[i] = Eigen::Vector3d(x, y, z);
            
            // Random velocity
            double vx = (rand() / double(RAND_MAX) - 0.5) * dyn_velocity_;
            double vy = (rand() / double(RAND_MAX) - 0.5) * dyn_velocity_;
            double vz = (rand() / double(RAND_MAX) - 0.5) * dyn_velocity_ * 0.1;
            
            dyn_obs_vel_vec_[i] = Eigen::Vector3d(vx, vy, vz);
            dyn_start_time_vec_[i] = this->now();
        }
    }

    void rcvOdometryCallbck(const nav_msgs::msg::Odometry::SharedPtr odom)
    {
        has_odom_ = true;
        odom_ = *odom;
    }

    void rcvGlobalPointCloudCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_map)
    {
        pcl::PointCloud<PointType> local_map;
        pcl::fromROSMsg(*pointcloud_map, local_map);
        
        // Store the map for rendering - the renderer will use it internally
        global_map_ = local_map;
        global_kdtree_.setInputCloud(global_map_.makeShared());
        has_global_map_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Global map received with %zu points", local_map.points.size());
    }

    void renderSensedPoints()
    {
        if (!has_odom_ || !has_global_map_) {
            return;
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        // Get current pose
        Eigen::Vector3f pos(odom_.pose.pose.position.x, 
                           odom_.pose.pose.position.y, 
                           odom_.pose.pose.position.z);
        
        Eigen::Quaternionf q(odom_.pose.pose.orientation.w,
                            odom_.pose.pose.orientation.x,
                            odom_.pose.pose.orientation.y,
                            odom_.pose.pose.orientation.z);

        // Render point cloud using OpenGL, or fallback to CPU local crop.
        pcl::PointCloud<PointType>::Ptr local_map(new pcl::PointCloud<PointType>);
        if (render_ready_) {
            double t_pattern_start = (this->now() - t_init_).seconds();
            render_.render_pointcloud(local_map, pos, q, t_pattern_start);
        } else {
            std::vector<int> k_indices;
            std::vector<float> k_sqr_dists;
            PointType search_pt;
            search_pt.x = pos.x();
            search_pt.y = pos.y();
            search_pt.z = pos.z();
            if (global_kdtree_.radiusSearch(search_pt, sensing_horizon_, k_indices, k_sqr_dists) > 0) {
                const float min_ray2 = static_cast<float>(min_raylength_ * min_raylength_);
                local_map->points.reserve(k_indices.size());
                for (size_t i = 0; i < k_indices.size(); ++i) {
                    if (k_sqr_dists[i] < min_ray2) {
                        continue;
                    }
                    local_map->points.push_back(global_map_.points[k_indices[i]]);
                }
            }
        }

        if (local_map->points.empty()) {
            return;
        }

        // Publish world frame point cloud
        sensor_msgs::msg::PointCloud2 local_map_pcd;
        pcl::toROSMsg(*local_map, local_map_pcd);
        local_map_pcd.header = odom_.header;
        local_map_pcd.header.frame_id = "world";
        pub_cloud_->publish(local_map_pcd);

        // Transform to sensor frame and publish
        Eigen::Matrix3f rot = q.toRotationMatrix();
        Eigen::Matrix4f sensor2world;
        sensor2world << rot(0,0), rot(0,1), rot(0,2), pos.x(),
                       rot(1,0), rot(1,1), rot(1,2), pos.y(),
                       rot(2,0), rot(2,1), rot(2,2), pos.z(),
                       0, 0, 0, 1;
        
        Eigen::Matrix4f world2sensor = sensor2world.inverse();
        
        pcl::PointCloud<PointType> point_in_sensor;
        pcl::transformPointCloud(*local_map, point_in_sensor, world2sensor);
        
        sensor_msgs::msg::PointCloud2 sensor_map_pcd;
        pcl::toROSMsg(point_in_sensor, sensor_map_pcd);
        sensor_map_pcd.header.frame_id = "/sensor";
        sensor_map_pcd.header.stamp = odom_.header.stamp;
        pub_intercloud_->publish(sensor_map_pcd);

        publishDepthImage(point_in_sensor, sensor_map_pcd.header.stamp);

        // Publish sensor pose
        geometry_msgs::msg::PoseStamped sensor_pose;
        sensor_pose.header = odom_.header;
        sensor_pose.header.frame_id = "/map";
        sensor_pose.pose = odom_.pose.pose;
        pub_pose_->publish(sensor_pose);

        // Calculate and publish computation time
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        double comp_time = duration.count() / 1000.0; // Convert to milliseconds
        
        comp_time_vec_.push_back(comp_time);
        comp_time_count_++;
        
        if (comp_time_count_ > 20) {
            comp_time_vec_.pop_front();
            geometry_msgs::msg::PoseStamped totaltime_pub;
            totaltime_pub.pose.position.x = std::accumulate(comp_time_vec_.begin(), comp_time_vec_.end(), 0.0) / comp_time_vec_.size();
            comp_time_pub_->publish(totaltime_pub);
        }
    }

    void publishDepthImage(const pcl::PointCloud<PointType>& sensor_cloud, const builtin_interfaces::msg::Time& stamp)
    {
        if (image_width_ <= 0 || image_height_ <= 0) {
            return;
        }
        cv::Mat depth = cv::Mat::ones(image_height_, image_width_, CV_32FC1) * static_cast<float>(sensing_horizon_);
        const float half_pitch = static_cast<float>(vertical_fov_ * 0.5);
        const float half_yaw = is_360lidar_ ? 180.0f : static_cast<float>(yaw_fov_ * 0.5);
        const float deg_step = static_cast<float>(polar_resolution_);

        for (const auto& pt : sensor_cloud.points) {
            const float x = pt.x;
            const float y = pt.y;
            const float z = pt.z;
            const float xy = std::sqrt(x * x + y * y);
            const float d = std::sqrt(x * x + y * y + z * z);
            if (d < static_cast<float>(min_raylength_) || d > static_cast<float>(sensing_horizon_)) {
                continue;
            }
            const float yaw = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
            const float pitch = std::atan2(z, std::max(1e-6f, xy)) * 180.0f / static_cast<float>(M_PI);
            if (std::abs(pitch) > half_pitch) {
                continue;
            }
            int u = 0;
            if (is_360lidar_) {
                u = static_cast<int>((yaw + 180.0f) / deg_step);
            } else {
                if (std::abs(yaw) > half_yaw) {
                    continue;
                }
                u = static_cast<int>((yaw + half_yaw) / deg_step);
            }
            const int v = static_cast<int>((half_pitch - pitch) / deg_step);
            if (u < 0 || u >= image_width_ || v < 0 || v >= image_height_) {
                continue;
            }
            if (d < depth.at<float>(v, u)) {
                depth.at<float>(v, u) = d;
            }
        }

        cv::Mat rgb(image_height_, image_width_, CV_8UC3, cv::Scalar(0, 0, 0));
        for (int r = 0; r < image_height_; ++r) {
            for (int c = 0; c < image_width_; ++c) {
                const float d = depth.at<float>(r, c);
                if (d >= static_cast<float>(sensing_horizon_)) {
                    continue;
                }
                const float norm = std::max(0.0f, std::min(1.0f, 1.0f - d / static_cast<float>(sensing_horizon_)));
                const uint8_t gray = static_cast<uint8_t>(norm * 255.0f);
                rgb.at<cv::Vec3b>(r, c) = cv::Vec3b(gray, gray, gray);
            }
        }

        cv_bridge::CvImage out;
        out.header.frame_id = "/sensor";
        out.header.stamp = stamp;
        out.encoding = "rgb8";
        out.image = rgb;
        depth_img_pub_->publish(*out.toImageMsg());
    }

    void dynobjGenerate()
    {
        if (!dynobj_enable_ || !has_odom_) {
            return;
        }

        // Update dynamic object positions
        auto current_time = this->now();
        
        for (int i = 0; i < dynobject_num_; i++) {
            double dt = (current_time - dyn_start_time_vec_[i]).seconds();
            dyn_obs_pos_vec_[i] += dyn_obs_vel_vec_[i] * dt;
            dyn_start_time_vec_[i] = current_time;
            
            // Reset if out of bounds
            if (dyn_obs_pos_vec_[i].norm() > sensing_horizon_) {
                double x = (rand() / double(RAND_MAX) - 0.5) * sensing_horizon_ * 0.5;
                double y = (rand() / double(RAND_MAX) - 0.5) * sensing_horizon_ * 0.5;
                double z = rand() / double(RAND_MAX) * 3.0;
                dyn_obs_pos_vec_[i] = Eigen::Vector3d(x, y, z);
            }
        }

        // Generate dynamic object point cloud
        pcl::PointCloud<PointType> dyn_cloud;
        for (int i = 0; i < dynobject_num_; i++) {
            PointType pt;
            pt.x = dyn_obs_pos_vec_[i].x();
            pt.y = dyn_obs_pos_vec_[i].y();
            pt.z = dyn_obs_pos_vec_[i].z();
            pt.intensity = 1.0;
            dyn_cloud.points.push_back(pt);
        }
        
        if (!dyn_cloud.points.empty()) {
            sensor_msgs::msg::PointCloud2 dyn_cloud_msg;
            pcl::toROSMsg(dyn_cloud, dyn_cloud_msg);
            dyn_cloud_msg.header = odom_.header;
            dyn_cloud_msg.header.frame_id = "world";
            pub_dyncloud_->publish(dyn_cloud_msg);
        }
    }

    // Member variables
    std::string map_file_path_;
    std::string quad_name_;
    int drone_num_, drone_id_;
    int is_360lidar_, use_avia_pattern_, use_vlp32_pattern_, use_minicf_pattern_;
    int use_os128_pattern_, use_gaussian_filter_, livox_linestep_;
    int dynobj_enable_, dynobject_num_, dyn_mode_, use_uav_extra_model_;
    int collisioncheck_enable_, output_pcd_;
    double sensing_horizon_, sensing_rate_, estimation_rate_, polar_resolution_;
    double yaw_fov_, vertical_fov_, min_raylength_, downsample_res_;
    double curvature_limit_, hash_cubesize_, dynobject_size_, dyn_velocity_;
    double collision_range_;
    
    bool has_odom_;
    bool has_global_map_;
    bool render_ready_;
    nav_msgs::msg::Odometry odom_;
    pcl::PointCloud<PointType> global_map_;
    pcl::KdTreeFLANN<PointType> global_kdtree_;
    opengl_pointcloud_render render_;
    int image_width_;
    int image_height_;
    
    std::deque<double> comp_time_vec_;
    int comp_time_count_;
    rclcpp::Time t_init_;
    
    // Dynamic objects
    std::vector<Eigen::Vector3d> dyn_obs_pos_vec_;
    std::vector<Eigen::Vector3d> dyn_obs_vel_vec_;
    std::vector<Eigen::Vector3d> dyn_obs_dir_;
    std::vector<rclcpp::Time> dyn_start_time_vec_;

    // ROS 2 publishers and subscribers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_intercloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dyncloud_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_uavcloud_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr comp_time_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_collisioncloud_;
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;
    
    rclcpp::TimerBase::SharedPtr local_sensing_timer_;
    rclcpp::TimerBase::SharedPtr dynobj_timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Check if map file argument is provided
    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("opengl_render_node"), 
                     "Usage: opengl_render_node <map_file_path>");
        return 1;
    }
    
    std::string map_file_path = argv[1];
    RCLCPP_INFO(rclcpp::get_logger("opengl_render_node"), 
                "Loading map file: %s", map_file_path.c_str());
    
    auto node = std::make_shared<OpenGLRenderNode>(map_file_path);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
