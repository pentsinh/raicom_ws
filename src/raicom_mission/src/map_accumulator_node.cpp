#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <iostream>
#include <sstream>
#include <mutex>
#include <thread>
#include <cmath>
#include <set>
#include <iomanip> // For std::setprecision

// 地图ROI参数结构 (保留区域)
struct MapROIParams {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    bool enable_roi_filter;
};

// DisROI参数结构 (移除区域)
struct DisROIParams {
    double min_x, max_x;
    double min_y, max_y;
    double min_z, max_z;
    bool enable_disroi_filter;
};

class MapAccumulatorOnlyPublish {
public:
    MapAccumulatorOnlyPublish(ros::NodeHandle& nh) 
        : nh_(nh), private_nh_("~"), tf_listener_(tf_buffer_), accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
          downsampled_cloud_(new pcl::PointCloud<pcl::PointXYZ>), clear_requested_(false) {
        
        // --- 读取基础参数 ---
        private_nh_.param("input_topic", input_topic_, std::string("/cloud_registered"));
        private_nh_.param("map_frame", map_frame_, std::string("camera_init"));
        private_nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.05);
        private_nh_.param("downsampled_voxel_leaf_size", downsampled_voxel_leaf_size_, 0.2);
        private_nh_.param("max_points", max_points_, 5000000);
        private_nh_.param("enable_realtime_preview", enable_realtime_preview_, true);
        private_nh_.param("preview_rate", preview_rate_, 20.0);
        
        // --- Map ROI过滤参数 ---
        private_nh_.param("enable_map_roi_filter", map_roi_params_.enable_roi_filter, false);
        private_nh_.param("map_roi_min_x", map_roi_params_.min_x, -50.0);
        private_nh_.param("map_roi_max_x", map_roi_params_.max_x, 50.0);
        private_nh_.param("map_roi_min_y", map_roi_params_.min_y, -50.0);
        private_nh_.param("map_roi_max_y", map_roi_params_.max_y, 50.0);
        private_nh_.param("map_roi_min_z", map_roi_params_.min_z, -10.0);
        private_nh_.param("map_roi_max_z", map_roi_params_.max_z, 1.5); // 假设地面以上1.5m是柱体可能出现的区域

        // --- DisROI过滤参数 ---
        private_nh_.param("enable_disroi_filter", disroi_params_.enable_disroi_filter, false);
        private_nh_.param("disroi_min_x", disroi_params_.min_x, -2.0);
        private_nh_.param("disroi_max_x", disroi_params_.max_x, 2.0);
        private_nh_.param("disroi_min_y", disroi_params_.min_y, -2.0);
        private_nh_.param("disroi_max_y", disroi_params_.max_y, 2.0);
        private_nh_.param("disroi_min_z", disroi_params_.min_z, -1.0);
        private_nh_.param("disroi_max_z", disroi_params_.max_z, 3.0);

        // --- 新增：高Z柱体清除参数 ---
        private_nh_.param("enable_initial_high_z_filter", enable_initial_high_z_filter_, true);
        private_nh_.param("grid_size", grid_size_, 0.03);
        private_nh_.param("custom_max_height", custom_max_height_, 5.0); // 超过5米的点不参与此逻辑
        private_nh_.param("high_z_filter_duration", high_z_filter_duration_, 30.0); // 持续30秒标记网格

        // --- 订阅与发布 ---
        cloud_sub_ = nh_.subscribe(input_topic_, 10, &MapAccumulatorOnlyPublish::cloudCallback, this);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_accumulator/accumulated_map", 1);
        downsampled_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/map_accumulator/downsampled_map", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/map_accumulator/map_info", 1);
        roi_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/map_accumulator/roi_markers", 1);
        info_pub_ = nh_.advertise<std_msgs::String>("/map_accumulator/info", 1);

        // --- 服务 ---
        clear_service_ = nh_.advertiseService("/map_accumulator/clear_map", &MapAccumulatorOnlyPublish::clearMapService, this);
        get_info_service_ = nh_.advertiseService("/map_accumulator/get_info", &MapAccumulatorOnlyPublish::getInfoService, this);

        // --- 初始化时间 ---
        last_preview_time_ = ros::Time::now();
        start_time_ = ros::Time::now(); // 记录节点启动时间

        // --- 统计计数器 ---
        total_points_received_ = 0;
        disroi_filtered_points_ = 0;
        roi_filtered_points_ = 0;
        accumulated_points_ = 0;
        frame_count_ = 0;

        ROS_INFO("MapAccumulatorOnlyPublish initialized (High-Z Filter: %s, Duration: %.1fs)", 
                 enable_initial_high_z_filter_ ? "Enabled" : "Disabled", high_z_filter_duration_);
        
        // --- 启动后台线程 ---
        background_thread_ = std::thread(&MapAccumulatorOnlyPublish::backgroundThread, this);
    }

    ~MapAccumulatorOnlyPublish() {
        if (background_thread_.joinable()) {
            background_thread_.join();
        }
        ROS_INFO("MapAccumulatorOnlyPublish shut down.");
    }

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher map_pub_;
    ros::Publisher downsampled_map_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher roi_marker_pub_;
    ros::Publisher info_pub_;
    ros::ServiceServer clear_service_;
    ros::ServiceServer get_info_service_;

    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // 参数
    std::string input_topic_;
    std::string map_frame_;
    double voxel_leaf_size_;
    double downsampled_voxel_leaf_size_;
    int max_points_;
    bool enable_realtime_preview_;
    double preview_rate_;

    // ROI参数
    MapROIParams map_roi_params_;
    DisROIParams disroi_params_;

    // 新增：高Z过滤参数
    bool enable_initial_high_z_filter_;
    double grid_size_;
    double custom_max_height_;
    double high_z_filter_duration_;
    ros::Time start_time_; // 地图开始/清除时间

    // 数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_;
    std::mutex cloud_mutex_;
    ros::Time last_preview_time_;
    int frame_count_;

    // 新增：高Z网格标记集
    std::set<std::pair<int, int>> filtered_xy_grids_; 

    // 统计计数器
    size_t total_points_received_;
    size_t disroi_filtered_points_;
    size_t roi_filtered_points_;
    size_t accumulated_points_;
    
    // 控制标志
    bool clear_requested_;
    std::mutex control_mutex_;

    // 线程
    std::thread background_thread_;

    // 回调函数
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    // 点云处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloudNew(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf_size);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterMapROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filterDisROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 新增：高Z柱体清除逻辑
    void highZFilterAndMark(const pcl::PointCloud<pcl::PointXYZ>::Ptr& incoming_cloud);
    void applyHighZFilterToMap(); 

    // 地图管理
    void addCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void publishMap();
    void publishDownsampledMap();
    void publishMapInfo();
    void publishROIMarkers();

    // 服务回调
    bool clearMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool getInfoService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    // 后台线程
    void backgroundThread();

    // 实时预览
    void realtimePreview();
};

void MapAccumulatorOnlyPublish::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud = transformCloud(cloud_msg);
    if (!transformed_cloud || transformed_cloud->empty()) {
        return;
    }

    total_points_received_ += transformed_cloud->size();
    frame_count_++;
    
    // !!! 新增：在高Z过滤持续时间内，标记高柱体网格 !!!
    if (enable_initial_high_z_filter_) {
        highZFilterAndMark(transformed_cloud); 
    }

    addCloudToMap(transformed_cloud);

    // 实时预览
    if (enable_realtime_preview_ && 
        (ros::Time::now() - last_preview_time_).toSec() > 1.0/preview_rate_) {
        realtimePreview();
        last_preview_time_ = ros::Time::now();
    }
}

// 坐标转换函数（保持不变）
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorOnlyPublish::transformCloud(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    try {
        geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
            map_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp, ros::Duration(0.1));
        sensor_msgs::PointCloud2 transformed_msg;
        tf2::doTransform(*cloud_msg, transformed_msg, transform);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(transformed_msg, *cloud);
        return cloud;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        return nullptr;
    }
}

// VoxelGrid 降采样函数（保持不变）
void MapAccumulatorOnlyPublish::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || voxel_leaf_size_ <= 0) {
        return;
    }
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_grid.filter(*cloud);
}

// 新增：创建新的降采样函数，接受参数（保持不变）
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorOnlyPublish::downsampleCloudNew(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double leaf_size) {
    if (cloud->empty() || leaf_size <= 0) {
        return cloud;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.filter(*downsampled_cloud);
    
    return downsampled_cloud;
}

// Map ROI 过滤函数 (保留ROI内的点)（保持不变）
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorOnlyPublish::filterMapROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || !map_roi_params_.enable_roi_filter) {
        return cloud;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr region_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;

    // X, Y, Z 方向串联过滤 (保留范围内)
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(map_roi_params_.min_x, map_roi_params_.max_x);
    pass.filter(*temp_cloud);

    if (temp_cloud->empty()) { roi_filtered_points_ += cloud->size(); return temp_cloud; }

    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(map_roi_params_.min_y, map_roi_params_.max_y);
    pass.filter(*region_cloud);

    if (region_cloud->empty()) { roi_filtered_points_ += temp_cloud->size(); return region_cloud; }

    pass.setInputCloud(region_cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(map_roi_params_.min_z, map_roi_params_.max_z);
    pass.filter(*region_cloud);

    size_t filtered_count = cloud->size() - region_cloud->size();
    roi_filtered_points_ += filtered_count;
    return region_cloud;
}

// DisROI 过滤函数 (移除DisROI内的点 - 手动遍历实现)（保持不变）
pcl::PointCloud<pcl::PointXYZ>::Ptr MapAccumulatorOnlyPublish::filterDisROI(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || !disroi_params_.enable_disroi_filter) {
        return cloud; 
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    size_t discard_count = 0;

    for (const auto& point : cloud->points) {
        bool inside_disroi = (point.x >= disroi_params_.min_x && point.x <= disroi_params_.max_x) &&
                             (point.y >= disroi_params_.min_y && point.y <= disroi_params_.max_y) &&
                             (point.z >= disroi_params_.min_z && point.z <= disroi_params_.max_z);
        
        if (!inside_disroi) {
            filtered_cloud->points.push_back(point);
        } else {
            discard_count++;
        }
    }

    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = cloud->is_dense;
    filtered_cloud->header = cloud->header;

    disroi_filtered_points_ += discard_count;
    return filtered_cloud;
}

// ====================================================================
// !!! 新增：高Z柱体清除逻辑实现 !!!
// ====================================================================

void MapAccumulatorOnlyPublish::highZFilterAndMark(const pcl::PointCloud<pcl::PointXYZ>::Ptr& incoming_cloud) {
    ros::Duration elapsed = ros::Time::now() - start_time_;
    
    // 超时则停止标记新的网格
    if (elapsed.toSec() > high_z_filter_duration_) {
        ROS_INFO_ONCE("Initial high-Z filtering duration reached (%.1fs). Disabling further grid marking.", high_z_filter_duration_);
        return; 
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);

    // 遍历当前帧，标记符合条件的高Z网格
    for (const auto& point : incoming_cloud->points) {
        // 条件：Z > Map ROI Max Z 且 Z <= Custom Max Height
        if (point.z > map_roi_params_.max_z && point.z <= custom_max_height_) {
            
            // 计算网格索引 (floor向下取整，确保网格划分)
            int grid_x = static_cast<int>(std::floor(point.x / grid_size_));
            int grid_y = static_cast<int>(std::floor(point.y / grid_size_));
            
            // 将网格标记为需要清除的区域
            filtered_xy_grids_.insert({grid_x, grid_y});
        }
    }
}

void MapAccumulatorOnlyPublish::applyHighZFilterToMap() {
    if (filtered_xy_grids_.empty() || accumulated_cloud_->empty()) {
        return;
    }

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filtered_cloud->reserve(accumulated_cloud_->size());
    
    size_t removed_count = 0;

    for (const auto& point : accumulated_cloud_->points) {
        // 计算点所在的网格
        int grid_x = static_cast<int>(std::floor(point.x / grid_size_));
        int grid_y = static_cast<int>(std::floor(point.y / grid_size_));

        // 检查该网格是否在已标记的清除列表中
        if (filtered_xy_grids_.count({grid_x, grid_y})) {
            removed_count++;
        } else {
            filtered_cloud->points.push_back(point);
        }
    }

    // 替换累积地图
    accumulated_cloud_.swap(filtered_cloud);
    accumulated_cloud_->width = accumulated_cloud_->points.size();
    accumulated_cloud_->height = 1;

    // 重新更新统计信息
    accumulated_points_ = accumulated_cloud_->size();
    
    // 更新降采样后的点云（用于发布）
    downsampled_cloud_ = downsampleCloudNew(accumulated_cloud_, downsampled_voxel_leaf_size_);

    // 避免频繁打印，只在DEBUG级别打印
    // if (removed_count > 0) {
    //     ROS_DEBUG("Removed %zu points via high-Z grid filter.", removed_count);
    // }
}

// ====================================================================
// !!! 地图累积逻辑（保持不变） !!!
// ====================================================================

void MapAccumulatorOnlyPublish::addCloudToMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    
    // 1. DisROI过滤（移除）
    pcl::PointCloud<pcl::PointXYZ>::Ptr disroi_filtered_cloud = filterDisROI(cloud);
    if (disroi_filtered_cloud->empty()) return;

    // 2. Map ROI过滤（保留）
    pcl::PointCloud<pcl::PointXYZ>::Ptr roi_filtered_cloud = filterMapROI(disroi_filtered_cloud);
    if (roi_filtered_cloud->empty()) return;

    // 3. 降采样过滤后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>(*roi_filtered_cloud));
    downsampleCloud(downsampled_cloud);

    // 4. 添加到累积地图
    *accumulated_cloud_ += *downsampled_cloud;

    accumulated_points_ += downsampled_cloud->size();

    // 如果点数过多，进行整体降采样
    if (accumulated_cloud_->size() > static_cast<size_t>(max_points_)) {
        ROS_WARN("Map size exceeds limit (%d points), downsampling the entire map...", max_points_);
        downsampleCloud(accumulated_cloud_);
    }
    
    // 更新降采样后的点云（用于发布）
    downsampled_cloud_ = downsampleCloudNew(accumulated_cloud_, downsampled_voxel_leaf_size_);
}

// ====================================================================
// !!! 服务回调和后台线程逻辑 !!!
// ====================================================================

bool MapAccumulatorOnlyPublish::clearMapService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    clear_requested_ = true;
    ROS_INFO("Clear map service requested");
    return true;
}

void MapAccumulatorOnlyPublish::backgroundThread() {
    ros::Rate rate(5); // 降低频率到 5Hz，高Z清除是较耗时的操作
    while (ros::ok()) {
        // 检查是否需要清除
        {
            std::lock_guard<std::mutex> lock(control_mutex_);
            if (clear_requested_) {
                std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
                accumulated_cloud_->clear();
                downsampled_cloud_->clear();
                
                // !!! 新增：重置高Z过滤相关数据 !!!
                filtered_xy_grids_.clear(); 
                start_time_ = ros::Time::now(); // 重置开始时间，重新开始计时
                
                frame_count_ = 0;
                total_points_received_ = 0;
                disroi_filtered_points_ = 0;
                roi_filtered_points_ = 0;
                accumulated_points_ = 0;
                clear_requested_ = false;
                ROS_INFO("Map cleared (statistics reset)");
            }
        }
        
        // !!! 新增：定期执行高Z网格清除操作 !!!
        if (enable_initial_high_z_filter_ && !filtered_xy_grids_.empty()) {
            applyHighZFilterToMap();
        }

        // 定期发布地图信息和ROI可视化
        static int counter = 0;
        if (++counter % 25 == 0) { // 大约每5秒 (5Hz * 25)
            publishMapInfo();
            publishROIMarkers(); 
        }
        rate.sleep();
    }
}

// 其他发布函数 (略有修改以包含高Z信息)
void MapAccumulatorOnlyPublish::publishMapInfo() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    // 发布文本信息
    std_msgs::String info_msg;
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1);

    ss << "Map Info:\n";
    ss << "Points: " << accumulated_cloud_->size() << "\n";
    ss << "Downsampled Points: " << downsampled_cloud_->size() << "\n";
    ss << "Frames: " << frame_count_ << "\n";
    ss << "HighZ Grids Marked: " << filtered_xy_grids_.size() << "\n"; // 新增：显示标记的网格数

    // 统计信息
    if (total_points_received_ > 0) {
        ss << "=== Filtering Statistics ===\n";
        ss << "Total received: " << total_points_received_ << "\n";
        ss << "DisROI filtered: " << disroi_filtered_points_ << " (" << 100.0 * disroi_filtered_points_ / total_points_received_ << "%)\n";
        ss << "ROI filtered: " << roi_filtered_points_ << " (" << 100.0 * roi_filtered_points_ / total_points_received_ << "%)\n";
        ss << "Accumulated: " << accumulated_points_ << " (" << 100.0 * accumulated_points_ / total_points_received_ << "%)\n";
    }
    
    info_msg.data = ss.str();
    info_pub_.publish(info_msg);

    // 发布可视化标记
    if (!accumulated_cloud_->empty()) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = map_frame_;
        marker.ns = "map_info";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 5; 
        marker.pose.orientation.w = 1.0;
        marker.scale.z = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        std::stringstream text_ss;
        text_ss << std::fixed << std::setprecision(0);
        text_ss << "Points: " << accumulated_cloud_->size() 
                << "\nDownsampled: " << downsampled_cloud_->size()
                << "\nFrames: " << frame_count_
                << "\nHigh-Z Grids: " << filtered_xy_grids_.size();
                
        marker.text = text_ss.str();
        marker_pub_.publish(marker);
    }
}

// 实时预览（保持不变）
void MapAccumulatorOnlyPublish::realtimePreview() {
    publishMap();
    publishDownsampledMap(); 
}

// 发布地图（保持不变）
void MapAccumulatorOnlyPublish::publishMap() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (accumulated_cloud_->empty()) {
        return;
    }
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*accumulated_cloud_, map_msg);
    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = map_frame_;
    map_pub_.publish(map_msg);
}

// 发布降采样地图（保持不变）
void MapAccumulatorOnlyPublish::publishDownsampledMap() {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    if (downsampled_cloud_->empty()) {
        return;
    }
    sensor_msgs::PointCloud2 downsampled_msg;
    pcl::toROSMsg(*downsampled_cloud_, downsampled_msg);
    downsampled_msg.header.stamp = ros::Time::now();
    downsampled_msg.header.frame_id = map_frame_;
    downsampled_map_pub_.publish(downsampled_msg);
}

// 发布 ROI Markers（保持不变）
void MapAccumulatorOnlyPublish::publishROIMarkers() {
    visualization_msgs::MarkerArray marker_array;
    
    // Map ROI 可视化（绿色框）
    if (map_roi_params_.enable_roi_filter) {
        visualization_msgs::Marker map_roi_marker;
        map_roi_marker.header.stamp = ros::Time::now();
        map_roi_marker.header.frame_id = map_frame_;
        map_roi_marker.ns = "map_roi";
        map_roi_marker.id = 0;
        map_roi_marker.type = visualization_msgs::Marker::CUBE;
        map_roi_marker.action = visualization_msgs::Marker::ADD;
        
        map_roi_marker.pose.position.x = (map_roi_params_.min_x + map_roi_params_.max_x) / 2.0;
        map_roi_marker.pose.position.y = (map_roi_params_.min_y + map_roi_params_.max_y) / 2.0;
        map_roi_marker.pose.position.z = (map_roi_params_.min_z + map_roi_params_.max_z) / 2.0;
        map_roi_marker.pose.orientation.w = 1.0;
        
        map_roi_marker.scale.x = map_roi_params_.max_x - map_roi_params_.min_x;
        map_roi_marker.scale.y = map_roi_params_.max_y - map_roi_params_.min_y;
        map_roi_marker.scale.z = map_roi_params_.max_z - map_roi_params_.min_z;
        
        map_roi_marker.color.r = 0.0;
        map_roi_marker.color.g = 1.0;
        map_roi_marker.color.b = 0.0;
        map_roi_marker.color.a = 0.2;
        
        marker_array.markers.push_back(map_roi_marker);
    }
    
    // DisROI 可视化（红色框）
    if (disroi_params_.enable_disroi_filter) {
        visualization_msgs::Marker disroi_marker;
        disroi_marker.header.stamp = ros::Time::now();
        disroi_marker.header.frame_id = map_frame_;
        disroi_marker.ns = "disroi";
        disroi_marker.id = 1;
        disroi_marker.type = visualization_msgs::Marker::CUBE;
        disroi_marker.action = visualization_msgs::Marker::ADD;
        
        disroi_marker.pose.position.x = (disroi_params_.min_x + disroi_params_.max_x) / 2.0;
        disroi_marker.pose.position.y = (disroi_params_.min_y + disroi_params_.max_y) / 2.0;
        disroi_marker.pose.position.z = (disroi_params_.min_z + disroi_params_.max_z) / 2.0;
        disroi_marker.pose.orientation.w = 1.0;
        
        disroi_marker.scale.x = disroi_params_.max_x - disroi_params_.min_x;
        disroi_marker.scale.y = disroi_params_.max_y - disroi_params_.min_y;
        disroi_marker.scale.z = disroi_params_.max_z - disroi_params_.min_z;
        
        disroi_marker.color.r = 1.0;
        disroi_marker.color.g = 0.0;
        disroi_marker.color.b = 0.0;
        disroi_marker.color.a = 0.3;
        
        marker_array.markers.push_back(disroi_marker);
    }
    
    if (!marker_array.markers.empty()) {
        roi_marker_pub_.publish(marker_array);
    }
}

// GetInfo 服务（保持不变）
bool MapAccumulatorOnlyPublish::getInfoService(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    publishMapInfo();
    return true;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "map_accumulator_only_node");
    ros::NodeHandle nh;
    MapAccumulatorOnlyPublish accumulator(nh);
    ros::spin();
    return 0;
}
