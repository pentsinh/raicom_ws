#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;

/**
 * @brief 根据四个顶点生成具有指定厚度的片状矩形点云
 *
 * @param vertices 矩形的四个顶点，必须按顺序排列（如逆时针或顺时针），
 *                 其中 vertices[0]-vertices[1] 和 vertices[0]-vertices[3] 为相邻边。
 * @param thickness 矩形物体的厚度 (单位: 米)
 * @param resolution 点云采样的分辨率/点间距 (单位: 米)
 * @return CloudT::Ptr 生成的点云指针
 */
CloudT::Ptr generatePlate(const std::vector<Eigen::Vector3f> &vertices, float thickness, float resolution)
{
    CloudT::Ptr cloud(new CloudT);
    if (vertices.size() != 4)
    {
        ROS_ERROR("顶点数量必须为4！");
        return cloud;
    }

    // 提取顶点，假设 v0, v1, v3 构成相邻边
    Eigen::Vector3f v0 = vertices[0];
    Eigen::Vector3f v1 = vertices[1];
    Eigen::Vector3f v3 = vertices[3];

    // 计算边向量和法向量
    Eigen::Vector3f e1 = v1 - v0;
    Eigen::Vector3f e2 = v3 - v0;
    Eigen::Vector3f n = e1.cross(e2).normalized();

    float len1 = e1.norm();
    float len2 = e2.norm();

    // 计算各个方向的采样步数
    int steps1 = std::max(1, (int)(len1 / resolution));
    int steps2 = std::max(1, (int)(len2 / resolution));
    int steps_t = std::max(1, (int)(thickness / resolution));

    float du = 1.0f / steps1;
    float dv = 1.0f / steps2;
    float dt = 1.0f / steps_t;

    // 1. 生成上下两个表面
    for (int i = 0; i <= steps1; ++i)
    {
        for (int j = 0; j <= steps2; ++j)
        {
            float u = i * du;
            float v = j * dv;
            Eigen::Vector3f p = v0 + u * e1 + v * e2;

            // 沿法向量正负方向偏移半个厚度
            Eigen::Vector3f p_top = p + 0.5f * thickness * n;
            Eigen::Vector3f p_bot = p - 0.5f * thickness * n;

            cloud->push_back(PointT(p_top.x(), p_top.y(), p_top.z()));
            cloud->push_back(PointT(p_bot.x(), p_bot.y(), p_bot.z()));
        }
    }

    // 2. 生成四个侧面以封闭物体
    // 侧面1和2 (沿 e2 方向)
    for (int j = 0; j <= steps2; ++j)
    {
        float v = j * dv;
        Eigen::Vector3f base1 = v0 + v * e2;
        Eigen::Vector3f base2 = v1 + v * e2;
        for (int k = 0; k <= steps_t; ++k)
        {
            float t = (k * dt - 0.5f) * thickness;
            cloud->push_back(PointT((base1 + t * n).x(), (base1 + t * n).y(), (base1 + t * n).z()));
            cloud->push_back(PointT((base2 + t * n).x(), (base2 + t * n).y(), (base2 + t * n).z()));
        }
    }

    // 侧面3和4 (沿 e1 方向)
    for (int i = 0; i <= steps1; ++i)
    {
        float u = i * du;
        Eigen::Vector3f base3 = v0 + u * e1;
        Eigen::Vector3f base4 = v3 + u * e1;
        for (int k = 0; k <= steps_t; ++k)
        {
            float t = (k * dt - 0.5f) * thickness;
            cloud->push_back(PointT((base3 + t * n).x(), (base3 + t * n).y(), (base3 + t * n).z()));
            cloud->push_back(PointT((base4 + t * n).x(), (base4 + t * n).y(), (base4 + t * n).z()));
        }
    }

    return cloud;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scene_generator_node");
    ros::NodeHandle nh;

    // 创建发布者，latch=true 保证新连接的订阅者能收到最后一帧静态场景
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("scene_pointcloud", 1, true);

    CloudT::Ptr scene_cloud(new CloudT);

    float thickness = 0.05;  // 厚度 10cm = 0.1m
    float resolution = 0.10; // 点云采样间距 10cm
    float wall_height = 3.0; // 场景物体的高度（如墙高）

    ROS_INFO("开始生成点云场景...");

    // ================= 场景物体 1：地面 =================
    std::vector<Eigen::Vector3f> ground_vertices = {
        Eigen::Vector3f(0.65, 0.75, 0.0),
        Eigen::Vector3f(0.65, -3.25, 0.0),
        Eigen::Vector3f(-3.35, -3.25, 0.0), // 对角点
        Eigen::Vector3f(-3.35, 0.75, 0.0)};
    *scene_cloud += *generatePlate(ground_vertices, thickness, resolution);

    // ================= 场景物体 2：墙 =================
    std::vector<Eigen::Vector3f> wall_vertices1 = {
        Eigen::Vector3f(0.65, 0.75, 0.0),
        Eigen::Vector3f(0.65, -3.25, 0.0),
        Eigen::Vector3f(0.65, -3.25, wall_height), // 墙高3米
        Eigen::Vector3f(0.65, 0.75, wall_height)};
    *scene_cloud += *generatePlate(wall_vertices1, thickness, resolution);

    std::vector<Eigen::Vector3f> wall_vertices2 = {
        Eigen::Vector3f(0.65, 0.75, 0.0),
        Eigen::Vector3f(-3.35, 0.75, 0.0),
        Eigen::Vector3f(-3.35, 0.75, wall_height), //
        Eigen::Vector3f(0.65, 0.75, wall_height)};
    *scene_cloud += *generatePlate(wall_vertices2, thickness, resolution);

    std::vector<Eigen::Vector3f> wall_vertices3 = {
        Eigen::Vector3f(0.65, -3.25, 0.0),
        Eigen::Vector3f(-3.35, -3.25, 0.0),
        Eigen::Vector3f(-3.35, -3.25, wall_height), //
        Eigen::Vector3f(0.65, -3.25, wall_height)};
    *scene_cloud += *generatePlate(wall_vertices3, thickness, resolution);

    std::vector<Eigen::Vector3f> wall_vertices4 = {
        Eigen::Vector3f(-3.35, 0.75, 0.0),
        Eigen::Vector3f(-3.35, -3.25, 0.0),
        Eigen::Vector3f(-3.35, -3.25, wall_height), //
        Eigen::Vector3f(-3.35, 0.75, wall_height)};
    *scene_cloud += *generatePlate(wall_vertices4, thickness, resolution);
    // ================= 场景物体 3：倾斜的坡道/板子 =================
    std::vector<Eigen::Vector3f> obs1_vertices = {
        Eigen::Vector3f(0.65, -0.8, 0.0),
        Eigen::Vector3f(-1.35, -0.8, 0.0),
        Eigen::Vector3f(-1.35, -0.8, wall_height), //
        Eigen::Vector3f(0.65, -0.8, wall_height)};
    *scene_cloud += *generatePlate(obs1_vertices, thickness, resolution);

    std::vector<Eigen::Vector3f> obs2_vertices = {
        Eigen::Vector3f(-1.3, -0.85, 0.0),
        Eigen::Vector3f(-1.3, -2.05, 0.0),
        Eigen::Vector3f(-1.3, -2.05, wall_height), //
        Eigen::Vector3f(-1.3, -0.85, wall_height)};
    *scene_cloud += *generatePlate(obs2_vertices, thickness, resolution);

    // ================= 场景物体 4：cols =================

    float rand_obs1[2] = {-2.05, -0.8};
    float rand_obs2[2] = {-2.65, -2.05};

    std::vector<Eigen::Vector3f> rand_obs1_vertices = {
        Eigen::Vector3f(rand_obs1[0]+0.1, rand_obs1[1], 0.0),
        Eigen::Vector3f(rand_obs1[0]-0.1, rand_obs1[1], 0.0),
        Eigen::Vector3f(rand_obs1[0]-0.1, rand_obs1[1], wall_height), //
        Eigen::Vector3f(rand_obs1[0]+0.1, rand_obs1[1], wall_height)};
    *scene_cloud += *generatePlate(rand_obs1_vertices, thickness, resolution);


    std::vector<Eigen::Vector3f> rand_obs2_vertices = {
        Eigen::Vector3f(rand_obs2[0]+0.1, rand_obs2[1], 0.0),
        Eigen::Vector3f(rand_obs2[0]-0.1, rand_obs2[1], 0.0),
        Eigen::Vector3f(rand_obs2[0]-0.1, rand_obs2[1], wall_height), //
        Eigen::Vector3f(rand_obs2[0]+0.1, rand_obs2[1], wall_height)};
    *scene_cloud += *generatePlate(rand_obs2_vertices, thickness, resolution);


    // ================= 点云后处理：体素滤波降采样去重 =================
    // 因为表面和侧面交界处会有重复点，使用体素滤波让点云更干净均匀
    CloudT::Ptr filtered_cloud(new CloudT);
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(scene_cloud);
    sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm 的体素网格
    sor.filter(*filtered_cloud);

    // 转换为 ROS 消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*filtered_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";

    ROS_INFO("场景生成完毕！总点数: %lu", filtered_cloud->size());

    // 循环发布场景
    ros::Rate rate(10); // 10Hz
    while (ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();
        pub.publish(cloud_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}