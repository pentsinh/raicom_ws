#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <deque>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <visualization_msgs/Marker.h>

using namespace std;

#define ALTITUDE 1.1

mavros_msgs::PositionTarget setpoint_raw;


/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
// ros::Publisher ego_odom_pub;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }

    // 仅将高度(z)替换为 ALTITUDE，其余保持原始 odom 不变
    // nav_msgs::Odometry ego_odom = *msg;
    // ego_odom.pose.pose.position.z = ALTITUDE;
    // ego_odom.header.stamp = ros::Time::now();

    // ROS_INFO("CHEATING");
    // ego_odom_pub.publish(ego_odom);
}

// 计算目标点指向的偏航角（弧度）
inline float calculate_yaw(float target_x, float target_y)
{
    float dx = target_x - local_pos.pose.pose.position.x;
    float dy = target_y - local_pos.pose.pose.position.y;
    return atan2(dy, dx);
}

/************************************************************************
函数 3: 起飞系位置控制
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x, y, z, yaw * 180.0 / M_PI);
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}
/************************************************************************
函数 4：机体系位置控制
！！！使用时需立刻退出，不要在if中放置时间，否则会一直累加当前位置！！！
*************************************************************************/
float current_position_cruise_last_position_x = 0;
float current_position_cruise_last_position_y = 0;
bool current_position_cruise_flag = false;
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max);
bool current_position_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (current_position_cruise_flag == false)
    {
        current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
        current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
        current_position_cruise_flag = true;
    }
    // float p_xy = 0.8;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + current_position_cruise_last_position_x;
    setpoint_raw.position.y = y + current_position_cruise_last_position_y;
    setpoint_raw.position.z = z;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, yaw * 180.0 / M_PI, x + current_position_cruise_last_position_x, y + current_position_cruise_last_position_y, z, target_yaw * 180.0 / M_PI);
    if (fabs(local_pos.pose.pose.position.x - current_position_cruise_last_position_x - x) < error_max && fabs(local_pos.pose.pose.position.y - current_position_cruise_last_position_y - y) < error_max && fabs(local_pos.pose.pose.position.z - z) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        current_position_cruise_flag = false;
        setpoint_raw.yaw_rate = 0.0; // 重置yaw_rate
        setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 + /*1024 + */ 2048;
        setpoint_raw.yaw = yaw; // 确保最终yaw角度正确
        return true;
    }
    return false;
}

/************************************************************************
函数 5:降落
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
ros::Time precision_land_last_time;
bool precision_land();
bool precision_land()
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    setpoint_raw.position.z = -0.15;
    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    if (ros::Time::now() - precision_land_last_time > ros::Duration(5.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        return true;
    }
    return false;
}

/************************************************************************
函数 6:lib计时器
！！！若使用超限，需要将lib_time_init_flag置为true！！！
*************************************************************************/
ros::Time last_time;
bool lib_time_init_flag = true; // 记录是否为第一次触发
bool lib_time_record_func(float duration, ros::Time current_time)
{
    // 静态变量，记录上次触发时间
    if (lib_time_init_flag)
    {
        last_time = current_time;
        lib_time_init_flag = false; // 只在第一次调用时初始化
    }

    ros::Duration elapsed = current_time - last_time;

    // 如果已经过去超过 duration 秒，返回 true 并更新时间
    if (elapsed.toSec() >= duration)
    {
        lib_time_init_flag = true;
        return true;
    }
    return false;
}

/************************************************************************
函数 7: hover保持悬停time秒
*************************************************************************/
bool hover_started = false;
ros::Time hover_last_time;
float hover_x = 0;
float hover_y = 0;
float hover_z = 0;
float hover_yaw = 0;
bool hover(float time_duration);
bool hover(float time_duration)
{
    if (!hover_started)
    {
        hover_last_time = ros::Time::now();
        hover_started = true;
        hover_x = local_pos.pose.pose.position.x;
        hover_y = local_pos.pose.pose.position.y;
        hover_z = local_pos.pose.pose.position.z;
        hover_yaw = yaw;
    }
    ros::Duration elapsed = ros::Time::now() - hover_last_time;
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = hover_x;
    setpoint_raw.position.y = hover_y;
    setpoint_raw.position.z = hover_z;
    setpoint_raw.yaw = hover_yaw;
    ROS_INFO("Hovering at (%.2f, %.2f, %.2f) for %.2f seconds", hover_x, hover_y, hover_z, elapsed.toSec());
    // 如果已经过去超过 time_duration 秒，返回 true 并重置状态
    if (elapsed.toSec() >= time_duration)
    {
        hover_started = false; // Reset for next hover
        return true;
    }
    return false;
}

/************************************************************************
函数 8: 绕圈，以传入点为圆心，当前点到传入点距离为半径，Ts为周期绕圆
*************************************************************************/
bool init_circle_flag = false;
float init_plane_x = 0;
float init_plane_y = 0;
float circle_radius = 0;
float beta_angle = 0;
ros::Time circle_start_time;
bool circle(float center_x, float center_y, float Ts);
bool circle(float center_x, float center_y, float Ts)
{
    if (!init_circle_flag)
    {
        // 记录初始位置（用于计算初始角度）
        init_plane_x = local_pos.pose.pose.position.x;
        init_plane_y = local_pos.pose.pose.position.y;
        circle_radius = sqrt(pow(init_plane_x - center_x, 2) + pow(init_plane_y - center_y, 2));

        // 初始角度：从圆心指向无人机的向量的角度（标准 atan2(y, x)）
        // 注意：atan2(dy, dx)，其中 dy = y - center_y, dx = x - center_x
        beta_angle = atan2(init_plane_y - center_y, init_plane_x - center_x);

        init_circle_flag = true;
        circle_start_time = ros::Time::now();
    }

    ros::Time current_time = ros::Time::now();
    float elapsed_time = (current_time - circle_start_time).toSec();
    float theta = (2 * M_PI / Ts) * elapsed_time; // 角速度 * 时间

    // 当前目标角度 = 初始角度 + theta（逆时针旋转）
    float current_angle = beta_angle + theta;

    // 目标位置：圆心 + 半径 * (cos, sin)
    setpoint_raw.position.x = center_x + circle_radius * cos(current_angle);
    setpoint_raw.position.y = center_y + circle_radius * sin(current_angle);
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0; // 或者可以设置为朝向飞行方向：current_angle

    // 设置 type_mask 和 coordinate_frame（保持你的设置）
    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048; // 位置控制
    setpoint_raw.coordinate_frame = 1;                                  // FRAME_LOCAL_NED

    ROS_INFO("Circle flying to (%.2f, %.2f, %.2f)",
             setpoint_raw.position.x, setpoint_raw.position.y, setpoint_raw.position.z);
    ROS_WARN("Angle: %.2f deg, Time: %.2f s", current_angle * 180.0 / M_PI, elapsed_time);

    float real_radius = sqrt(pow(local_pos.pose.pose.position.x - center_x, 2) +
                             pow(local_pos.pose.pose.position.y - center_y, 2));
    ROS_WARN("Expected radius: %.2f, Real radius: %.2f", circle_radius, real_radius);

    if (theta >= 2 * M_PI)
    {
        init_circle_flag = false;
        ROS_INFO("Circle flight complete.");
        return true;
    }
    else
    {
        return false;
    }
}
/************************************************************************
函数 9: 绕圈（提高版，基于角度增量，绕完一圈返回 true）
传入圆心 (center_x, center_y) 和每次角度增量 d_angle（弧度，可正可负）
每次调用生成下一个目标点，并累计角度
当累计旋转角度 ≥ 2π 时，返回 true（表示一圈完成）
*************************************************************************/

bool init_circle_advanced_flag = false;
float circle_adv_radius = 0;
float circle_adv_center_x = 0;
float circle_adv_center_y = 0;
float planned_angle = 0;

// 实际飞行状态跟踪
float actual_start_angle = 0;
float actual_unwrapped_angle = 0;
float prev_actual_angle = 0;
float actual_total_delta = 0;

bool circle_advanced(float center_x, float center_y, float d_angle)
{
    float plane_x = local_pos.pose.pose.position.x;
    float plane_y = local_pos.pose.pose.position.y;
    float curr_actual_angle = atan2(plane_y - center_y, plane_x - center_x);

    if (!init_circle_advanced_flag)
    {
        // 初始化轨迹
        circle_adv_radius = sqrt(pow(plane_x - center_x, 2) + pow(plane_y - center_y, 2));
        circle_adv_center_x = center_x;
        circle_adv_center_y = center_y;

        // 初始化实际角度跟踪
        actual_start_angle = curr_actual_angle;
        actual_unwrapped_angle = curr_actual_angle;
        prev_actual_angle = curr_actual_angle;
        actual_total_delta = 0;

        // 初始化计划角度（用于生成 setpoint）
        planned_angle = curr_actual_angle;

        init_circle_advanced_flag = true;

        ROS_INFO("Advanced circle initialized. Radius: %.2f, Start angle: %.2f deg",
                 circle_adv_radius, actual_start_angle * 180.0 / M_PI);
    }

    // === 1. 更新计划轨迹（用于发 setpoint）===
    planned_angle += d_angle;
    setpoint_raw.position.x = circle_adv_center_x + circle_adv_radius * cos(planned_angle);
    setpoint_raw.position.y = circle_adv_center_y + circle_adv_radius * sin(planned_angle);
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    setpoint_raw.type_mask = 8 + 16 + 32 + 64 + 128 + 256 + 512 + 2048;
    setpoint_raw.coordinate_frame = 1;

    // === 2. 更新实际累计转角（关键！）===
    // 解卷绕：处理角度从 π → -π 或 -π → π 的跳变
    float delta = curr_actual_angle - prev_actual_angle;
    if (delta > M_PI)
        delta -= 2 * M_PI;
    if (delta < -M_PI)
        delta += 2 * M_PI;
    actual_unwrapped_angle += delta;
    prev_actual_angle = curr_actual_angle;

    actual_total_delta = fabs(actual_unwrapped_angle - actual_start_angle);

    // === 3. 日志 ===
    ROS_INFO("Target: (%.2f, %.2f), Planned Angle: %.2f°, Actual Angle: %.2f°, Actual Δ: %.2f°",
             setpoint_raw.position.x, setpoint_raw.position.y,
             planned_angle * 180.0 / M_PI,
             curr_actual_angle * 180.0 / M_PI,
             actual_total_delta * 180.0 / M_PI);

    float real_radius = sqrt(pow(plane_x - circle_adv_center_x, 2) +
                             pow(plane_y - circle_adv_center_y, 2));
    ROS_WARN("Expected radius: %.2f, Real radius: %.2f", circle_adv_radius, real_radius);

    // === 4. 判断：基于实际飞行角度是否满一圈 ===
    if (actual_total_delta >= 2 * M_PI - M_PI / 180 * 45) // 提前退出
    {
        init_circle_advanced_flag = false;
        ROS_INFO("Actual full circle completed! Total actual rotation: %.2f°",
                 actual_total_delta * 180.0 / M_PI);
        return true;
    }
    else
    {
        return false;
    }
}

/************************************************************************
函数 10: rviz当前位置轨迹可视化
*************************************************************************/
// 全局变量
ros::Publisher marker_pub;
std::deque<geometry_msgs::Point> path_points;
const int MAX_POINTS = 1000000000;   // 最大轨迹点数
const float TRAJECTORY_WIDTH = 0.05; // 轨迹线宽
// 位置回调函数
inline void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    // 添加新点到轨迹
    geometry_msgs::Point new_point;
    new_point.x = msg->pose.position.x;
    new_point.y = msg->pose.position.y;
    new_point.z = msg->pose.position.z;

    path_points.push_back(new_point);

    // 限制轨迹长度
    if (path_points.size() > MAX_POINTS)
    {
        path_points.pop_front();
    }

    // 创建轨迹Marker
    visualization_msgs::Marker trajectory;
    // trajectory.header.frame_id = msg->header.frame_id;
    trajectory.header.frame_id = "world";
    trajectory.header.stamp = ros::Time::now();
    trajectory.ns = "drone_trajectory";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::Marker::ADD;

    // 轨迹样式设置
    trajectory.scale.x = TRAJECTORY_WIDTH; // 线宽
    trajectory.color.r = 0.0;              // 红色分量
    trajectory.color.g = 1.0;              // 绿色分量
    trajectory.color.b = 0.0;              // 蓝色分量
    trajectory.color.a = 0.8;              // 透明度

    // 轨迹点数据
    trajectory.points = std::vector<geometry_msgs::Point>(
        path_points.begin(), path_points.end());

    // 发布轨迹
    marker_pub.publish(trajectory);

    // 创建当前位置标记
    visualization_msgs::Marker position_marker;
    position_marker.header = trajectory.header;
    position_marker.ns = "drone_position";
    position_marker.id = 1;
    position_marker.type = visualization_msgs::Marker::SPHERE;
    position_marker.action = visualization_msgs::Marker::ADD;
    position_marker.pose.position = new_point;
    position_marker.pose.orientation.w = 1.0;
    position_marker.scale.x = 0.3; // 球体直径
    position_marker.scale.y = 0.3;
    position_marker.scale.z = 0.3;
    position_marker.color.r = 1.0;
    position_marker.color.g = 0.0;
    position_marker.color.b = 0.0;
    position_marker.color.a = 1.0;

    // 发布位置标记
    marker_pub.publish(position_marker);
}
