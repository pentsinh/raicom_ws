#include <template.h>
// #include <ego.h>
#include <algorithm>
#include <cmath>
#include <utility>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yolov8_ros_msgs/BoundingBoxes.h>

// 全局变量定义
int mission_num = 0; // 任务标志位

int laser_altitude = 0.9; // 激光打靶高度

// 目标点坐标数组
vector<float> target_array_x;
vector<float> target_array_y;

// 障碍物坐标
float obs_array[4][2] = {{-2.05, -0.8}, {-2.65, -0.8}, {-2.05, -2.05}, {-2.65, -2.05}}; // 障碍物坐标数组
// float obs_array[4][2]={{-2.05+0.65,-0.8+0.75},{-2.65+0.65,-0.8+0.75},{-2.05+0.65,-2.05+0.75},{-2.65+0.65,-2.05+0.75}}; // 障碍物坐标数组
float obs_radious = 0.15;                     // 障碍物半径
float obs_height = 1.0;                       // 障碍物高度
bool is_obs[] = {false, false, false, false}; // 障碍物是否存在标志位
bool case4_initialized = false;
bool case4_obs_ready = false;

// 各个目标点坐标
float target_array[7][2]; // 目标点坐标数组
float if_debug = 0;       // 是否开启调试模式

float err_max = 0; // 最大误差
// float err_max_ego = 0; // ego规划器最大误差

string laser_target[2]; // 激光靶标识别结果
bool laser_target_ready = false;
bool case6_initialized = false;

string drop_target; // 投货靶标识别结果
bool drop_target_ready = false;
bool case8_initialized = false;

static std::string extractClassSuffix(const std::string &class_name)
{
    if (class_name.empty())
    {
        return "";
    }

    return std::string(1, class_name.back());
}

void yoloBoundingBoxesCb(const yolov8_ros_msgs::BoundingBoxes::ConstPtr &msg)
{
    if (mission_num == 6)
    {
        if (msg->bounding_boxes.size() < 2)
        {
            return;
        }

        std::vector<yolov8_ros_msgs::BoundingBox> filtered_boxes(msg->bounding_boxes.begin(), msg->bounding_boxes.end());
        if (filtered_boxes.size() == 3)
        {
            auto max_y_it = std::max_element(
                filtered_boxes.begin(), filtered_boxes.end(),
                [](const auto &lhs, const auto &rhs)
                {
                    const double lhs_center_y = (static_cast<double>(lhs.ymin) + static_cast<double>(lhs.ymax)) / 2.0;
                    const double rhs_center_y = (static_cast<double>(rhs.ymin) + static_cast<double>(rhs.ymax)) / 2.0;
                    return lhs_center_y < rhs_center_y;
                });
            filtered_boxes.erase(max_y_it);
        }

        std::vector<std::pair<double, std::string>> centers_and_classes;
        centers_and_classes.reserve(filtered_boxes.size());

        for (const auto &box : filtered_boxes)
        {
            const double center_x = (static_cast<double>(box.xmin) + static_cast<double>(box.xmax)) / 2.0;
            centers_and_classes.emplace_back(center_x, extractClassSuffix(box.Class));
        }

        std::sort(centers_and_classes.begin(), centers_and_classes.end(),
                  [](const auto &lhs, const auto &rhs)
                  {
                      return lhs.first < rhs.first;
                  });

        laser_target[0] = centers_and_classes.front().second;
        laser_target[1] = centers_and_classes.back().second;
        laser_target_ready = true;
        return;
    }

    if (mission_num == 8 && !msg->bounding_boxes.empty())
    {
        drop_target = extractClassSuffix(msg->bounding_boxes.front().Class);
        drop_target_ready = true;
    }
}

void stretchedCloudCb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (mission_num != 4)
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    int obs_count[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        is_obs[i] = false;
    }

    for (const auto &point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        if (point.z < obs_height * 0.5 || point.z > obs_height * 1.5)
        {
            continue;
        }

        for (int i = 0; i < 4; ++i)
        {
            const float dx = point.x - obs_array[i][0];
            const float dy = point.y - obs_array[i][1];
            if (dx * dx + dy * dy <= obs_radious * obs_radious)
            {
                obs_count[i]++;
            }
        }
    }

    is_obs[0] = obs_count[0] >= obs_count[1];
    is_obs[1] = obs_count[1] > obs_count[0];
    is_obs[2] = obs_count[2] >= obs_count[3];
    is_obs[3] = obs_count[3] > obs_count[2];

    case4_obs_ready = true;
}

void print_param()
{
    std::cout << "=== 目标点参数 ===" << std::endl;
    for (size_t i = 0; i < target_array_x.size(); ++i)
    {
        std::cout << "target" << (i + 1) << "_x: " << target_array_x[i] << std::endl;
        std::cout << "target" << (i + 1) << "_y: " << target_array_y[i] << std::endl;
    }
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    // std::cout << "err_max_ego: " << err_max_ego << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    if (if_debug == 1)
        cout << "自动offboard" << std::endl;
    else
        cout << "遥控器offboard" << std::endl;
}

int main(int argc, char **argv)
{
    // 防止中文输出乱码
    setlocale(LC_ALL, "");

    // 初始化ROS节点
    ros::init(argc, argv, "raicom_mission");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 订阅ego_planner规划出来的结果
    // ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);
    // ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

    // 发布ego_planner目标和完成信号
    // planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);
    // finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);
    // ego_planner_mode_pub = nh.advertise<std_msgs::Int32>("/ego_planner_mode", 10);

    // 订阅mavros相关话题
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    // 发布无人机多维控制话题
    ros::Publisher mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);

    // arduino控制话题
    ros::Publisher arduino_cmd_pub = nh.advertise<std_msgs::Int32>("/arduino_control", 100);
    ros::Publisher cam_index_pub = nh.advertise<std_msgs::Int32>("/cam_index", 1, true);

    // 视觉识别结果订阅
    ros::Subscriber yolo_boxes_sub = nh.subscribe("/yolov8/BoundingBoxes", 1, yoloBoundingBoxesCb);
    ros::Subscriber stretched_cloud_sub = nh.subscribe("/map_accumulator/downsampled_map", 1, stretchedCloudCb);
    // ros::Subscriber stretched_cloud_sub = nh.subscribe("/cloud_registered", 1, stretchedCloudCb);

    // 创建服务客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // 设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(20);

    // 参数读取
    for (int i = 0; i < 7; ++i)
    {
        pnh.param<float>("target" + std::to_string(i + 1) + "_x", target_array[i][0], 0);
        pnh.param<float>("target" + std::to_string(i + 1) + "_y", target_array[i][1], 0);
    }

    pnh.param<float>("if_debug", if_debug, 1);
    pnh.param<float>("err_max", err_max, 0.1);
    // pnh.param<float>("err_max_ego", err_max_ego, 0.3);

    target_array_x.clear();
    target_array_y.clear();
    for (int i = 0; i < 7; ++i)
    {
        target_array_x.push_back(target_array[i][0]);
        target_array_y.push_back(target_array[i][1]);
    }

    print_param();
    ros::spinOnce();
    rate.sleep();
    while (local_pos.pose.pose.position.z == 0)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_WARN("%f", init_position_z_take_off);

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    ros::spinOnce();
    rate.sleep();
    std::cin >> choice;
    if (choice != 1)
        return 0;
    // 等待连接到飞控
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    // 设置无人机的期望位置

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "ok" << std::endl;

    // 定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    // 定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // 记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (if_debug == 1)
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }
            }
            else
            {
                ROS_INFO("Waiting for OFFBOARD mode");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        // 当无人机到达起飞点高度后，悬停0秒后进入任务模式，提高视觉效果
        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < err_max)
        {

            mission_num = 3;
            last_request = ros::Time::now();
            break;
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    int tmp_mission = 3;

    while (ros::ok())
    {
        if (tmp_mission != mission_num)
        {
            tmp_mission = mission_num;
            printf("change mission_num = %d\r\n", mission_num);
        }

        printf("mission_num = %d\r\n", mission_num);

        switch (mission_num)
        {
        // case 2: // 前往目标点1
        // {
        //     float target_x = target_array_x[0];
        //     float target_y = target_array_y[0];
        //     if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
        //     {
        //         last_request = ros::Time::now();
        //         mission_num++;
        //     }
        //     break;
        // }
        case 3: // 前往目标点2
        {
            float target_x = target_array_x[1];
            float target_y = target_array_y[1];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 4;
            }
            break;
        }

        case 4: // 判断障碍物位置
        {
            if (!case4_initialized)
            {
                case4_obs_ready = false;
                for (int i = 0; i < 4; ++i)
                {
                    is_obs[i] = false;
                }
                case4_initialized = true;
            }

            float target_x = target_array_x[1];
            float target_y = target_array_y[1];
            mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max);

            if (case4_obs_ready)
            {
                case4_initialized = false;
                last_request = ros::Time::now();
                mission_num = 41;
            }
            break;
        }
        case 41: // 进入避障区
        {
            float target_x, target_y;
            if (!is_obs[0]) // 障碍物1不存在
            {
                target_x = obs_array[0][0];
                target_y = obs_array[0][1] + 0.8;
            }
            else
            {
                target_x = obs_array[1][0];
                target_y = obs_array[1][1] + 0.8;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 5;
            }
            break;
        }

        case 5: // 前往避障点1
        {
            float target_x, target_y;
            if (!is_obs[0]) // 障碍物1不存在
            {
                target_x = obs_array[0][0];
                target_y = obs_array[0][1] - 0.6;
            }
            else
            {
                target_x = obs_array[1][0];
                target_y = obs_array[1][1] - 0.6;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 51;
            }
            break;
        }

        case 51: // 前往避障点2
        {
            float target_x, target_y;
            if (!is_obs[2]) // 障碍物3不存在
            {
                target_x = obs_array[2][0];
                target_y = obs_array[2][1] + 0.6;
            }
            else
            {
                target_x = obs_array[3][0];
                target_y = obs_array[3][1] + 0.6;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 52;
            }
            break;
        }

        case 52: // 出避障区
        {
            float target_x, target_y;
            if (!is_obs[2]) // 障碍物3不存在
            {
                target_x = obs_array[2][0];
                target_y = obs_array[2][1] - 0.5;
            }
            else
            {
                target_x = obs_array[3][0];
                target_y = obs_array[3][1] - 0.5;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 6;
            }
            break;
        }

        case 6: // 识别激光靶标
        {
            if (!case6_initialized)
            {
                laser_target_ready = false;
                laser_target[0].clear();
                laser_target[1].clear();
                last_request = ros::Time::now();
                case6_initialized = true;
            }

            std_msgs::Int32 cam_index_msg;
            cam_index_msg.data = 1;
            cam_index_pub.publish(cam_index_msg);

            float target_x = target_array_x[3];
            float target_y = target_array_y[3];
            mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max);

            if (laser_target_ready)
            {
                ROS_INFO("Laser targets recognized: left=%s, right=%s",
                         laser_target[0].c_str(), laser_target[1].c_str());
                case6_initialized = false;
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }

        case 7: // 前往目标点5
        {
            float target_x = target_array_x[4];
            float target_y = target_array_y[4];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 8: // 识别投货靶标
        {
            if (!case8_initialized)
            {
                drop_target.clear();
                drop_target_ready = false;
                last_request = ros::Time::now();
                case8_initialized = true;
            }

            std_msgs::Int32 cam_index_msg;
            cam_index_msg.data = 0;
            cam_index_pub.publish(cam_index_msg);

            float target_x = target_array_x[4];
            float target_y = target_array_y[4];
            mission_pos_cruise(target_x, target_y, ALTITUDE / 2, 0, err_max);

            if (drop_target_ready && ros::Time::now() - last_request >= ros::Duration(3.0))
            {
                ROS_INFO("Drop target recognized: %s", drop_target.c_str());
                case8_initialized = false;
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 9: // 投货
        {
            float target_x = target_array_x[4];
            float target_y = target_array_y[4];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE / 2, 0, err_max / 2))
            {
                std_msgs::Int32 arduino_cmd_msg;
                arduino_cmd_msg.data = 2; // 2 代表关闭电磁铁
                arduino_cmd_pub.publish(arduino_cmd_msg);
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 10: // 前往打把点
        {
            float target_x, target_y;
            target_x = target_array_x[5];
            target_y = target_array_y[5];

            // if (mission_pos_cruise(target_x, target_y, laser_altitude, 0, err_max))
            if (mission_pos_cruise(target_x, target_y, ALTITUDE - 0.2, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 11: // 打靶
        {
            // 刚进入该状态时（0.1秒内）发布打开激光笔指令
            if (ros::Time::now() - last_request < ros::Duration(0.1))
            {
                std_msgs::Int32 arduino_cmd_msg;
                arduino_cmd_msg.data = 1; // 1 代表打开激光笔
                arduino_cmd_pub.publish(arduino_cmd_msg);
            }
            // 防止漂移
            float target_x, target_y;
            if (laser_target[0] == drop_target)
            {
                target_x = target_array_x[5];
                target_y = target_array_y[5];
            }
            else
            {
                target_x = target_array_x[6];
                target_y = target_array_y[6];
            }
            // mission_pos_cruise(target_x, target_y, laser_altitude, 0, err_max);
            mission_pos_cruise(target_x, target_y, ALTITUDE - 0.2, 0, err_max);
            // 悬停3秒后进入下一状态
            if (ros::Time::now() - last_request >= ros::Duration(3.0))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }

        /*********开始返程*********/
        case 12: // 前往目标点7
        {
            float target_x = target_array_x[6];
            float target_y = target_array_y[6];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }

        case 13: // 准备进入避障区
        {
            float target_x, target_y;
            if (!is_obs[2]) // 障碍物3不存在
            {
                target_x = obs_array[2][0];
                target_y = obs_array[2][1] - 0.6;
            }
            else
            {
                target_x = obs_array[3][0];
                target_y = obs_array[3][1] - 0.6;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 131;
            }
            break;
        }

        case 131: // 前往避障点2
        {
            float target_x, target_y;
            if (!is_obs[2]) // 障碍物3不存在
            {
                target_x = obs_array[2][0];
                target_y = obs_array[2][1] + 0.6;
            }
            else
            {
                target_x = obs_array[3][0];
                target_y = obs_array[3][1] + 0.6;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 132;
            }
            break;
        }

        case 132: // 前往避障点1
        {
            float target_x, target_y;
            if (!is_obs[0]) // 障碍物1不存在
            {
                target_x = obs_array[0][0];
                target_y = obs_array[0][1] - 0.6;
            }
            else
            {
                target_x = obs_array[1][0];
                target_y = obs_array[1][1] - 0.6;
            }
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num = 14;
            }
            break;
        }

        case 14: // 出避障区
        {
            float target_x, target_y;
            if (!is_obs[0]) // 障碍物1不存在
            {
                target_x = obs_array[0][0];
                target_y = obs_array[0][1] + 0.8;
            }
            else
            {
                target_x = obs_array[1][0];
                target_y = obs_array[1][1] + 0.8;
            }

            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 15: // 前往目标点8
        {
            // float target_x = target_array_x[7];
            // float target_y = target_array_y[7];
            if (mission_pos_cruise(0.0, 0.0, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 16: // 在降落点降落
        {
            if (mission_pos_cruise(0.0, 0.0, 0.2, 0, err_max))
            {
                mission_num = -1; // 任务结束
            }
            else if (ros::Time::now() - last_request >= ros::Duration(5.0))
            {
                mission_num = -1; // 任务结束
            }
            break;
        }
        default:
            break;
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();

        if (mission_num == -1)
        {
            return 0;
        }
    }

    return 0;
}
