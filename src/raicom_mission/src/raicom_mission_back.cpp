#include <template.h>
#include <ego.h>
#include <algorithm>
#include <utility>
#include <yolov8_ros_msgs/BoundingBoxes.h>

// 全局变量定义
int mission_num = 0; // 任务标志位

int laser_altitude = 0.9; //激光打靶高度

// 目标点坐标数组
vector<float> target_array_x;
vector<float> target_array_y;

// 各个目标点坐标
float target_array[7][2]; // 目标点坐标数组
float if_debug = 0;       // 是否开启调试模式

float err_max = 0;     // 最大误差
float err_max_ego = 0; // ego规划器最大误差

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

        std::vector<std::pair<double, std::string>> centers_and_classes;
        centers_and_classes.reserve(msg->bounding_boxes.size());

        for (const auto &box : msg->bounding_boxes)
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
    std::cout << "err_max_ego: " << err_max_ego << std::endl;
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
    ros::init(argc, argv, "ego_demo");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 订阅ego_planner规划出来的结果
    ros::Subscriber ego_sub = nh.subscribe("/position_cmd", 100, ego_sub_cb);
    ros::Subscriber rec_traj_sub = nh.subscribe("/rec_traj", 100, rec_traj_cb);

    // 发布ego_planner目标和完成信号
    planner_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/ego_planner/goal", 100);
    finish_ego_pub = nh.advertise<std_msgs::Bool>("/finish_ego", 1);
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
    pnh.param<float>("err_max_ego", err_max_ego, 0.3);

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
        // 当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
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
                mission_num++;
            }
            break;
        }
        case 4: // 前往目标点3
        {
            float target_x = target_array_x[2];
            float target_y = target_array_y[2];

            if (pub_ego_goal(target_x, target_y, ALTITUDE, err_max_ego))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 5: // 前往目标点4
        {
            float target_x = target_array_x[3];
            float target_y = target_array_y[3];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
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
            mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max);

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
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
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
            // if (mission_pos_cruise(target_x, target_y, laser_altitude, 0, err_max))
            if (mission_pos_cruise(target_x, target_y, ALTITUDE-0.2, 0, err_max))
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
            mission_pos_cruise(target_x, target_y, ALTITUDE-0.2, 0, err_max);
            // 悬停3秒后进入下一状态
            if (ros::Time::now() - last_request >= ros::Duration(3.0))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }

        /*********开始返程*********/
        case 12: // 前往目标点4
        {
            float target_x = target_array_x[3];
            float target_y = target_array_y[3];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 13: // 前往目标点3
        {
            float target_x = target_array_x[2];
            float target_y = target_array_y[2];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 14: // 前往目标点2
        {
            float target_x = target_array_x[1];
            float target_y = target_array_y[1];

            if (pub_ego_goal(target_x, target_y, ALTITUDE, err_max_ego))
            {
                last_request = ros::Time::now();
                mission_num++;
            }
            break;
        }
        case 15: // 前往目标点8
        {
            float target_x = target_array_x[7];
            float target_y = target_array_y[7];
            if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0, err_max))
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
