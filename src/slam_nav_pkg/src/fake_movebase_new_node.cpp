#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/tf.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>
#include <queue>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>

// 全局变量

int sock_cmd = -1;    // 已连接客户端
int server_sock = -1; // 服务端监听 socket
bool map_received_ = false;
nav_msgs::OccupancyGrid current_map_;
bool localmap_received = false;
nav_msgs::OccupancyGrid current_localmap;

typedef enum
{
    Search,
    Plan,
    Back
} CarState;

// tcp服务端初始化

bool initTCPServer(const char *ip, int port)
{
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        ROS_ERROR("Failed to create server socket");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = inet_addr(ip);

    int opt = 1;
    setsockopt(server_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(server_sock, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        ROS_ERROR("Bind failed");
        return false;
    }

    if (listen(server_sock, 1) < 0)
    {
        ROS_ERROR("Listen failed");
        return false;
    }

    ROS_INFO("Waiting for CMD client to connect...");
    sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    sock_cmd = accept(server_sock, (sockaddr *)&client_addr, &client_len);
    if (sock_cmd < 0)
    {
        ROS_ERROR("Accept failed");
        return false;
    }

    ROS_INFO("Client connected: %s", inet_ntoa(client_addr.sin_addr));
    return true;
}

// 射线检测
bool isPathClear(double x0, double y0, double x1, double y1)
{
    if (!map_received_)
        return false;

    const auto &info = current_map_.info;
    double res = info.resolution;
    int width = info.width;
    int height = info.height;
    double origin_x = info.origin.position.x;
    double origin_y = info.origin.position.y;

    int x0_idx = std::floor((x0 - origin_x) / res);
    int y0_idx = std::floor((y0 - origin_y) / res);
    int x1_idx = std::floor((x1 - origin_x) / res);
    int y1_idx = std::floor((y1 - origin_y) / res);

    if (x0_idx < 0 || x0_idx >= width || y0_idx < 0 || y0_idx >= height ||
        x1_idx < 0 || x1_idx >= width || y1_idx < 0 || y1_idx >= height)
        return false;

    int dx = std::abs(x1_idx - x0_idx);
    int dy = std::abs(y1_idx - y0_idx);
    int sx = (x0_idx < x1_idx) ? 1 : -1;
    int sy = (y0_idx < y1_idx) ? 1 : -1;
    int err = dx - dy;

    int x = x0_idx;
    int y = y0_idx;

    while (true)
    {
        int index = y * width + x;
        if (index >= 0 && index < (int)current_map_.data.size())
        {
            int occ = current_map_.data[index];
            if (occ > 50)
                return false;
        }

        if (x == x1_idx && y == y1_idx)
            break;

        int e2 = 2 * err;
        if (e2 > -dy)
        {
            err -= dy;
            x += sx;
        }
        if (e2 < dx)
        {
            err += dx;
            y += sy;
        }
    }

    return true;
}

bool mutiplePathClear(double x0, double y0, double x1, double y1)
{

    bool test1 = isPathClear(x0, y0, x1, y1);
    bool test2 = isPathClear(x0 + 0.15, y0 + 0.15, x1, y1);
    bool test3 = isPathClear(x0 - 0.15, y0 - 0.15, x1, y1);
    bool test4 = isPathClear(x0 + 0.15, y0 - 0.15, x1, y1);
    bool test5 = isPathClear(x0 - 0.15, y0 + 0.15, x1, y1);
    return test1 & test2 & test3 & test4 & test5;
}

// FakeMoveBase类
class FakeMoveBase
{
public:
    // 构造函数
    FakeMoveBase(std::string name) : as_(nh_, name, boost::bind(&FakeMoveBase::executeCB, this, _1), false),
                                     action_name_(name)
    {
        as_.start();
        ROS_INFO("FakeMoveBase server started");
        exit_x_ = 2.2;
        exit_y_ = -2.2;
        exit_tolerance_ = 0.5;
        current_state_ = Search;
        state_pub_ = nh_.advertise<std_msgs::String>("/robot_state", 10);
        robot_pub = nh_.advertise<geometry_msgs::Point>("/robot_xy", 10);
        goal_pub = nh_.advertise<geometry_msgs::Point>("/goal_xy", 10);
        replan_pub = nh_.advertise<std_msgs::String>("/replan", 10);
        odom_sub_ = nh_.subscribe("/odom", 10, &FakeMoveBase::odomCallback, this);
        map_sub = nh_.subscribe("/map", 10, &FakeMoveBase::mapCallback, this);
        scan_sub = nh_.subscribe("/scan", 10, &FakeMoveBase::scanCallback, this);
        local_cost_sub = nh_.subscribe("/local_costmap", 10, &FakeMoveBase::localmapCallback, this);
        testmove = nh_.subscribe("/move_base_simple/goal", 10, &FakeMoveBase::testCallback, this);
        path_sub_ = nh_.subscribe("/planned_path", 10, &FakeMoveBase::pathPointCallback, this);
        finish_sub_ = nh_.subscribe("/planned_path_done", 10, &FakeMoveBase::pathFinishCallback, this);
    }

    // tf变换，从map系到base局部系
    bool mapGoalToRobotFrame(const geometry_msgs::Point &goal_map, double &dx, double &dy)
    {
        geometry_msgs::PointStamped goal_map_msg, goal_robot_msg;
        goal_map_msg.header.frame_id = "map";
        goal_map_msg.header.stamp = ros::Time(0); // 最新 transform
        goal_map_msg.point = goal_map;

        try
        {
            listener.transformPoint("base_link", goal_map_msg, goal_robot_msg);
            dx = goal_robot_msg.point.x;
            dy = goal_robot_msg.point.y;
            if (!std::isfinite(dx) || !std::isfinite(dy) ||
                fabs(dx) > 1e3 || fabs(dy) > 1e3)
            {
                ROS_ERROR("Invalid transform result: dx=%.2f, dy=%.2f", dx, dy);
                return false;
            }
            return true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
            return false;
        }
    }

    // 检测前方是否有障碍
    bool checkFrontObstacle(double forward_m = 0.1, double width_m = 0.1, int cost_threshold = 50)
    {
        if (current_localmap.data.empty())
        {
            ROS_WARN_THROTTLE(2.0, "No local costmap available yet.");
            return false;
        }

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(current_localmap.header.frame_id, "base_link", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "TF lookup failed: %s", ex.what());
            return false;
        }

        double robot_x = transform.getOrigin().x();
        double robot_y = transform.getOrigin().y();
        double robot_yaw = tf::getYaw(transform.getRotation());

        // costmap 信息
        int width = current_localmap.info.width;
        int height = current_localmap.info.height;
        double resolution = current_localmap.info.resolution;
        double origin_x = current_localmap.info.origin.position.x;
        double origin_y = current_localmap.info.origin.position.y;

        int forward_cells = std::max(1, static_cast<int>(std::ceil(forward_m / resolution)));
        int half_width_cells = std::max(0, static_cast<int>(std::ceil((width_m / 2.0) / resolution)));

        for (int ix = 0; ix <= forward_cells; ++ix)
        {
            double local_x = ix * resolution;
            for (int iy = -half_width_cells; iy <= half_width_cells; ++iy)
            {
                double local_y = iy * resolution;
                double wx = robot_x + local_x * std::cos(robot_yaw) - local_y * std::sin(robot_yaw);
                double wy = robot_y + local_x * std::sin(robot_yaw) + local_y * std::cos(robot_yaw);

                int mx = static_cast<int>(std::floor((wx - origin_x) / resolution));
                int my = static_cast<int>(std::floor((wy - origin_y) / resolution));

                if (mx < 0 || mx >= width || my < 0 || my >= height)
                    continue;

                int idx = my * width + mx;
                int cost = current_localmap.data[idx];

                if (cost >= cost_threshold)
                {
                    // 计算该格中心的世界坐标（米）
                    double obs_x = origin_x + (mx + 0.5) * resolution;
                    double obs_y = origin_y + (my + 0.5) * resolution;

                    ROS_WARN_THROTTLE(1.0,
                                      "Obstacle detected! cost=%d | map idx=(%d,%d) | world=(%.3f, %.3f) | robot=(%.3f, %.3f) yaw=%.1fdeg | local=(%.3f,%.3f)",
                                      cost, mx, my, obs_x, obs_y, robot_x, robot_y, robot_yaw * 180.0 / M_PI, local_x, local_y);

                    return true;
                }
            }
        }

        return false;
    }

    // 局部路径规划，避障
    void local_planner()
    {
        char cmd[3];
        sprintf(cmd, "S00");
        send(sock_cmd, cmd, strlen(cmd), 0);
        sprintf(cmd, "B00");
        send(sock_cmd, cmd, strlen(cmd), 0);
        sleep(1.2);
        sprintf(cmd, "S00");
        send(sock_cmd, cmd, strlen(cmd), 0);
        ROS_WARN("Emergency stop due to obstacle!");

        if (!scan_received_)
        {
            ROS_WARN("No scan data received, skipping local planner.");
            return;
        }

        float Rtest = 0, Ltest = 0;

        for (int i = 120; i < 140; i++)
        {
            Rtest += latest_scan_.ranges[i];
        }
        for (int i = 220; i < 240; i++)
        {
            Ltest += latest_scan_.ranges[i];
        }
        if (Rtest / 20 > 0.5 && Rtest / 20 > Ltest / 20)
        {
            sprintf(cmd, "R50");
            ROS_INFO(cmd);
            send(sock_cmd, cmd, strlen(cmd), 0);
            sleep(1);
            sprintf(cmd, "F00");
            send(sock_cmd, cmd, strlen(cmd), 0);
            sleep(1);
            sprintf(cmd, "S00");
            send(sock_cmd, cmd, strlen(cmd), 0);
        }
        else if (Ltest / 20 > 0.5)
        {
            sprintf(cmd, "L50");
            ROS_INFO(cmd);
            send(sock_cmd, cmd, strlen(cmd), 0);
            sleep(1);
            sprintf(cmd, "F00");
            send(sock_cmd, cmd, strlen(cmd), 0);
            sleep(1);
            sprintf(cmd, "S00");
            send(sock_cmd, cmd, strlen(cmd), 0);
        }
        else
        {
            ROS_WARN("No free path found, attempt... ");
        }
    }

    // 转弯控制
    void turn_control(double dx, double dy)
    {
        double angle_rad = atan2(dy, dx);
        double angle_degree = angle_rad / M_PI * 180;
        char cmd[3];

        if (angle_degree > 20)
        {
            sprintf(cmd, "S00");
            send(sock_cmd, cmd, strlen(cmd), 0);
            if (angle_degree < 90)
            {
                sprintf(cmd, "L%02.0f", angle_degree);
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
            }
            else
            {
                sprintf(cmd, "L90");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
                sprintf(cmd, "L%02.0f", angle_degree - 90);
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
            }
        }
        else if (angle_degree < -20)
        {
            sprintf(cmd, "S00");
            send(sock_cmd, cmd, strlen(cmd), 0);
            if (angle_degree > -90)
            {
                sprintf(cmd, "R%02.0f", -angle_degree);
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
            }
            else
            {
                sprintf(cmd, "R90");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
                sprintf(cmd, "R%02.0f", (-angle_degree) - 90);
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                sleep(1);
            }
        }
    }

    // 运动封装
    bool MovetoGoal(double x, double y, double tolerance)
    {
        double dx, dy;
        geometry_msgs::Point goal_point_map;
        goal_point_map.x = x;
        goal_point_map.y = y;
        goal_point_map.z = 0.0;

        if (!mapGoalToRobotFrame(goal_point_map, dx, dy))
        {
            ROS_WARN("Failed to transform goal to robot frame, aborting");
            return false;
        }
        ROS_INFO("Goal in robot frame: dx=%.2f, dy=%.2f", dx, dy);
        double distance = sqrt(dx * dx + dy * dy);
        double angle_rad = atan2(dy, dx);
        double angle_degree = angle_rad / M_PI * 180;
        char cmd[3];
        ros::Rate rate(5);
        int testcount = 0;
        turn_control(dx, dy);
        ros::Duration timeout(1.0);
        ros::Time start = ros::Time::now();
        while ((ros::Time::now() - start) < timeout)
        {
            ros::spinOnce();            // 让回调执行
            ros::Duration(0.1).sleep(); // 防止占用CPU
        }
        if (distance > tolerance)
        {
            sprintf(cmd, "F00");
            ROS_INFO(cmd);
            send(sock_cmd, cmd, strlen(cmd), 0);
        }
        while (distance > tolerance && ros::ok())
        {
            mapGoalToRobotFrame(goal_point_map, dx, dy);
            distance = sqrt(dx * dx + dy * dy);
            angle_rad = atan2(dy, dx);
            angle_degree = angle_rad / M_PI * 180;
            if (angle_degree > 30)
            {
                turn_control(dx, dy);
            }
            sprintf(cmd, "F00");
            ROS_INFO(cmd);
            send(sock_cmd, cmd, strlen(cmd), 0);

            ROS_WARN("ischeck %d", checkFrontObstacle());
            if (checkFrontObstacle())
            {
                testcount++;
                local_planner();
                if (testcount > 2)
                {
                    ROS_ERROR("Local planner stuck, marking goal as unreachable.");
                    return false;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
        return true;
    }

    geometry_msgs::Point findInnerFreePoint(const nav_msgs::OccupancyGrid &map,
                                            const geometry_msgs::Point &frontier)
    {
        int map_width = map.info.width;
        int map_height = map.info.height;
        double res = map.info.resolution;

        int fx = (frontier.x - map.info.origin.position.x) / res;
        int fy = (frontier.y - map.info.origin.position.y) / res;

        // 八方向搜索，往内退
        std::vector<std::pair<int, int>> dirs = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {-1, -1}, {1, -1}, {-1, 1}};

        int best_nx = fx;
        int best_ny = fy;
        bool found = false;

        for (auto &d : dirs)
        {
            int free_count = 0;
            for (int i = 1; i <= 8; ++i) // 最多向内看 8 格
            {
                int nx = fx + d.first * i;
                int ny = fy + d.second * i;
                if (nx < 0 || ny < 0 || nx >= map_width || ny >= map_height)
                    continue;

                int idx = ny * map_width + nx;
                if (map.data[idx] == 0)
                {
                    free_count++;

                    // 找到连续3个 free cell（表示足够往内退）
                    if (free_count >= 3)
                    {
                        best_nx = nx;
                        best_ny = ny;
                        found = true;
                        break;
                    }
                }
                else
                {
                    free_count = 0; // 一旦中断，重新计数
                }
            }

            if (found)
                break;
        }

        geometry_msgs::Point result;
        result.x = map.info.origin.position.x + best_nx * res;
        result.y = map.info.origin.position.y + best_ny * res;
        return result;
    }

    // 核心控制
    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        double x = goal->target_pose.pose.position.x;
        double y = goal->target_pose.pose.position.y;
        double yaw = tf::getYaw(goal->target_pose.pose.orientation);

        geometry_msgs::Point target_goal;
        target_goal.x = x;
        target_goal.y = y;
        target_goal.z = 0.0;

        geometry_msgs::Point frontier = findInnerFreePoint(current_map_, target_goal);
        x = frontier.x;
        y = frontier.y;

        ROS_INFO("Received goal: (%.2f, %.2f, %.2f)", x, y, yaw);
        if (current_state_ != Back)
        {
            if (mutiplePathClear(robot_x_map, robot_y_map, x, y))
            {
                current_state_ = Search;
            }
            else
            {
                current_state_ = Plan;
            }
        }

        if (current_state_ == Search)
        {
            ROS_INFO("State: SEARCH");
            std_msgs::String msg;
            msg.data = "SEARCH";
            state_pub_.publish(msg);

            if (mutiplePathClear(robot_x_map, robot_y_map, exit_x_, exit_y_))
            {
                ROS_WARN("Direct path to exit detected! Navigating directly to exit...");
                bool issucceed = MovetoGoal(exit_x_, exit_y_, exit_tolerance_);
                if (issucceed)
                {
                    char cmd[3];
                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0);

                    std_msgs::String replan;
                    replan.data = "Replan";
                    replan_pub.publish(replan);

                    geometry_msgs::Point goal_point_map;
                    goal_point_map.x = 0.0;
                    goal_point_map.y = 0.0;
                    goal_point_map.z = 0.0;
                    goal_pub.publish(goal_point_map);

                    geometry_msgs::Point robot_point_map;
                    robot_point_map.x = robot_x_map;
                    robot_point_map.y = robot_y_map;
                    robot_point_map.z = 0.0;
                    robot_pub.publish(robot_point_map);

                    ros::Duration(0.5).sleep();
                    current_state_ = Back;
                    std_msgs::String msg;
                    msg.data = "Back";
                    state_pub_.publish(msg);
                    ROS_INFO("Reached exit, switching to BACK state.");
                    as_.setSucceeded();
                    return;
                }
                else
                {
                    as_.setAborted();
                    return;
                }
            }

            bool issucceed = MovetoGoal(x, y, 0.3);
            if (issucceed)
            {
                char cmd[3];
                sprintf(cmd, "S00");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                as_.setSucceeded();
                ROS_INFO("Goal reached");
                if (checkExitReached())
                {

                    ROS_WARN("Exit reached! Stopping robot.");
                    current_state_ = Back;
                }
            }
            else
            {
                as_.setAborted();
                return;
            }
        }
        else if (current_state_ == Plan)
        {
            std_msgs::String replan;
            replan.data = "Replan";
            replan_pub.publish(replan);

            geometry_msgs::Point goal_point_map;
            goal_point_map.x = x;
            goal_point_map.y = y;
            goal_point_map.z = 0.0;
            goal_pub.publish(goal_point_map);

            geometry_msgs::Point robot_point_map;
            robot_point_map.x = robot_x_map;
            robot_point_map.y = robot_y_map;
            robot_point_map.z = 0.0;
            robot_pub.publish(robot_point_map);

            ros::Duration(0.5).sleep();
            ROS_INFO("State: Plan");
            std_msgs::String msg1;
            msg1.data = "Plan";
            state_pub_.publish(msg1);

            ros::Time start = ros::Time::now();
            ros::Duration timeout(3.0); // 最多等3秒
            while ((ros::Time::now() - start) < timeout)
            {
                ros::spinOnce(); // 让回调执行
                if (path_received)
                    break;
                ros::Duration(0.1).sleep(); // 防止占用CPU
            }

            ROS_WARN("path receive: %d", path_received);

            if (!path_received)
            {
                ROS_ERROR(" marking goal is unreachable. here");
                as_.setAborted();
                return;
            }

            if (!path_points.empty())
            {
                char cmd[3];
                for (auto &v : path_points)
                {
                    if (mutiplePathClear(robot_x_map, robot_y_map, exit_x_, exit_y_))
                    {
                        ROS_WARN("Direct path to exit detected! Navigating directly to exit...");
                        bool issucceed = MovetoGoal(exit_x_, exit_y_, exit_tolerance_);
                        if (issucceed)
                        {
                            char cmd[3];
                            sprintf(cmd, "S00");
                            send(sock_cmd, cmd, strlen(cmd), 0);

                            std_msgs::String replan;
                            replan.data = "Replan";
                            replan_pub.publish(replan);

                            geometry_msgs::Point goal_point_map;
                            goal_point_map.x = 0.0;
                            goal_point_map.y = 0.0;
                            goal_point_map.z = 0.0;
                            goal_pub.publish(goal_point_map);

                            geometry_msgs::Point robot_point_map;
                            robot_point_map.x = robot_x_map;
                            robot_point_map.y = robot_y_map;
                            robot_point_map.z = 0.0;
                            robot_pub.publish(robot_point_map);

                            ros::Duration(0.5).sleep();
                            current_state_ = Back;
                            std_msgs::String msg;
                            msg.data = "Back";
                            state_pub_.publish(msg);
                            ROS_INFO("Reached exit, switching to BACK state.");
                            as_.setSucceeded();
                            return;
                        }
                        else
                        {
                            as_.setAborted();
                            return;
                        }
                    }
                    bool success = MovetoGoal(v.first, v.second, 0.2);
                    if (success)
                    {
                        sprintf(cmd, "S00");
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        ROS_WARN("success");
                    }
                    else
                    {
                        sprintf(cmd, "S00");
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        ROS_WARN("fail to reach,go to next");
                    }
                }
                ROS_WARN("Path done");
                path_points.clear();
                path_received = false;
            }
            double dx = robot_x_map - x;
            double dy = robot_y_map - y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist < 0.2)
            {
                ROS_WARN("Reach goal");
                as_.setSucceeded();
            }
            else
            {
                ROS_WARN("Go to wrong way");
                as_.setAborted();
            }
        }
        else if (current_state_ == Back)
        {
            ros::Rate r(10);
            while (ros::ok())
            {
                ros::spinOnce();
                r.sleep();
            }
        }
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
    std::string action_name_;

    ros::Publisher state_pub_;      // 发布状态
    ros::Publisher robot_pub;       // 发布小车坐标
    ros::Publisher goal_pub;        // 发布目标坐标
    ros::Publisher replan_pub;      // 发送重新规划标志
    ros::Subscriber odom_sub_;      // 订阅位姿
    ros::Subscriber map_sub;        // 订阅地图
    ros::Subscriber path_sub_;      // 订阅路径
    ros::Subscriber finish_sub_;    // 订阅路径接收状态
    ros::Subscriber scan_sub;       // 订阅雷达
    ros::Subscriber local_cost_sub; // 订阅局部代价地图
    ros::Subscriber testmove;       // 接收Rviz的目标，用于测试

    double robot_x_ = 0.0; // 小车位姿坐标
    double robot_y_ = 0.0;
    double robot_x_map = 0; // 小车地图坐标
    double robot_y_map = 0;

    tf::TransformListener listener; // tf变换

    std::vector<std::pair<double, double>> path_points;
    bool path_received = false;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) // 位姿回调函数，获取小车当前位姿，转换到map系
    {
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;

        geometry_msgs::PointStamped robot_odom, robot_map;
        robot_odom.header.frame_id = "odom";
        robot_odom.header.stamp = ros::Time(0); // 最新可用 transform
        robot_odom.point.x = robot_x_;
        robot_odom.point.y = robot_y_;
        robot_odom.point.z = 0.0;

        try
        {
            listener.transformPoint("map", robot_odom, robot_map);
            robot_x_map = robot_map.point.x;
            robot_y_map = robot_map.point.y;
            ROS_DEBUG("Robot in map frame: x=%.2f y=%.2f", robot_x_map, robot_y_map);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN_THROTTLE(1.0, "Transform failed: %s", ex.what());
        }
    }

    double exit_x_, exit_y_; // 出口坐标，map系
    double exit_tolerance_;  // 判断出口到达条件
    bool checkExitReached()
    {
        double dx = robot_x_map - exit_x_;
        double dy = robot_y_map - exit_y_;
        double dist = sqrt(dx * dx + dy * dy);
        return dist < exit_tolerance_;
    }

    CarState current_state_; // 记录状态，SEARCH/BACK

    sensor_msgs::LaserScan latest_scan_; // 雷达消息包
    bool scan_received_ = false;

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) // 地图回调函数
    {
        current_map_ = *msg;
        map_received_ = true;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) // 雷达回调函数
    {
        latest_scan_ = *msg;
        scan_received_ = true;
    }

    void localmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        current_localmap = *msg;
        localmap_received = true;
    }

    void replanback()
    {
        std_msgs::String replan;
        replan.data = "Replan";
        replan_pub.publish(replan);

        geometry_msgs::Point goal_point_map;
        goal_point_map.x = 0.0;
        goal_point_map.y = 0.0;
        goal_point_map.z = 0.0;
        goal_pub.publish(goal_point_map);

        geometry_msgs::Point robot_point_map;
        robot_point_map.x = robot_x_map;
        robot_point_map.y = robot_y_map;
        robot_point_map.z = 0.0;
        robot_pub.publish(robot_point_map);

        ros::Duration(0.5).sleep();
        ROS_INFO("State: Back");
        std_msgs::String msg1;
        msg1.data = "Back";
        state_pub_.publish(msg1);
    }

    void testCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        double x = msg->pose.position.x;
        double y = msg->pose.position.y;
        double yaw = tf::getYaw(msg->pose.orientation);

        ROS_INFO("Received RViz 2D Nav Goal: x=%.2f y=%.2f yaw=%.2f", x, y, yaw);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = msg->pose;

        ROS_WARN("ischeck %d", checkFrontObstacle());

        executeCB(boost::make_shared<move_base_msgs::MoveBaseGoal>(goal));
    }

    void pathPointCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        if (current_state_ != Back)
        {
            if (msg->poses.size() != 0)
            {
                for (int i = 0; i < msg->poses.size() - 1; i++)
                {
                    path_points.push_back({msg->poses[i + 1].pose.position.x, msg->poses[i + 1].pose.position.y});
                }
            }
        }
        if (current_state_ == Back)
        {

            std::vector<std::pair<double, double>> path_back;
            if (msg->poses.size() != 0)
            {
                for (int i = 0; i < msg->poses.size() - 1; i++)
                {
                    path_back.push_back({msg->poses[i + 1].pose.position.x, msg->poses[i + 1].pose.position.y});
                }
            }
            char cmd[3];
            int wrongtime = 0;
            for (auto &v : path_back)
            {
                if (wrongtime > 5)
                {
                    ROS_WARN("stop this path");

                    sprintf(cmd, "S00");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    replanback();
                    return;
                }
                if (mutiplePathClear(robot_x_map, robot_y_map, 0, 0))
                {
                    ROS_WARN("Direct path to exit detected! Navigating directly to exit...");
                    bool issucceed = MovetoGoal(0, 0, 0.5);
                    if (issucceed)
                    {
                        ROS_WARN("Return");
                        sprintf(cmd, "S00");
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        return;
                    }
                    else
                    {
                        sprintf(cmd, "S00");
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        replanback();
                        return;
                    }
                }
                bool success = MovetoGoal(v.first, v.second, 0.15);
                if (success)
                {
                    sprintf(cmd, "S00");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    ROS_WARN("success");
                    wrongtime = 0;
                }
                else
                {
                    sprintf(cmd, "S00");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    ROS_WARN("fail to reach,go to next");
                    wrongtime++;
                }
            }
            ROS_WARN("Path done");
            double dis = sqrt(robot_x_map * robot_x_map + robot_y_map * robot_y_map);
            if (dis < 0.5)
            {
                ROS_WARN("Return");
                sprintf(cmd, "S00");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
            }
            else
            {
                sprintf(cmd, "S00");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                replanback();
            }
        }
    }

    void pathFinishCallback(const std_msgs::String &msg)
    {
        if (msg.data == "PATH_DONE")
        {
            path_received = true;
        }
        else if (msg.data == "NO_PATH")
        {
            path_received = false;
        }
    }
};

int main(int argc, char **argv)
{
    if (!initTCPServer("0.0.0.0", 9102))
        return -1;
    ros::init(argc, argv, "fake_movebase_new");
    FakeMoveBase server("move_base");
    ros::spin();

    if (sock_cmd >= 0)
        close(sock_cmd);
    if (server_sock >= 0)
        close(server_sock);

    return 0;
}