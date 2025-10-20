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

int sock_cmd = -1;    // 已连接客户端
int server_sock = -1; // 服务端监听 socket
bool map_received_ = false;
nav_msgs::OccupancyGrid current_map_;
std::queue<geometry_msgs::Point> path_queue_;
bool path_complete_ = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    current_map_ = *msg;
    map_received_ = true;
}

typedef enum
{
    Search,
    Back
} CarState;

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
        int index = y * width + x; // 或试试 (height - 1 - y) * width + x
        if (index >= 0 && index < (int)current_map_.data.size())
        {
            int occ = current_map_.data[index];
            if (occ > 50 || occ < 0)
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

class FakeMoveBase
{

public:
    FakeMoveBase(std::string name) : as_(nh_, name, boost::bind(&FakeMoveBase::executeCB, this, _1), false),
                                     action_name_(name)
    {
        as_.start();
        ROS_INFO("FakeMoveBase server started");
        odom_sub_ = nh_.subscribe("/odom", 10, &FakeMoveBase::odomCallback, this);
        exit_x_ = 2.0;
        exit_y_ = -2.0;
        exit_tolerance_ = 0.5;
        state_pub_ = nh_.advertise<std_msgs::String>("/robot_state", 10);
        current_state_ = Search;
        map_sub = nh_.subscribe("/map", 10, mapCallback);
        path_sub_ = nh_.subscribe("/planned_path_point", 10, &FakeMoveBase::pathPointCallback, this);
        finish_sub_ = nh_.subscribe("/planned_path_done", 10, &FakeMoveBase::pathFinishCallback, this);
        scan_sub = nh_.subscribe("/scan", 10, &FakeMoveBase::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
    {
        latest_scan_ = *msg;
        scan_received_ = true;
    }

    void pathPointCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        path_queue_.push(*msg);
    }

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
            return true;
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Transform failed: %s", ex.what());
            return false;
        }
    }

    void pathFinishCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "PATH_DONE")
        {
            ros::Rate rate(50);
            double dx, dy = 0;
            while (!path_queue_.empty() && ros::ok())
            {
                geometry_msgs::Point pt = path_queue_.front();
                path_queue_.pop();
                mapGoalToRobotFrame(pt, dx, dy);
                double distance = sqrt(dx * dx + dy * dy);
                double angle_rad = atan2(dy, dx);
                double angle_degree = angle_rad / M_PI * 180;
                char cmd[3];

                if (angle_degree > 5)
                {
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
                        sleep(1.5);
                        sprintf(cmd, "L%02.0f", angle_degree - 90);
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        sleep(1.5);
                    }
                }
                else if (angle_degree < -5)
                {
                    if (angle_degree > -90)
                    {
                        sprintf(cmd, "R%02.0f", -angle_degree);
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                        sleep(1.5);
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
                        sleep(1.5);
                    }
                }

                if (distance > 0.25)
                {
                    sprintf(cmd, "F00");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                }
                double time = distance / 0.4;
                sleep(time);
                sprintf(cmd, "S00");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
                rate.sleep();
            }
        }
    }

    bool checkFrontObstacle()
    {
        if (!scan_received_)
            return false;

        std::vector<float> lidar_scan;

        for (int i = 160; i < 200; i++)
        {
            lidar_scan.push_back(latest_scan_.ranges[i]);
        }

        for (auto &v : lidar_scan)
        {
            if (v < 0.28)
            {
                ROS_WARN("Obstacle detected! %.2f", v);
                return true;
            }
        }

        return false;
    }

    void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal)
    {
        if (current_state_ == Search)
        {
            ROS_INFO("State: SEARCH");
            std_msgs::String msg;
            msg.data = "SEARCH";
            state_pub_.publish(msg);

            double x = goal->target_pose.pose.position.x;
            double y = goal->target_pose.pose.position.y;
            double yaw = tf::getYaw(goal->target_pose.pose.orientation);

            ROS_INFO("Received goal: (%.2f, %.2f, %.2f)", x, y, yaw);

            double dx, dy;
            geometry_msgs::Point goal_point_map;
            goal_point_map.x = x;
            goal_point_map.y = y;
            goal_point_map.z = 0.0;

            if (!mapGoalToRobotFrame(goal_point_map, dx, dy))
            {
                ROS_WARN("Failed to transform goal to robot frame, aborting");
                as_.setAborted();
                return;
            }
            ROS_INFO("Goal in robot frame: dx=%.2f, dy=%.2f", dx, dy);

            double distance = sqrt(dx * dx + dy * dy);
            dx-=dx/distance*0.35;
            dy-=dy/distance*0.35;
            double angle_rad = atan2(dy, dx);
            double angle_degree = angle_rad / M_PI * 180;
            char cmd[3];

            if (angle_degree > 5)
            {
                if (angle_degree < 90)
                {
                    sprintf(cmd, "L%02.0f", angle_degree);
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                }
                else
                {
                    sprintf(cmd, "L90");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                    sprintf(cmd, "L%02.0f", angle_degree - 90);
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                }
            }
            else if (angle_degree < -5)
            {
                if (angle_degree > -90)
                {
                    sprintf(cmd, "R%02.0f", -angle_degree);
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                }
                else
                {
                    sprintf(cmd, "R90");
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                    sprintf(cmd, "R%02.0f", (-angle_degree) - 90);
                    ROS_INFO(cmd);
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1.5);
                }
            }

            if (distance > 0.35)
            {
                sprintf(cmd, "F00");
                ROS_INFO(cmd);
                send(sock_cmd, cmd, strlen(cmd), 0);
            }
            ros::Rate rate(10);
            while (distance > 0.35 && ros::ok())
            {
                /* double dx_r = robot_x_ - begin_x;
                double dy_r = robot_y_ - begin_y;
                distance = sqrt((dx - dx_r) * (dx - dx_r) + (dy - dy_r) * (dy - dy_r)); */
                mapGoalToRobotFrame(goal_point_map, dx, dy);
                distance = sqrt(dx * dx + dy * dy);

                if (checkFrontObstacle())
                {
                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sprintf(cmd, "B00");
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    sleep(1);
                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    ROS_WARN("Emergency stop due to obstacle!");
                    as_.setAborted();
                    return;
                }

                if (isPathClear(robot_x_map, robot_y_map, exit_x_, exit_y_))
                {
                    ROS_WARN("Direct path to exit detected! Navigating directly to exit...");
                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0); // 停止当前前沿动作
                    sleep(1);

                    // 计算朝向出口的角度
                    geometry_msgs::Point exit_point_map;
                    exit_point_map.x = exit_x_;
                    exit_point_map.y = exit_y_;
                    exit_point_map.z = 0.0;
                    double dx_e, dy_e;
                    mapGoalToRobotFrame(goal_point_map, dx_e, dy_e);
                    double angle_rad_exit = atan2(dy_e, dx_e);
                    double angle_degree_exit = angle_rad_exit / M_PI * 180;
                    double distance_to_exit = sqrt(dx_e * dx_e + dy_e * dy_e);
                    char cmd_exit[3];
                    if (angle_degree_exit > 5)
                    {
                        if (angle_degree_exit < 90)
                        {
                            sprintf(cmd, "L%02.0f", angle_degree_exit);
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                        }
                        else
                        {
                            sprintf(cmd, "L90");
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                            sprintf(cmd, "L%02.0f", angle_degree_exit - 90);
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                        }
                    }
                    else if (angle_degree_exit < -5)
                    {
                        if (angle_degree_exit > -90)
                        {
                            sprintf(cmd, "R%02.0f", -angle_degree_exit);
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                        }
                        else
                        {
                            sprintf(cmd, "R90");
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                            sprintf(cmd, "R%02.0f", (-angle_degree_exit) - 90);
                            ROS_INFO(cmd);
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1.5);
                        }
                    }

                    // 启动前进到出口
                    if (distance_to_exit > 0.35)
                    {
                        sprintf(cmd, "F00");
                        ROS_INFO(cmd);
                        send(sock_cmd, cmd, strlen(cmd), 0);
                    }

                    // 等待到达出口
                    while (!checkExitReached() && ros::ok())
                    {

                        if (checkFrontObstacle())
                        {
                            sprintf(cmd, "S00");
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sprintf(cmd, "B00");
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            sleep(1);
                            sprintf(cmd, "S00");
                            send(sock_cmd, cmd, strlen(cmd), 0);
                            ROS_WARN("Emergency stop due to obstacle!");
                            as_.setAborted();
                            return;
                        }

                        ros::spinOnce();
                        rate.sleep();
                    }

                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    current_state_ = Back;
                    std_msgs::String msg;
                    msg.data = "Back";
                    state_pub_.publish(msg);
                    ROS_INFO("Reached exit, switching to BACK state.");
                    as_.setSucceeded();
                    return;
                }

                if (checkExitReached())
                {
                    ROS_WARN("Exit reached! Stopping robot.");
                    sprintf(cmd, "S00");
                    send(sock_cmd, cmd, strlen(cmd), 0);
                    current_state_ = Back;
                    break;
                }

                ros::spinOnce();
                rate.sleep();
            }
            sprintf(cmd, "S00");
            ROS_INFO(cmd);
            send(sock_cmd, cmd, strlen(cmd), 0);

            as_.setSucceeded();
            ROS_INFO("Goal reached");
        }
        else if (current_state_ == Back)
        {
            ROS_INFO("State: Back");
            std_msgs::String msg;
            msg.data = "Back";
            state_pub_.publish(msg);
        }
    }

private:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;
    std::string action_name_;
    ros::Subscriber odom_sub_;
    ros::Subscriber map_sub;
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_x_map = 0;
    double robot_y_map = 0;
    double begin_x = 0;
    double begin_y = 0;
    tf::TransformListener listener;

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
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

    double exit_x_, exit_y_;
    double exit_tolerance_;
    bool checkExitReached()
    {
        double dx = robot_x_map - exit_x_;
        double dy = robot_y_map - exit_y_;
        double dist = sqrt(dx * dx + dy * dy);
        return dist < exit_tolerance_;
    }

    CarState current_state_;
    ros::Publisher state_pub_;
    ros::Subscriber path_sub_;
    ros::Subscriber finish_sub_;
    ros::Subscriber scan_sub;

    sensor_msgs::LaserScan latest_scan_;
    bool scan_received_ = false;
};

int main(int argc, char **argv)
{
    if (!initTCPServer("0.0.0.0", 9102))
        return -1;
    ros::init(argc, argv, "fake_move_base");
    FakeMoveBase server("move_base");
    ros::spin();

    if (sock_cmd >= 0)
        close(sock_cmd);
    if (server_sock >= 0)
        close(server_sock);

    return 0;
}
