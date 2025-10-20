#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <iostream>
#include <angles/angles.h>

#define LIDAR_PORT 9000
#define ODOM_PORT 9001

std::mutex odom_mutex;
float current_odom[3] = {0};
ros::Publisher scan_pub;
ros::Publisher odom_pub;
int sock_cmd = -1;
tf::TransformBroadcaster *g_br = nullptr;

bool setupServer(int port, int &server_sock, int &client_sock)
{
    server_sock = socket(AF_INET, SOCK_STREAM, 0);
    if (server_sock < 0)
    {
        perror("socket");
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(server_sock, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        perror("bind");
        return false;
    }

    listen(server_sock, 1);
    ROS_INFO("Waiting for connection on port %d...", port);

    socklen_t len = sizeof(addr);
    client_sock = accept(server_sock, (sockaddr *)&addr, &len);
    if (client_sock < 0)
    {
        perror("accept");
        return false;
    }

    ROS_INFO("Client connected on port %d", port);
    return true;
}

// --- 线程1：接收雷达 ---
void lidarThread()
{
    int server_sock, client_sock;
    if (!setupServer(LIDAR_PORT, server_sock, client_sock))
        return;

    float buf[360];
    

    while (ros::ok())
    {
        int bytes = recv(client_sock, buf, sizeof(buf), MSG_WAITALL);
        if (bytes != sizeof(buf))
        {
            ROS_WARN("Lidar recv size mismatch: %d", bytes);
            continue;
        }

        ros::Time now = ros::Time::now();

        sensor_msgs::LaserScan scan;
        scan.header.stamp = now;
        scan.header.frame_id = "laser_link";
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = (2 * M_PI / 360);
        scan.range_min = 0.0;
        scan.range_max = 8.0;
        scan.ranges.resize(360);

        for (int i = 0; i < 360; ++i)
        {
            int j = (i + 180) % 360;       // 平移180°
            scan.ranges[i] = buf[359 - j]; // 再反转顺序
        }

        scan_pub.publish(scan);

        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "base_link";
        tf_msg.child_frame_id = "laser_link";
        tf_msg.transform.translation.x = -0.15; 
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.10;                           // 高度
        tf_msg.transform.rotation = tf::createQuaternionMsgFromYaw(0.0); // 没旋转
        g_br->sendTransform(tf_msg);
    }

    close(client_sock);
    close(server_sock);
}

// --- 线程2：接收odom并广播TF ---

void odomThread()
{
    int server_sock, client_sock;
    if (!setupServer(ODOM_PORT, server_sock, client_sock))
        return;

    float buf[3];
    nav_msgs::Odometry last_odom_msg;
    ros::Time last_time = ros::Time(0);
    bool have_last = false;

    while (ros::ok())
    {
        int bytes = recv(client_sock, buf, sizeof(buf), MSG_WAITALL);
        if (bytes != sizeof(buf))
        {
            ROS_WARN("Odom recv size mismatch: %d", bytes);
            continue;
        }

        // 统一时间戳
        ros::Time now = ros::Time::now();

        // 保存最新里程计（原始）
        {
            std::lock_guard<std::mutex> lock(odom_mutex);
            memcpy(current_odom, buf, sizeof(buf));
        }

        // 计算速度（基于上一条）
        double vx = 0.0, vth = 0.0;
        if (have_last)
        {
            double dt = (now - last_time).toSec();
            if (dt > 0.0001)
            {
                vx = (buf[0] - last_odom_msg.pose.pose.position.x) / dt;
                double last_yaw = tf::getYaw(last_odom_msg.pose.pose.orientation);
                double yaw = buf[2];
                // 归一化差值到 [-pi, pi]
                double dyaw = angles::shortest_angular_distance(last_yaw, yaw);
                vth = dyaw / dt;
            }
        }
        last_time = now;
        have_last = true;

        // 发布里程计
        nav_msgs::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = buf[0];
        odom.pose.pose.position.y = buf[1];
        odom.pose.pose.position.z = 0.0;
        geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(buf[2]);
        odom.pose.pose.orientation = q;

        // 填写 twist
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = vth;

        // 可选：设置协方差（不要全部为0）
        for (int i = 0; i < 36; i++)
            odom.pose.covariance[i] = 0.0;
        odom.pose.covariance[0] = 0.01;  // x
        odom.pose.covariance[7] = 0.01;  // y
        odom.pose.covariance[35] = 0.05; // yaw

        odom_pub.publish(odom);
        last_odom_msg = odom;

        // 发布 TF 使用相同的 now
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = now;
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = buf[0];
        tf_msg.transform.translation.y = buf[1];
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation = q;
        g_br->sendTransform(tf_msg);
    }

    close(client_sock);
    close(server_sock);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcp_receive_node");
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    g_br = &br;

    scan_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 10);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    std::thread t1(lidarThread);
    std::thread t2(odomThread);

    ros::spin();

    t1.join();
    t2.join();
    return 0;
}
