#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <vector>
#include <cmath>

struct Node
{
    int x, y;
    double g, h;
    Node *parent;
    Node(int x_, int y_, double g_ = 0, double h_ = 0, Node *p = nullptr)
        : x(x_), y(y_), g(g_), h(h_), parent(p) {}
    double f() const { return g + h; }
};

struct CompareNode
{
    bool operator()(Node *a, Node *b)
    {
        return a->f() > b->f();
    }
};

class GlobalPlanner
{
public:
    GlobalPlanner()
    {
        // 订阅全局代价地图
        map_sub_ = nh_.subscribe("/global_costmap", 1, &GlobalPlanner::mapCallback, this);

        // 接收小车坐标、目标坐标、状态
        robot_sub_ = nh_.subscribe("/robot_xy", 10, &GlobalPlanner::robotCallback, this);
        goal_sub_ = nh_.subscribe("/goal_xy", 10, &GlobalPlanner::goalCallback, this);
        state_sub_ = nh_.subscribe("/robot_state", 10, &GlobalPlanner::stateCallback, this);
        replan_sub = nh_.subscribe("/replan", 10, &GlobalPlanner::replanCallback, this);

        // 发布 RViz 可显示路径
        path_pub_ = nh_.advertise<nav_msgs::Path>("/planned_path", 1);

        // 发布路径规划完成标志
        finish_pub_ = nh_.advertise<std_msgs::String>("/planned_path_done", 10);

        has_map_ = false;
        has_robot_ = false;
        has_goal_ = false;
        replan = false;

        ROS_INFO("Global Planner initialized.");
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
    {
        map_ = *msg;
        has_map_ = true;
    }

    void robotCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        robot_x_ = msg->x;
        robot_y_ = msg->y;
        has_robot_ = true;
    }

    void goalCallback(const geometry_msgs::Point::ConstPtr &msg)
    {
        goal_x_ = msg->x;
        goal_y_ = msg->y;
        has_goal_ = true;
    }

    void replanCallback(const std_msgs::String &msg)
    {
        if (msg.data == "Replan")
            replan = true;
    }

    void stateCallback(const std_msgs::String::ConstPtr &msg)
    {
        if ((msg->data == "Plan"||msg->data == "Back") && has_map_ && has_robot_ && has_goal_ && replan)
        {
            ROS_WARN("Enter PLAN state -> start global planning...");
            planGlobalPath();
            replan = false;
        }
    }

    std::vector<geometry_msgs::PoseStamped> simplifyByTurning(
        const std::vector<geometry_msgs::PoseStamped> &path_points)
    {
        if (path_points.size() < 3)
            return path_points;

        std::vector<geometry_msgs::PoseStamped> simplified;
        simplified.push_back(path_points.front());

        for (size_t i = 1; i < path_points.size() - 1; ++i)
        {
            double dx1 = path_points[i].pose.position.x - path_points[i - 1].pose.position.x;
            double dy1 = path_points[i].pose.position.y - path_points[i - 1].pose.position.y;
            double dx2 = path_points[i + 1].pose.position.x - path_points[i].pose.position.x;
            double dy2 = path_points[i + 1].pose.position.y - path_points[i].pose.position.y;

            double dot = dx1 * dx2 + dy1 * dy2;
            double norm1 = sqrt(dx1 * dx1 + dy1 * dy1);
            double norm2 = sqrt(dx2 * dx2 + dy2 * dy2);
            double cos_angle = dot / (norm1 * norm2 + 1e-6);

            // 当方向变化超过阈值（比如15°）时保留点
            if (cos_angle < cos(15.0 * M_PI / 180.0))
                simplified.push_back(path_points[i]);
        }

        simplified.push_back(path_points.back());
        return simplified;
    }

    std::vector<geometry_msgs::PoseStamped> simplifyByDistance(
        const std::vector<geometry_msgs::PoseStamped> &path_points, double min_dist)
    {
        std::vector<geometry_msgs::PoseStamped> simplified;
        if (path_points.empty())
            return simplified;

        simplified.push_back(path_points.front());
        geometry_msgs::PoseStamped last = path_points.front();

        for (auto &p : path_points)
        {
            double dx = p.pose.position.x - last.pose.position.x;
            double dy = p.pose.position.y - last.pose.position.y;
            double dist = sqrt(dx * dx + dy * dy);
            if (dist >= min_dist)
            {
                simplified.push_back(p);
                last = p;
            }
        }
        return simplified;
    }
    void planGlobalPath()
    {
        double res = map_.info.resolution;
        double origin_x = map_.info.origin.position.x;
        double origin_y = map_.info.origin.position.y;
        int width = map_.info.width;
        int height = map_.info.height;

        // 坐标 -> 栅格索引
        auto worldToMap = [&](double wx, double wy, int &mx, int &my)
        {
            mx = (wx - origin_x) / res;
            my = (wy - origin_y) / res;
        };

        int sx, sy, gx, gy;
        worldToMap(robot_x_, robot_y_, sx, sy);
        worldToMap(goal_x_, goal_y_, gx, gy);

        ROS_WARN("Start position (meters): (%.3f, %.3f)  Goal position (meters): (%.3f, %.3f)",
                 robot_x_, robot_y_, goal_x_, goal_y_);

        std::priority_queue<Node *, std::vector<Node *>, CompareNode> open_list;
        std::vector<std::vector<bool>> closed(width, std::vector<bool>(height, false));

        Node *start_node = new Node(sx, sy, 0, heuristic(sx, sy, gx, gy));
        open_list.push(start_node);
        Node *final_node = nullptr;

        int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        while (!open_list.empty())
        {
            Node *current = open_list.top();
            open_list.pop();

            if (current->x == gx && current->y == gy)
            {
                final_node = current;
                break;
            }

            if (current->x < 0 || current->y < 0 || current->x >= width || current->y >= height)
                continue;

            if (closed[current->x][current->y])
                continue;

            closed[current->x][current->y] = true;

            for (int i = 0; i < 8; i++)
            {
                int nx = current->x + dx[i];
                int ny = current->y + dy[i];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                    continue;

                int idx = ny * width + nx;
                if (map_.data[idx] > 35)
                    continue;
                if (!closed[nx][ny])
                {
                    double cost = (i < 4) ? 1.0 : 1.414;
                    Node *neighbor = new Node(nx, ny, current->g + cost, heuristic(nx, ny, gx, gy), current);
                    open_list.push(neighbor);
                }
            }
        }

        if (final_node)
        {
            nav_msgs::Path path_msg;
            path_msg.header.frame_id = "map";
            path_msg.header.stamp = ros::Time::now();

            std::vector<geometry_msgs::PoseStamped> path_points;
            Node *n = final_node;
            while (n)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = n->x * res + origin_x + res / 2.0;
                pose.pose.position.y = n->y * res + origin_y + res / 2.0;
                pose.pose.orientation.w = 1.0;
                path_points.push_back(pose);
                n = n->parent;
            }

            std::reverse(path_points.begin(), path_points.end());
            path_points = simplifyByTurning(path_points);
            path_points = simplifyByDistance(path_points, 0.2);

            path_msg.poses = path_points;
            path_pub_.publish(path_msg);

            ROS_INFO("Path found with %zu points.", path_points.size());

            std_msgs::String done;
            done.data = "PATH_DONE";
            finish_pub_.publish(done);
        }
        else
        {
            ROS_WARN("No path found!");
            std_msgs::String wrong;
            wrong.data = "NO_PATH";
            finish_pub_.publish(wrong);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber robot_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber state_sub_;
    ros::Subscriber replan_sub;
    ros::Publisher path_pub_;
    ros::Publisher finish_pub_;

    nav_msgs::OccupancyGrid map_;
    bool has_map_, has_robot_, has_goal_, flag_send_, replan;
    double robot_x_, robot_y_, goal_x_, goal_y_;

    double heuristic(int x1, int y1, int x2, int y2)
    {
        return std::hypot(x1 - x2, y1 - y2);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planner");
    GlobalPlanner gp;
    ros::spin();
    return 0;
}
