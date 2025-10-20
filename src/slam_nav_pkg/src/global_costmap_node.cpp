#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <cmath>

class SimpleCostmapNode
{
public:
    SimpleCostmapNode()
    {
        ros::NodeHandle nh("~");
        
        // 参数
        nh.param("inflation_radius_m", inflation_radius_m_, 0.2); // 膨胀半径（米）
        nh.param("map_topic", map_topic_, std::string("/map"));
        nh.param("costmap_topic", costmap_topic_, std::string("/global_costmap"));
        nh.param("threshold", threshold_, 50); // map 中障碍阈值
        nh.param("max_cost", max_cost_, 100);  // 最大代价值
        nh.param("min_cost", min_cost_, 1);    // 最小代价值

        // 订阅 map
        map_sub_ = nh.subscribe(map_topic_, 1, &SimpleCostmapNode::mapCallback, this);

        // 发布 costmap
        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(costmap_topic_, 1, true);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        nav_msgs::OccupancyGrid costmap;
        costmap.header = map_msg->header;
        costmap.info = map_msg->info;
        costmap.data.resize(map_msg->data.size());

        int width = map_msg->info.width;
        int height = map_msg->info.height;
        double res = map_msg->info.resolution;

        int inflation_radius_cells = std::ceil(inflation_radius_m_ / res);

        // 先找障碍物
        std::vector<int> obstacle_indices;
        for (size_t i = 0; i < map_msg->data.size(); ++i)
        {
            int val = map_msg->data[i];
            if (val >= threshold_)
            {
                costmap.data[i] = max_cost_; // 原始障碍
                obstacle_indices.push_back(i);
            }
            else if (val == -1)
            {
                costmap.data[i] = -1;
            }
            else
            {
                costmap.data[i] = 0; // free
            }
        }

        // 膨胀（距离递减）
        for (int idx : obstacle_indices)
        {
            int x = idx % width;
            int y = idx / width;

            for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; ++dx)
            {
                for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; ++dy)
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx >= 0 && nx < width && ny >= 0 && ny < height)
                    {
                        int nidx = ny * width + nx;
                        if (costmap.data[nidx] == -1)
                            continue;
                        double dist = std::sqrt(dx * dx + dy * dy) * res;
                        if (dist <= inflation_radius_m_)
                        {
                            int cost = static_cast<int>(
                                max_cost_ * (1.0 - dist / inflation_radius_m_));
                            if (cost < min_cost_)
                                cost = min_cost_;
                            // 取最大的代价值，避免被覆盖
                            if (cost > costmap.data[nidx])
                                costmap.data[nidx] = cost;
                        }
                    }
                }
            }
        }

        costmap_pub_.publish(costmap);
    }

private:
    ros::Subscriber map_sub_;
    ros::Publisher costmap_pub_;

    double inflation_radius_m_;
    int threshold_;
    int max_cost_;
    int min_cost_;
    std::string map_topic_;
    std::string costmap_topic_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_costmap_node");
    SimpleCostmapNode node;
    ros::spin();
    return 0;
}
