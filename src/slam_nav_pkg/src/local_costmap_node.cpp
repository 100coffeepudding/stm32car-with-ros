#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <cmath>

class LocalCostmapNode
{
public:
    LocalCostmapNode()
    {
        ros::NodeHandle nh("~");

        nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
        nh.param("map_frame", map_frame_, std::string("odom")); // 局部costmap参考frame
        nh.param("radius_m", radius_m_, 2.0);                  // 局部代价地图半径
        nh.param("resolution", resolution_, 0.05);            // 格子分辨率
        nh.param("inflation_radius", inflation_radius_, 0.2); // 膨胀半径
        nh.param("max_cost", max_cost_, 100);
        nh.param("min_cost", min_cost_, 1);

        costmap_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("local_costmap", 1, true);
        scan_sub_ = nh.subscribe("/scan", 1, &LocalCostmapNode::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        tf::StampedTransform transform;
        try {
            listener_.lookupTransform(map_frame_, robot_base_frame_, ros::Time(0), transform);
        } catch(tf::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "%s", ex.what());
            return;
        }

        // 局部 costmap 初始化
        int width = static_cast<int>(2*radius_m_ / resolution_);
        int height = width;
        nav_msgs::OccupancyGrid costmap;
        costmap.header.stamp = ros::Time::now();
        costmap.header.frame_id = map_frame_;
        costmap.info.resolution = resolution_;
        costmap.info.width = width;
        costmap.info.height = height;
        costmap.info.origin.position.x = transform.getOrigin().x() - radius_m_;
        costmap.info.origin.position.y = transform.getOrigin().y() - radius_m_;
        costmap.info.origin.position.z = 0.0;
        costmap.info.origin.orientation.w = 1.0;
        costmap.data.resize(width*height, 0);

        // 将激光点投影到局部地图
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float r = scan->ranges[i];
            if (r < scan->range_min || r > scan->range_max) continue;

            float angle = scan->angle_min + i * scan->angle_increment;
            float lx = r * cos(angle);
            float ly = r * sin(angle);

            // 转换到 map_frame
            double gx = transform.getOrigin().x() + lx * transform.getBasis().getColumn(0).x() +
                        ly * transform.getBasis().getColumn(1).x();
            double gy = transform.getOrigin().y() + lx * transform.getBasis().getColumn(0).y() +
                        ly * transform.getBasis().getColumn(1).y();

            int mx = static_cast<int>((gx - costmap.info.origin.position.x) / resolution_);
            int my = static_cast<int>((gy - costmap.info.origin.position.y) / resolution_);

            if (mx >=0 && mx < width && my >=0 && my < height)
                costmap.data[my*width + mx] = max_cost_;
        }

        // 简单膨胀
        int inflation_cells = static_cast<int>(inflation_radius_ / resolution_);
        nav_msgs::OccupancyGrid inflated = costmap;
        for (int y=0; y<height; ++y)
        {
            for (int x=0; x<width; ++x)
            {
                int idx = y*width + x;
                if (costmap.data[idx] < max_cost_) continue;

                for (int dy=-inflation_cells; dy<=inflation_cells; ++dy)
                {
                    for (int dx=-inflation_cells; dx<=inflation_cells; ++dx)
                    {
                        int nx = x+dx;
                        int ny = y+dy;
                        if (nx<0 || nx>=width || ny<0 || ny>=height) continue;
                        double dist = std::sqrt(dx*dx + dy*dy)*resolution_;
                        if (dist <= inflation_radius_)
                        {
                            int nidx = ny*width + nx;
                            int cost = static_cast<int>(max_cost_*(1.0 - dist/inflation_radius_));
                            if (cost < min_cost_) cost = min_cost_;
                            if (cost > inflated.data[nidx]) inflated.data[nidx] = cost;
                        }
                    }
                }
            }
        }

        costmap_pub_.publish(inflated);
    }

private:
    ros::Publisher costmap_pub_;
    ros::Subscriber scan_sub_;
    tf::TransformListener listener_;

    std::string robot_base_frame_;
    std::string map_frame_;
    double radius_m_;
    double resolution_;
    double inflation_radius_;
    int max_cost_;
    int min_cost_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_costmap_node");
    LocalCostmapNode node;
    ros::spin();
    return 0;
}
