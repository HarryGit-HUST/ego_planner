#include "grid_map.h"
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <cmath>
#include <algorithm>
#include <iostream>

GridMap::GridMap() {}
GridMap::~GridMap() {}

// ============================================================================
// 地图初始化：设置参数、构建静态墙、订阅点云、启动衰减定时器
// ============================================================================
void GridMap::init(ros::NodeHandle &nh)
{
    // 读取参数
    nh.param("planner/map_resolution", param_.resolution, 0.1);
    nh.param("planner/map_width_m", param_.width_m, 20.0);
    nh.param("planner/map_height_m", param_.height_m, 20.0);
    nh.param("planner/map_origin_x", param_.origin_x, -5.0);
    nh.param("planner/map_origin_y", param_.origin_y, -10.0);
    nh.param("planner/map_decay_rate", param_.decay_rate, 5);
    nh.param("planner/map_front_x", param_.front_x, 5.3f);
    nh.param("planner/map_back_x", param_.back_x, -0.5f);
    nh.param("planner/map_left_y", param_.left_y, 1.0f);
    nh.param("planner/map_right_y", param_.right_y, -7.5f);
    nh.param("planner/map_exp", param_.exp, 0.2f);  
    nh.param("planner/map_intensity_threshold", param_.intensity_threshold, 10.0f);

    grid_w_ = std::ceil(param_.width_m / param_.resolution);
    grid_h_ = std::ceil(param_.height_m / param_.resolution);
    occupancy_buffer_.assign(grid_w_ * grid_h_, 0);

    //buildStaticWalls(start_x, start_y);

    // 订阅 2D ROI 点云
    cloud_sub_ = nh.subscribe("/projected_accumulated_cloud", 1, &GridMap::cloudCallback, this);

    // 在 init 函数的末尾，加上这一句发布器声明：
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("/viz/grid_map", 1, true);

    ROS_INFO("[GridMap] 测绘部初始化完成! 地图尺寸: %d x %d", grid_w_, grid_h_);
}

bool GridMap::posToIndex(const Eigen::Vector2d &pos, int &gx, int &gy) const
{
    gx = (int)((pos.x() - param_.origin_x) / param_.resolution);
    gy = (int)((pos.y() - param_.origin_y) / param_.resolution);
    return (gx >= 0 && gx < grid_w_ && gy >= 0 && gy < grid_h_);
}

void GridMap::indexToPos(int gx, int gy, Eigen::Vector2d &pos) const
{
    pos.x() = param_.origin_x + (gx + 0.5) * param_.resolution;
    pos.y() = param_.origin_y + (gy + 0.5) * param_.resolution;
}

bool GridMap::isOccupied(int gx, int gy) const
{
    if (gx < 0 || gx >= grid_w_ || gy < 0 || gy >= grid_h_)
        return true; // 越界视为撞墙
    return occupancy_buffer_[gx + gy * grid_w_] > 50;
}

bool GridMap::isOccupied(const Eigen::Vector2d &pos) const
{
    int gx, gy;
    if (!posToIndex(pos, gx, gy))
        return true;
    return isOccupied(gx, gy);
}

void GridMap::clearMap()
{
    std::fill(occupancy_buffer_.begin(), occupancy_buffer_.end(), 0);
}

void GridMap::buildStaticWalls(double start_x, double start_y)
{
    // [修复] 强行绑定真实的起飞原点，防止里程计漂移导致墙体错位！
    float f_x = start_x + param_.front_x;
    float b_x = start_x + param_.back_x;
    float l_y = start_y + param_.left_y;
    float r_y = start_y + param_.right_y;

    std::vector<std::vector<float>> walls = {
        {f_x - param_.exp, f_x + param_.exp, r_y, l_y}, // 前墙
        {b_x - param_.exp, b_x + param_.exp, r_y, l_y}, // 后墙
        {b_x, f_x, l_y - param_.exp, l_y + param_.exp}, // 左墙
        {b_x, f_x, r_y - param_.exp, r_y + param_.exp}  // 右墙
    };

    for (const auto &w : walls)
    {
        int min_gx, min_gy, max_gx, max_gy;
        posToIndex(Eigen::Vector2d(w[0], w[2]), min_gx, min_gy);
        posToIndex(Eigen::Vector2d(w[1], w[3]), max_gx, max_gy);

        min_gx = std::max(0, min_gx);
        min_gy = std::max(0, min_gy);
        max_gx = std::min(grid_w_ - 1, max_gx);
        max_gy = std::min(grid_h_ - 1, max_gy);

        for (int x = min_gx; x <= max_gx; ++x)
        {
            for (int y = min_gy; y <= max_gy; ++y)
            {
                occupancy_buffer_[x + y * grid_w_] = 1000; // 1000 代表静态墙，永远不会衰减
            }
        }
    }
    ROS_INFO("[GridMap] 静态高精电子围栏已刻入地图！原点:(%.2f, %.2f)", start_x, start_y);
}

void GridMap::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // 1. 瞬间擦除地图上所有的动态障碍物 (只保留静态墙 1000)
    for (size_t i = 0; i < occupancy_buffer_.size(); ++i)
    {
        if (occupancy_buffer_[i] < 1000)
        {
            occupancy_buffer_[i] = 0;
        }
    }

    // 2. 接收新的点云 (使用 PointXYZ 而非 PointXYZI，避免 intensity 字段缺失问题)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    for (const auto &pt : cloud.points)
    {
        int gx, gy;
        if (!posToIndex(Eigen::Vector2d(pt.x, pt.y), gx, gy))
            continue;

        // 直接将所有点视为障碍物
        occupancy_buffer_[gx + gy * grid_w_] = std::max(occupancy_buffer_[gx + gy * grid_w_], 999);
    }
}

// 🧱 积木三：寻找最近的白格子 (BFS 广度优先搜索)
bool GridMap::searchNearestFreeSpace(const Eigen::Vector2d &pt, Eigen::Vector2d &free_pt) const
{
    int start_x, start_y;
    if (!posToIndex(pt, start_x, start_y))
        return false;

    // 如果起点本身就是安全的，直接返回
    if (!isOccupied(start_x, start_y))
    {
        free_pt = pt;
        return true;
    }

    std::queue<Eigen::Vector2i> q;
    std::vector<bool> visited(grid_w_ * grid_h_, false);

    q.push(Eigen::Vector2i(start_x, start_y));
    visited[start_x + start_y * grid_w_] = true;

    // 经典的 BFS 八向搜索方向
    int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    int max_search_steps = 1000; // 防止卡死

    while (!q.empty() && max_search_steps-- > 0)
    {
        Eigen::Vector2i cur = q.front();
        q.pop();
        // 1. 检查 cur 这个格子是否 isOccupied？
        if(!isOccupied(cur.x(), cur.y()))
        {
            // 2. 如果不是障碍物，说明找到水面了！用 indexToPos 把 cur 转成 free_pt，然后 return true;
            indexToPos(cur.x(), cur.y(), free_pt);
            return true;
        }
        // 3. 如果还是障碍物，用一个 for 循环遍历 8 个方向 (dx, dy)，计算 nx, ny。
        for(int i = 0; i < 8; ++i)
        {
            int nx = cur.x() + dx[i];
            int ny = cur.y() + dy[i];
            // 4. 检查 nx, ny 是否越界？是否已被 visited？如果没有，设为 visited 并 push 进 q。
            if(nx >= 0 && nx < grid_w_ && ny >= 0 && ny < grid_h_)
            {
                int idx = nx + ny * grid_w_;
                if(!visited[idx])
                {
                    visited[idx] = true;
                    q.push(Eigen::Vector2i(nx, ny));
                }
            }
        }
    }
    return false; // 没找到逃生点
}

// 🧱 积木四：ESDF-Free 灵魂 —— 计算排斥梯度
bool GridMap::getObstacleGradient(const Eigen::Vector2d &pt, Eigen::Vector2d &grad, double &penetration_depth) const
{
    if (!isOccupied(pt))
        return false;

    Eigen::Vector2d free_pt;
    if (!searchNearestFreeSpace(pt, free_pt))
    {
        grad = Eigen::Vector2d::Zero();
        penetration_depth = 0;
        return true;
    }

    Eigen::Vector2d dir = free_pt - pt;
    double dist = dir.norm(); // [修复] 补充 double 声明

    if (dist < 1e-4)
    {
        grad = Eigen::Vector2d::Random().normalized();
        penetration_depth = 0;
        return true;
    }

    // [修复] 直接赋值，绝不能加前面的类型声明！
    grad = dir / dist;
    penetration_depth = dist;
    return true;
}

int GridMap::getGridW() const
{
    return grid_w_;
}
int GridMap::getGridH() const
{
    return grid_h_;
}

// 在文件末尾，加上发布地图的实现函数：
void GridMap::publishMap()
{
    if (occupancy_buffer_.empty())
        return;

    nav_msgs::OccupancyGrid msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map"; // RViz里的 Fixed Frame 记得填 "map" 或 "local_origin"
    msg.info.resolution = param_.resolution;
    msg.info.width = grid_w_;
    msg.info.height = grid_h_;
    msg.info.origin.position.x = param_.origin_x;
    msg.info.origin.position.y = param_.origin_y;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_w_ * grid_h_);
    for (int i = 0; i < grid_w_ * grid_h_; ++i)
    {
        if (occupancy_buffer_[i] == 1000)
            msg.data[i] = 100; // 静态虚拟墙：纯黑
        else if (occupancy_buffer_[i] >= 999)
            msg.data[i] = 80; // 雷达障碍物：深灰
        else
            msg.data[i] = 0; // 安全区：纯白
    }
    map_pub_.publish(msg);
}