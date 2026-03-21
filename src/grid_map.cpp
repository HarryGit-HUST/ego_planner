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
    nh.param("planner/map_safe_distance", param_.safe_distance, 0.2f);

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
    for (size_t i = 0; i < occupancy_buffer_.size(); ++i)
    {
        if (occupancy_buffer_[i] < 1000)
            occupancy_buffer_[i] = 0;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);

    // 重新引入膨胀机制！
    int inf_cells = std::ceil(param_.safe_distance / param_.resolution);
    int inf_sq = inf_cells * inf_cells;

    for (const auto &pt : cloud.points)
    {
        // 1. Intensity 过滤
        if (pt.intensity < param_.intensity_threshold)
            continue;

        int cx, cy;
        if (!posToIndex(Eigen::Vector2d(pt.x, pt.y), cx, cy))
            continue;

        // 2. 将高置信度点向四周安全膨胀
        for (int dx = -inf_cells; dx <= inf_cells; ++dx)
        {
            for (int dy = -inf_cells; dy <= inf_cells; ++dy)
            {
                if (dx * dx + dy * dy <= inf_sq)
                {
                    int nx = cx + dx, ny = cy + dy;
                    if (nx >= 0 && nx < grid_w_ && ny >= 0 && ny < grid_h_)
                    {
                        if (occupancy_buffer_[nx + ny * grid_w_] != 1000)
                        {
                            // 用 std::max 防止高频点被低频点覆盖
                            occupancy_buffer_[nx + ny * grid_w_] = std::max(occupancy_buffer_[nx + ny * grid_w_], (int)(pt.intensity * 999));
                        }
                    }
                }
            }
        }
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

// 🧱 积木四：ESDF-Free 灵魂 —— 计算排斥梯度（连续势场版）
// 返回值：true = 在障碍物内，false = 在障碍物外
// penetration_depth: 正值表示在障碍物外的距离，负值表示在障碍物内的深度
// grad: 始终指向远离障碍物的方向（逃生方向）
bool GridMap::getObstacleGradient(const Eigen::Vector2d &pt, Eigen::Vector2d &grad, double &penetration_depth) const
{
    int cx, cy;
    if (!posToIndex(pt, cx, cy))
    {
        grad = Eigen::Vector2d(1.0, 0.0);
        penetration_depth = param_.safe_distance;
        return false;
    }

    if (isOccupied(pt))
    {
        // 在障碍物内：寻找最近的白格子 (自由空间)
        Eigen::Vector2d free_pt;
        if (!searchNearestFreeSpace(pt, free_pt))
        {
            grad = Eigen::Vector2d(1.0, 0.0);
            penetration_depth = -2.0;
            return true;
        }

        // [连续性修复] 用欧几里得精确中心距离，代替阶梯状跳变距离
        Eigen::Vector2d dir = free_pt - pt;
        double dist = dir.norm();

        if (dist < 1e-4)
        {
            grad = Eigen::Vector2d(1.0, 0.0);
            penetration_depth = -0.05;
            return true;
        }

        grad = dir / dist;         // 逃向 free_pt 的单位向量
        penetration_depth = -dist; // 负值代表陷入深度
        return true;
    }
    else
    {
        // 在障碍物外：寻找最近的黑格子 (障碍物中心)
        std::queue<Eigen::Vector2i> q;
        std::vector<bool> visited(grid_w_ * grid_h_, false);

        q.push(Eigen::Vector2i(cx, cy));
        visited[cx + cy * grid_w_] = true;

        int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
        int max_search_steps = 150;
        Eigen::Vector2d obs_pt(-1, -1);

        while (!q.empty() && max_search_steps-- > 0)
        {
            Eigen::Vector2i cur = q.front();
            q.pop();

            if (isOccupied(cur.x(), cur.y()))
            {
                // [连续性修复] 获取障碍物格子的精确物理中心坐标
                indexToPos(cur.x(), cur.y(), obs_pt);
                break;
            }

            for (int i = 0; i < 8; ++i)
            {
                int nx = cur.x() + dx[i];
                int ny = cur.y() + dy[i];
                if (nx >= 0 && nx < grid_w_ && ny >= 0 && ny < grid_h_)
                {
                    int idx = nx + ny * grid_w_;
                    if (!visited[idx])
                    {
                        visited[idx] = true;
                        q.push(Eigen::Vector2i(nx, ny));
                    }
                }
            }
        }

        if (obs_pt.x() < 0)
        {
            grad = Eigen::Vector2d(1.0, 0.0);
            penetration_depth = param_.safe_distance + 1.0;
            return false;
        }

        // 计算精确的推力向量 (从障碍物中心推向当前点)
        Eigen::Vector2d dir = pt - obs_pt;
        double dist = dir.norm();

        if (dist < 1e-4)
        {
            grad = Eigen::Vector2d(1.0, 0.0);
            penetration_depth = 0.0;
            return false;
        }

        grad = dir / dist;        // 远离障碍物的单位向量
        penetration_depth = dist; // 正值表示在障碍物外的安全距离
        return false;
    }
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
        else if (occupancy_buffer_[i] > 0.0)
            msg.data[i] = 80; // 雷达障碍物：深灰
        else
            msg.data[i] = 0; // 安全区：纯白
    }
    map_pub_.publish(msg);
}