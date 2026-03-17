#ifndef GRID_MAP_H
#define GRID_MAP_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <vector>
#include <memory>

// 只有测绘部需要和传感器打交道，所以 PCL 和 ROS 传感器头文件只在这里出现！
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

// 静态墙体结构体（内部使用）
struct StaticWall
{
    Eigen::Vector2d center;
    Eigen::Vector2d dir;
    double length;
    double thickness;
    std::vector<Eigen::Vector2d> footprint;
};

class GridMap
{
public:
    GridMap();
    ~GridMap();

    // 1. 初始化：加载地图参数，并独自去订阅点云话题
    void init(ros::NodeHandle &nh);

    // 2. 清理函数：起飞瞬间调用，擦除地面噪点
    void clearMap();

    // =================================================================
    // 【核心 API 组 1】：提供给 A* 探路部使用的离散接口 (0 或 1)
    // =================================================================

    // 检查世界坐标 (x, y) 是否是障碍物
    bool isOccupied(const Eigen::Vector2d &pos) const;
    // 检查栅格坐标 (grid_x, grid_y) 是否是障碍物
    bool isOccupied(int gx, int gy) const;

    // 坐标系互相转换工具
    bool posToIndex(const Eigen::Vector2d &pos, int &gx, int &gy) const;
    void indexToPos(int gx, int gy, Eigen::Vector2d &pos) const;

    int getGridW() const;
    int getGridH() const;


        // =================================================================
        // 【核心 API 组 2】：提供给 L-BFGS 优化器使用的连续梯度接口 (ESDF-Free 核心)
        // =================================================================

        // 灵魂函数：检查某一个控制点 pt 是否陷入了障碍物？
        // 如果陷入了，返回 true，并且通过引用带回“逃生方向 (grad)”和“陷入深度 (penetration_depth)”
        bool getObstacleGradient(const Eigen::Vector2d &pt,
                                 Eigen::Vector2d &grad,
                                 double &penetration_depth) const;

private:
    // ================= 测绘部的私有财产 =================

    // 1. 动态二维数组，存储栅格占据概率 (0~100)
    std::vector<int> occupancy_buffer_;

    // 2. 高精电子围栏 (写死在测绘部内部)
    std::vector<StaticWall> static_walls_;

    // 3. 地图的物理属性
    struct Param
    {
        double resolution;
        double width_m;
        double height_m;
        double origin_x;
        double origin_y;
        int decay_rate;          // 衰减速率
        float front_x ;
        float back_x ;
        float left_y;
        float right_y;
        float exp; // 墙体厚度/安全余量
        float intensity_threshold
    } param_;

    int grid_w_;
    int grid_h_;

    // ================= 测绘部的私有通信设备 =================
    ros::Subscriber cloud_sub_;
    ros::Publisher map_pub_;


    // ================= 测绘部的工作手册 =================

    // 当收到雷达点云时触发，只负责把点云画到 occupancy_buffer_ 里
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

    // 初始化时建好四堵虚拟墙
    void buildStaticWalls();

    // BFS 广度优先搜索：如果在障碍物内部，向外寻找最近的安全点 (为了算梯度)
    bool searchNearestFreeSpace(const Eigen::Vector2d &pt, Eigen::Vector2d &free_pt) const;
};

#endif // GRID_MAP_H