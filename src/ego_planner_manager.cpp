#include "ego_planner_manager.h"
#include <iostream>
// [修复 9]：极其关键！既然你在这个 cpp 里用到了这些类的具体方法，必须包含它们的真实头文件！
#include "grid_map.h"
#include "a_star.h"
#include "bspline_optimizer.h"
PlannerManager::PlannerManager() {}
PlannerManager::~PlannerManager() {}
// (初始化等基础函数省略，你需要自己把指针实例化、并赋值给下属)
void PlannerManager::init(ros::NodeHandle &nh)
{
    // 1. 从ROS参数服务器读取配置
    nh.param("max_vel", param_.max_vel, 1.0);
    nh.param("max_acc", param_.max_acc, 1.0);
    nh.param("ctrl_pt_dist", param_.ctrl_pt_dist, 0.1);

    // 2. 实例化所有子模块（关键：智能指针初始化）
    grid_map_ = std::make_shared<GridMap>();
    a_star_ = std::make_shared<AStar>();
    optimizer_ = std::make_shared<BsplineOptimizer>();

    // 3. 子模块初始化（传递参数、地图指针）
    grid_map_->init(nh);
    a_star_->init(grid_map_); // [修复] 名字统一为 init

    optimizer_->setEnvironment(grid_map_); // [修复] 名字统一
    optimizer_->setParam(nh);
}
bool PlannerManager::replan(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &target_pt)
{
    // TODO 1: 让 A* 出马，寻找路径
    std::vector<Eigen::Vector2d> astar_path;
    if (!a_star_->search(start_pt, target_pt, astar_path))
    {
        ROS_WARN("[CEO] A* 寻路失败！前方完全封闭！");
        return false; // 死胡同
    }

    //[核心修复 1] 正确计算 B样条的时间间隔 ts
    // 计算 A* 路径总长度
    double total_len = 0.0;
    for (size_t i = 0; i < astar_path.size() - 1; ++i)
    {
        total_len += (astar_path[i + 1] - astar_path[i]).norm();
    }
    // 假设以最高速度飞行，总时间 = 长度 / 速度
    double total_time = total_len / param_.max_vel;
    // 按照期望的空间间距划分控制点
    int num_segments = std::max(1, (int)(total_len / param_.ctrl_pt_dist));
    double ts = total_time / num_segments;

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, astar_path, std::vector<Eigen::Vector2d>(), ctrl_pts);

    if (!optimizer_->optimize(ctrl_pts, ts))
    {
        ROS_WARN("[CEO] 轨迹优化失败！");
        return false;
    }

    // TODO 4: 把优化好的控制点封装成 B样条轨迹档案
    local_traj_.setUniformBspline(ctrl_pts, 3, 0.1);

    // TODO 5: 动力学时间重分配
    double ratio;
    if (!local_traj_.checkFeasibility(ratio))
    {
        local_traj_.lengthenTime(ratio);
    }

    // 盖上时间戳，归档入库！
    traj_start_time_ = ros::Time::now();
    ROS_INFO("[CEO] 轨迹生成成功！耗时极短。");
    return true;
}

Eigen::Vector2d PlannerManager::getPosition(double t_sec)
{
    // TODO: 从 local_traj_ 里调用 evaluateDeBoor(t_sec) 并返回
    // 细节保护：如果 t_sec 超出了轨迹总时间，就返回轨迹的最后一个点
    double t_sum = local_traj_.getTimeSum();
    if (t_sec > t_sum)
    {
        return local_traj_.evaluateDeBoor(t_sum);
    }
    else
    {
        return local_traj_.evaluateDeBoor(t_sec);
    }
}
Eigen::Vector2d PlannerManager::getVelocity(double t_sec)
{
    if(t_sec > local_traj_.getTimeSum())
    {
        return Eigen::Vector2d(0, 0); // 轨迹结束后速度为零
    }
    else
    {
        UniformBspline derivative_traj = local_traj_.getDerivative();
        return derivative_traj.evaluateDeBoor(t_sec);
    }
}
bool PlannerManager::checkCollision()
{
    double duration = local_traj_.getTimeSum();
    // [核心修复 2] CEO 沿着自己生成的轨迹抽样检查，看看有没有撞墙
    // 每隔 0.1 秒踩个点，去问地图：前面堵了没？
    for (double t = 0.0; t <= duration; t += 0.1)
    {
        Eigen::Vector2d pt = local_traj_.evaluateDeBoor(t);
        if (grid_map_->isOccupied(pt))
        {
            ROS_WARN_THROTTLE(1.0, "[CEO 警报] 前方出现新障碍物！请求重新规划！");
            return true;
        }
    }
    return false;
}

void PlannerManager::buildWalls(double start_x, double start_y)
{
    // CEO 不亲自干活，直接使唤底层的 grid_map_ 部门去建墙
    if (grid_map_ != nullptr)
    {
        grid_map_->buildStaticWalls(start_x, start_y);
    }
}