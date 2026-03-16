#ifndef EGO_PLANNER_MANAGER_H   
#define EGO_PLANNER_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

// 引入 B 样条数学类（因为我们要向 Boss 返回曲线数据，必须知道它长什么样）
// 假设你底层写了一个 uniform_bspline.h
#include "uniform_bspline.h"

// ================= 前向声明下属部门 =================
// CEO 只需要知道它们的名字，不需要在这里包含它们庞大的 .h 文件
class GridMap;
class AStar;
class BsplineOptimizer;

class PlannerManager
{
public:
    PlannerManager();
    ~PlannerManager();

    // 1. 初始化：CEO 上任，读取参数，并成立下属的三个部门
    void init(ros::NodeHandle &nh);

    // 2. 核心指令：要求 CEO 马上生成一条从 start 到 target 的避障轨迹
    // 返回值：如果成功找到路返回 true，如果彻底死锁返回 false
    bool replan(const Eigen::Vector3d &start_pt,
                const Eigen::Vector3d &start_vel,
                const Eigen::Vector3d &target_pt);

    // 3. 轨迹查询：Boss 拿着当前时间 t 来问 CEO，飞机现在该飞到哪个位置？
    // 这个函数供飞控在 50Hz 的 tick() 循环里疯狂调用
    Eigen::Vector3d getPosition(double t_sec);
    Eigen::Vector3d getVelocity(double t_sec);

    // 4. 突发检查：当前正在飞的轨迹，前方是不是突然出现了新障碍物？
    bool checkCollision();

private:
    // ================= 雇佣的下属部门 (智能指针) =================
    // 使用 shared_ptr，生命周期由 CEO 统一管理
    std::shared_ptr<GridMap> grid_map_;
    std::shared_ptr<AStar> a_star_;
    std::shared_ptr<BsplineOptimizer> optimizer_;

    // ================= 轨迹档案室 =================
    // 每次 replan 成功后，将最终曲线存放在这里
    UniformBspline local_traj_;
    ros::Time traj_start_time_; // 这条轨迹是几分几秒开始执行的？(极其重要)

    // ================= CEO 的私有配置参数 =================
    struct Param
    {
        double max_vel;
        double max_acc;
        double ctrl_pt_dist; // 控制点之间的间隔
        // ... 其他由 CEO 管控的参数
    } param_;

    // ================= 内部工作流 (流水线拆解) =================
    // replan() 函数太长了，CEO 会把它拆成几个内部步骤：

    // 步骤 A：命令 A* 探路，并提取出初始的 B 样条控制点
    bool generateInitialControlPoints(const Eigen::Vector3d &start_pt,
                                      const Eigen::Vector3d &target_pt,
                                      std::vector<Eigen::Vector3d> &init_pts);

    // 步骤 B：把初始点丢给优化器，推离障碍物
    bool optimizeTrajectory(std::vector<Eigen::Vector3d> &ctrl_pts);

    // 步骤 C：检查动力学，如果超速了，就把轨迹时间拉长 (Time Reallocation)
    bool refineTrajectoryTime(UniformBspline &traj);
};

#endif // PLANNER_MANAGER_H