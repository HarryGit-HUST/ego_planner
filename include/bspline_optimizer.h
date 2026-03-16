#ifndef BSPLINE_OPTIMIZER_H
#define BSPLINE_OPTIMIZER_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <memory>
#include <vector>

// 引入底层环境 (眼睛)
class GridMap;

// ============================================================================
// 法务优化部：B 样条轨迹最优化核心 (The Optimization Engine)
// 结合 L-BFGS 拟牛顿法，处理平滑度、碰撞和动力学极限的多目标优化
// ============================================================================

class BsplineOptimizer
{
public:
    BsplineOptimizer();
    ~BsplineOptimizer();

    // 1. 初始化：读取 YAML 中的各项惩罚权重
    void setParam(ros::NodeHandle &nh);

    // 2. 依赖注入：挂载环境地图 (CEO 会把 GridMap 的指针递给他)
    void setEnvironment(std::shared_ptr<GridMap> map);

    // ========================================================================
    // 3. 核心业务接口：执行最优化！
    // 输入/输出：ctrl_pts (传入初始控制点，函数结束后变成完美控制点)
    // 输入：ts (B 样条的时间间隔)
    // ========================================================================
    bool optimize(Eigen::MatrixXd &ctrl_pts, double ts);

private:
    // ================= 依赖的下属与环境 =================
    std::shared_ptr<GridMap> grid_map_;

    // ================= 部门私有参数 (权重调节器) =================
    struct Param
    {
        double weight_smooth;      // 平滑度权重 (越大曲线越直)
        double weight_collision;   // 避障权重 (越大离墙越远)
        double weight_feasibility; // 动力学权重 (惩罚超速超载)

        double max_vel;       // 物理极限：最大速度
        double max_acc;       // 物理极限：最大加速度
        double safe_distance; // 期望的安全距离

        int max_iteration_num; // L-BFGS 最大迭代次数 (比如 300 次)
    } param_;

    // ================= 当前正在优化的轨迹属性 =================
    int order_;               // B 样条阶数 (默认 3 阶)
    int pt_num_;              // 控制点总数
    double bspline_interval_; // 节点时间间隔 dt

    // ========================================================================
    // 内部数学引擎：代价与梯度计算 (The Math Core)
    // 计算总代价 Cost，并把每个控制点受到的推力 (Gradient) 填入 grad 矩阵
    // ========================================================================

    // 总线函数：汇总所有代价
    void combineCost(const double *x, double *grad, double &f_combine);

    // 分项代价 1：平滑度 (惩罚控制点之间的距离不均和急折角)
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    // 分项代价 2：碰撞斥力 (ESDF-Free 核心！向 GridMap 索要逃生向量)
    void calcCollisionCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    // 分项代价 3：动力学可行性 (检查导数是否超标)
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);

    // ========================================================================
    // C++ 与 C 的终极桥梁：L-BFGS 静态回调函数
    // L-BFGS 库通常是纯 C 语言写的，只认静态函数指针，我们必须用一个壳把它映射回类的内部！
    // ========================================================================
    static double costFunction(void *instance, const double *x, double *g, const int n);
};

#endif // BSPLINE_OPTIMIZER_H