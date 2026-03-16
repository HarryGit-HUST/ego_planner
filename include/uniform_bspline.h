#ifndef UNIFORM_BSPLINE_H
#define UNIFORM_BSPLINE_H

#include <Eigen/Dense>
#include <vector>
#include <iostream>

// ============================================================================
// 纯数学部：均匀 B 样条曲线引擎 (Uniform B-Spline)
// 没有任何物理实体的概念，纯粹的代数与微积分工具类
// ============================================================================

class UniformBspline
{
public:
    // 默认构造与核心构造函数
    UniformBspline();
    UniformBspline(const Eigen::MatrixXd &points, int order, double interval);
    ~UniformBspline();

    // ========================================================================
    // 1. 核心属性与重置
    // ========================================================================
    void setUniformBspline(const Eigen::MatrixXd &points, int order, double interval);

    Eigen::MatrixXd getControlPoints() const { return control_points_; }
    double getInterval() const { return interval_; }
    int getDegree() const { return p_; }

    // 获取整条曲线的物理飞行总时间
    double getTimeSum() const;

    // ========================================================================
    // 2. 空间位置与微积分 (Evaluation & Calculus)
    // ========================================================================

    // 计算 t 时刻的绝对位置 (使用 De Boor-Cox 算法，极速求解)
    Eigen::Vector2d evaluateDeBoor(double t) const;

    // 获取导数曲线！(这是 B 样条的魔法：它的导数依然是一条 B 样条曲线，只是阶数 -1)
    UniformBspline getDerivative() const;

    // ========================================================================
    // 3. 动力学约束与时间重分配 (Time Reallocation / Refinement)
    // ========================================================================

    // 设置无人机的物理极限
    void setPhysicalLimits(double max_vel, double max_acc);

    // 检查当前曲线是否超速或超载？
    // 如果超载，返回 false，并通过引用带回一个放缩比例 ratio
    bool checkFeasibility(double &ratio, bool show_info = false) const;

    // 时间魔法：不改变曲线的空间形状，仅仅把飞行时间拉长 ratio 倍，从而降低速度和加速度！
    void lengthenTime(double ratio);

    // ========================================================================
    // 4. 静态工厂方法 (Static Methods)
    // ========================================================================

    // 给定 A* 搜索出来的一堆离散路点，反解出 B 样条的“控制点”。
    // 这让 B 样条曲线能最大程度地贴合 A* 的折线走廊。
    static void parameterizeToBspline(const double ts,
                                      const std::vector<Eigen::Vector2d> &point_set,
                                      const std::vector<Eigen::Vector2d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

private:
    // ================= 核心数学数据 =================

    int p_; // B 样条的阶数 (Degree)，通常 EGO-Planner 使用 3 阶 (Cubic)
    int n_; // 控制点的数量 - 1
    int m_; // 节点向量的数量 - 1 (m = n + p + 1)

    double interval_;                // 节点之间的时间间隔 (Delta t)
    Eigen::VectorXd knot_;           // 节点向量 (Knot Vector)，记录时间刻度
    Eigen::MatrixXd control_points_; // 控制点矩阵 (2 x N)，因为 L-BFGS 用矩阵算梯度比 vector 快 10 倍！

    // ================= 动力学极限 =================
    double limit_vel_;
    double limit_acc_;

    // ================= 私有数学辅助函数 =================
    // 基础的基函数计算 (B-spline basis function)
    void buildKnotVector();
};

#endif // UNIFORM_BSPLINE_H