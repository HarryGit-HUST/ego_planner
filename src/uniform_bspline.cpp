#include "uniform_bspline.h"
#include <cmath>
#include <algorithm>
#include <iostream>

UniformBspline::UniformBspline() {}

UniformBspline::UniformBspline(const Eigen::MatrixXd &points, int order, double interval)
{
    setUniformBspline(points, order, interval);
}

UniformBspline::~UniformBspline() {}

void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, int order, double interval)
{
    control_points_ = points;
    p_ = order;
    interval_ = interval;
    n_ = control_points_.cols() - 1;
    m_ = n_ + p_ + 1;
    buildKnotVector();
}

void UniformBspline::buildKnotVector()
{
    knot_.resize(m_ + 1);
    for (int i = 0; i <= m_; i++)
    { 
        knot_(i) = i * interval_;
    }
}


double UniformBspline::getTimeSum() const
{
    
    return knot_(m_ - p_) - knot_(p_);
}

void UniformBspline::setPhysicalLimits(double max_vel, double max_acc)
{
    limit_vel_ = max_vel;
    limit_acc_ = max_acc;
}

// [重构] 极度优雅的求导：返回一个新的 B 样条！
UniformBspline UniformBspline::getDerivative() const
{
    int count = control_points_.cols() - 1;
    Eigen::MatrixXd V(control_points_.rows(), count);

    // V_i = p * (P_{i+1} - P_i) / dt
    for (int i = 0; i < count; ++i)
    {
        V.col(i) = p_ * (control_points_.col(i + 1) - control_points_.col(i)) / interval_;
    }

    // 返回一条降了一阶 (p_ - 1) 的新 B 样条
    return UniformBspline(V, p_ - 1, interval_);
}

// [修复] 加上了 UniformBspline:: 作用域
bool UniformBspline::checkFeasibility(double &ratio, bool show_info) const
{
    // 直接用面向对象的思想，极其优雅！
    UniformBspline vel_spline = getDerivative();
    UniformBspline acc_spline = vel_spline.getDerivative();

    double max_vel = vel_spline.getControlPoints().colwise().norm().maxCoeff();
    double max_acc = acc_spline.getControlPoints().colwise().norm().maxCoeff();

    if (show_info)
    {
        std::cout << "[B样条诊断] 最大速度: " << max_vel << " m/s, 最大加速度: " << max_acc << " m/s^2" << std::endl;
    }

    if (max_vel > limit_vel_ || max_acc > limit_acc_)
    {
        double vel_ratio = max_vel / limit_vel_;
        double acc_ratio = max_acc / limit_acc_;
        ratio = std::max(vel_ratio, acc_ratio);
        if (show_info)
        {
            std::cout << "  ❌ 轨迹超载！需要将时间拉长 " << ratio << " 倍！" << std::endl;
        }
        return false;
    }
    return true;
}

void UniformBspline::lengthenTime(double ratio)
{
    interval_ *= ratio;
    buildKnotVector(); // 重新刻刻度
}

Eigen::Vector2d UniformBspline::evaluateDeBoor(double t) const
{
    double t_min = knot_(p_);
    double t_max = knot_(m_ - p_);
    if (t < t_min)
        t = t_min;
    if (t > t_max)
        t = t_max;

    int k = p_;
    while (k < m_ - p_ && t >= knot_(k + 1))
    {
        k++;
    }

    std::vector<Eigen::Vector2d> d;
    for (int i = 0; i <= p_; ++i)
    {
        d.push_back(control_points_.col(k - p_ + i));
    }

    // 你写的完美 De Boor-Cox 算法！
    for (int r = 1; r <= p_; ++r)
    {
        for (int i = p_; i >= r; --i)
        {
            double alpha = (t - knot_(i + k - p_)) / (knot_(i + 1 + k - r) - knot_(i + k - p_));
            d[i] = (1.0 - alpha) * d[i - 1] + alpha * d[i];
        }
    }
    return d[p_];
}

// [修复] 加上了 UniformBspline:: 作用域，并引入了“首尾重叠”法则
void UniformBspline::parameterizeToBspline(const double ts,
                                           const std::vector<Eigen::Vector2d> &point_set,
                                           const std::vector<Eigen::Vector2d> &start_end_derivative,
                                           Eigen::MatrixXd &ctrl_pts)
{
    int num = point_set.size();
    if (num < 2)
        return;

    // 为了让 B 样条两端牢牢钉在起点和终点上，首尾控制点必须重复 3 次 (p_=3)
    int p = 3;
    ctrl_pts.resize(2, num + 2 * p);

    // 1. 重复插入起点 3 次
    for (int i = 0; i < p; ++i)
    {
        ctrl_pts.col(i) = point_set.front();
    }
    // 2. 插入中间的 A* 路径点
    for (int i = 0; i < num; ++i)
    {
        ctrl_pts.col(i + p) = point_set[i];
    }
    // 3. 重复插入终点 3 次
    for (int i = 0; i < p; ++i)
    {
        ctrl_pts.col(num + p + i) = point_set.back();
    }
}