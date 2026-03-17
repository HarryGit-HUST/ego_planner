#ifndef A_STAR_H
#define A_STAR_H

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <memory>

// 前向声明测绘部，A* 必须要查图才能寻路
class GridMap;

// =========================================================
// 内部数据结构：A* 节点
// =========================================================
struct AStarNode
{
    Eigen::Vector2i index; // 在栅格地图里的二维索引 (x, y)
    Eigen::Vector2d pos;   // 在世界坐标系里的物理位置 (米)

    double g_score; // 走到这的实际代价
    double f_score; // f = g + h (总评估代价)

    AStarNode *parent; // 记录来时的路 (用于最后回溯)
    int state;         // 状态：0=未访问, 1=在Open表中, 2=在Closed表中

    // 构造函数
    AStarNode(Eigen::Vector2i idx, Eigen::Vector2d p)
        : index(idx), pos(p), g_score(1e9), f_score(1e9), parent(nullptr), state(0) {}
};
// 2. 优先队列比较器：让队列按 f_score 从小到大排序（最小堆）
struct NodeComparator
{
    bool operator()(const AStarNode *a, const AStarNode *b) const
    {
        return a->f_score > b->f_score;
    }
};

// =========================================================
// 探路部 (AStar 算法引擎)
// =========================================================
class AStar
{
public:
    AStar();
    ~AStar();

    // 1. 部门入驻：Boss 会把测绘部(GridMap)的指针交给他
    void init(std::shared_ptr<GridMap> grid_map);

    // 2. 核心业务：给定起点和终点，返回一条“极简、不撞墙”的物理折线
    bool search(const Eigen::Vector2d &start_pt,
                const Eigen::Vector2d &target_pt,
                std::vector<Eigen::Vector2d> &path);

private:
    // ================= 依赖的下属 =================
    std::shared_ptr<GridMap> grid_map_;

    // ================= 高阶优化：核心工具库 =================

    // 1. 打破对称性的启发式函数 (Tie-breaker Heuristic)
    double getHeuristic(const Eigen::Vector2d &p1,
                        const Eigen::Vector2d &p2,
                        const Eigen::Vector2d &start_pt);

    // 2. 视距法路径剪枝 (Line-of-Sight Pruning) - 去除锯齿，输出极简控制点！
    void simplifyPath(const std::vector<Eigen::Vector2d> &raw_path,
                      std::vector<Eigen::Vector2d> &simplified_path);

    // 3. 碰撞射线检测 (Bresenham 算法)：检查两点之间有没有被黑块挡住
    bool checkLineOfSight(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

    // 4. 起点/终点安全卫士：如果给的起点在墙里，就近找个安全的白格子
    bool findNearestFreePoint(Eigen::Vector2d &pt);
};

#endif // A_STAR_H