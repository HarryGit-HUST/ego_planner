#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>
#include <thread> // [新增] 多线程库
#include <atomic> // [新增] 原子变量库 (线程安全的红绿灯)

#include <ros/ros.h>
#include <Eigen/Dense>

#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

class PlannerManager;

enum class MissionState
{
    IDLE,
    TAKEOFF,
    WAIT_FOR_MAP, // [新增] 悬停等图状态！防止起飞瞬间地图为空导致 A* 走直线穿墙
    NAV_RECOG_AREA,
    HOVER_RECOGNIZE,
    NAV_AIRDROP_AREA,
    HOVER_AIRDROP,
    NAV_STRIKE_AREA,
    LASER_STRIKE,
    RETURN_TO_LAUNCH,
    LANDING,
    FINISHED
};

class MissionController
{
public:
    MissionController();
    ~MissionController();

    void init(ros::NodeHandle &nh);
    void tick();
    bool isConnected() const { return is_connected_; }

private:
    MissionState current_state_;
    Eigen::Vector3d current_pos_;
    double current_yaw_;

    mavros_msgs::State mavros_state_;
    bool is_connected_;
    bool has_global_plan_;
    Eigen::Vector3d init_pos_;
    double init_yaw_;
    ros::Time hover_start_time_;

    // [新增] 悬停等图计时器
    ros::Time map_wait_start_time_;

    // [新增] 多线程防阻塞保命锁！
    std::atomic<bool> is_planning_{false};

    struct Param
    {
        double takeoff_height;
        Eigen::Vector2d wp_recog;
        Eigen::Vector2d wp_airdrop;
        Eigen::Vector2d wp_strike;
    } param_;

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    std::shared_ptr<PlannerManager> planner_manager_;

    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
    bool setOffboardAndArm();

    bool flyToXY(const Eigen::Vector2d &target_xy);
    void publishSetpoint(const Eigen::Vector2d &xy, double z, double yaw);
};

#endif // MISSION_CONTROLLER_H