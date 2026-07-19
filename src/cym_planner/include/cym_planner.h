#ifndef CYM_PLANNER_H_
#define CYM_PLANNER_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <base_local_planner/costmap_model.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_core/base_local_planner.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <visualization_msgs/MarkerArray.h>

namespace cym_planner
{

class CymPlanner : public nav_core::BaseLocalPlanner
{
public:
    CymPlanner();
    ~CymPlanner() override;

    void initialize(
        std::string name,
        tf2_ros::Buffer* tf,
        costmap_2d::Costmap2DROS* costmap_ros) override;

    bool setPlan(
        const std::vector<geometry_msgs::PoseStamped>& plan) override;

    bool computeVelocityCommands(
        geometry_msgs::Twist& cmd_vel) override;

    bool isGoalReached() override;

private:
    friend class CymPlannerTestPeer;

    enum class PlannerState
    {
        TRACK,
        SELECT_SIDE,
        AVOID_LEFT,
        AVOID_RIGHT,
        RETURN_PATH,
        STOPPING,
        GOAL_ALIGN
    };

    struct PathPoint
    {
        double x = 0.0;
        double y = 0.0;
        double yaw = 0.0;
        double s = 0.0;
        double curvature = 0.0;
        double offset = 0.0;
        double clearance = std::numeric_limits<double>::infinity();
    };

    struct CollisionResult
    {
        bool collision = false;
        int first_collision_index = -1;
        int last_collision_index = -1;
        double first_collision_distance = 0.0;
    };

    struct CandidatePath
    {
        // -1: 右绕，0: 参考路径，+1: 左绕。
        int side = 0;
        double peak_offset = 0.0;
        double score = std::numeric_limits<double>::infinity();
        double minimum_clearance = std::numeric_limits<double>::infinity();
        bool valid = false;
        std::vector<PathPoint> points;
    };

    void carryModeCallback(const std_msgs::Bool::ConstPtr& message);

    bool transformPose(
        const geometry_msgs::PoseStamped& input,
        const std::string& target_frame,
        geometry_msgs::PoseStamped& output) const;

    int findNearestGlobalIndex(
        const geometry_msgs::PoseStamped& robot_pose);

    bool cropGlobalPlan(
        int start_index,
        double horizon,
        std::vector<PathPoint>& cropped_path) const;

    std::vector<PathPoint> resamplePath(
        const std::vector<PathPoint>& path,
        double resolution) const;

    void computePathGeometry(std::vector<PathPoint>& path) const;

    bool checkPoseCollision(double x, double y, double yaw) const;

    CollisionResult evaluatePathCollision(
        const std::vector<PathPoint>& path,
        double horizon);

    CandidatePath generateSeedCandidate(
        const std::vector<PathPoint>& reference,
        const CollisionResult& collision,
        int side,
        double peak_offset) const;

    CandidatePath findFirstFeasibleCandidate(
        const std::vector<PathPoint>& reference,
        const CollisionResult& collision,
        int side);

    CandidatePath generatePathFromOffsets(
        const std::vector<PathPoint>& reference,
        const std::vector<double>& offsets,
        int side) const;

    CandidatePath generateReturnCandidate(
        const std::vector<PathPoint>& reference,
        int side);

    void refineOffsets(
        const std::vector<PathPoint>& reference,
        std::vector<double>& offsets,
        int locked_side) const;

    double computeLateralObstacleForce(
        const PathPoint& reference,
        double offset) const;

    double sampleCost(double x, double y) const;

    void rebuildDistanceField();

    double getObstacleDistance(double x, double y) const;

    double scoreCandidate(const CandidatePath& candidate) const;

    double smoothStep5(double value) const;

    double computeSeedOffset(
        double path_s,
        double collision_start,
        double collision_end,
        int side,
        double peak_offset,
        double path_end) const;

    bool selectAvoidancePath(
        const std::vector<PathPoint>& reference,
        const CollisionResult& collision,
        const ros::Time& now);

    bool selectReturnPath(
        const std::vector<PathPoint>& reference,
        const ros::Time& now);

    bool shouldEnterGoalAlign(
        const geometry_msgs::PoseStamped& robot_pose) const;

    bool computeGoalAlignCommand(
        geometry_msgs::Twist& cmd_vel);

    geometry_msgs::Twist computePurePursuitCommand(
        const geometry_msgs::PoseStamped& robot_pose,
        const std::vector<PathPoint>& path) const;

    geometry_msgs::Twist applyAccelerationLimits(
        const geometry_msgs::Twist& target_cmd,
        const ros::Time& now);

    geometry_msgs::Twist computeSafeStopCommand(
        const ros::Time& now);

    double clampVelocityDelta(
        double target,
        double previous,
        double increase_limit,
        double decrease_limit,
        double dt) const;

    double distanceToGoal(
        const geometry_msgs::PoseStamped& robot_pose) const;

    void publishDebugPaths(
        const std::vector<PathPoint>& reference,
        const std::vector<PathPoint>& selected,
        const ros::Time& stamp);

    void publishPath(
        const ros::Publisher& publisher,
        const std::vector<PathPoint>& path,
        const ros::Time& stamp) const;

    void publishCollisionFootprints(const ros::Time& stamp) const;
    void publishPlannerState() const;
    std::string plannerStateName() const;

    bool initialized_ = false;

    tf2_ros::Buffer* tf_buffer_ = nullptr;
    costmap_2d::Costmap2DROS* costmap_ros_ = nullptr;
    std::unique_ptr<base_local_planner::CostmapModel> world_model_;

    std::vector<geometry_msgs::Point> footprint_;
    double inscribed_radius_ = 0.0;
    double circumscribed_radius_ = 0.0;

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    std::vector<PathPoint> selected_local_path_;
    std::vector<PathPoint> last_collision_poses_;
    std::vector<PathPoint> last_left_candidate_;
    std::vector<PathPoint> last_right_candidate_;

    PlannerState state_ = PlannerState::TRACK;
    int nearest_global_index_ = 0;
    int locked_side_ = 0;
    int blocked_cycles_ = 0;
    bool goal_reached_ = false;

    ros::Time no_path_since_;
    ros::Time state_enter_time_;
    ros::Time path_clear_since_;
    ros::Time previous_control_time_;
    geometry_msgs::Twist previous_cmd_;
    double return_scale_ = 1.0;
    std::vector<double> previous_offsets_;

    std::string base_link_frame_ = "base_link";
    std::string local_frame_;

    double planning_horizon_ = 2.5;
    double collision_horizon_ = 1.2;
    double path_resolution_ = 0.04;
    double collision_check_step_ = 0.025;
    double collision_yaw_step_ = 0.07;
    double transform_timeout_ = 0.10;

    double lookahead_min_ = 0.20;
    double lookahead_max_ = 0.65;
    double lookahead_time_ = 0.80;

    double max_vel_x_ = 0.15;
    double max_vel_theta_ = 0.80;
    double acc_lim_x_ = 0.45;
    double dec_lim_x_ = 0.80;
    double acc_lim_theta_ = 1.50;
    double dec_lim_theta_ = 2.00;
    double max_lateral_acceleration_ = 0.50;

    double final_xy_tolerance_ = 0.05;
    double final_yaw_tolerance_ = 0.10;
    double final_yaw_gain_ = 2.0;
    double final_yaw_max_vel_ = 0.80;

    double no_path_grace_time_ = 0.50;
    double no_path_timeout_ = 1.00;

    double offset_step_ = 0.05;
    double max_lateral_offset_ = 0.50;
    double shift_in_distance_ = 0.60;
    double shift_out_distance_ = 0.80;
    double obstacle_pass_margin_ = 0.35;
    double desired_clearance_ = 0.18;
    double hard_clearance_ = 0.08;
    double distance_gradient_step_ = 0.05;
    double distance_field_update_period_ = 0.10;
    int optimization_iterations_ = 12;
    double optimization_step_ = 0.05;
    double weight_reference_ = 1.0;
    double weight_smooth_ = 8.0;
    double weight_obstacle_ = 12.0;
    double weight_temporal_ = 5.0;
    double weight_curvature_ = 8.0;
    double clearance_score_weight_ = 0.50;
    double offset_score_weight_ = 1.0;
    double curvature_score_weight_ = 0.50;
    double side_change_penalty_ = 2.0;
    int obstacle_trigger_cycles_ = 2;
    double side_lock_time_ = 1.0;
    double clear_hold_time_ = 0.5;
    double return_time_ = 0.80;

    bool carry_mode_ = false;
    double carry_speed_scale_ = 0.80;

    ros::Subscriber carry_mode_sub_;
    ros::Publisher reference_path_pub_;
    ros::Publisher left_seed_path_pub_;
    ros::Publisher right_seed_path_pub_;
    ros::Publisher selected_path_pub_;
    ros::Publisher collision_footprints_pub_;
    ros::Publisher planner_state_pub_;

    std::vector<double> obstacle_distance_field_;
    unsigned int distance_field_width_ = 0;
    unsigned int distance_field_height_ = 0;
    double distance_field_resolution_ = 0.0;
    ros::Time distance_field_time_;
};

}  // namespace cym_planner

#endif  // CYM_PLANNER_H_
