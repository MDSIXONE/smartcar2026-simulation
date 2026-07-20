#include "cym_planner.h"

#include <algorithm>
#include <clocale>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>

#include <angles/angles.h>
#include <costmap_2d/cost_values.h>
#include <costmap_2d/footprint.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace
{

constexpr double kEpsilon = 1e-6;

double clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

template <typename T>
void loadPlannerParam(
    const ros::NodeHandle& planner_nh,
    const ros::NodeHandle& legacy_nh,
    const std::string& key,
    T& value,
    const T& default_value)
{
    if (!planner_nh.getParam(key, value) && !legacy_nh.getParam(key, value))
    {
        value = default_value;
    }
}

}  // namespace

namespace cym_planner
{

CymPlanner::CymPlanner()
{
    std::setlocale(LC_ALL, "");
}

CymPlanner::~CymPlanner() = default;

void CymPlanner::initialize(
    std::string name,
    tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    if (initialized_)
    {
        ROS_WARN("cym_planner: initialize called more than once");
        return;
    }
    if (tf == nullptr || costmap_ros == nullptr || costmap_ros->getCostmap() == nullptr)
    {
        ROS_ERROR("cym_planner: initialize received a null TF buffer or costmap");
        return;
    }

    tf_buffer_ = tf;
    costmap_ros_ = costmap_ros;
    local_frame_ = costmap_ros_->getGlobalFrameID();
    world_model_.reset(
        new base_local_planner::CostmapModel(*costmap_ros_->getCostmap()));

    footprint_ = costmap_ros_->getRobotFootprint();
    if (footprint_.empty())
    {
        ROS_ERROR("cym_planner: robot footprint is empty");
        return;
    }
    costmap_2d::calculateMinAndMaxDistances(
        footprint_, inscribed_radius_, circumscribed_radius_);

    ros::NodeHandle planner_nh("~/" + name);
    ros::NodeHandle legacy_nh("~/CymPlanner");

    loadPlannerParam(planner_nh, legacy_nh, "base_link_frame", base_link_frame_, std::string("base_link"));
    loadPlannerParam(planner_nh, legacy_nh, "planning_horizon", planning_horizon_, 2.5);
    loadPlannerParam(planner_nh, legacy_nh, "collision_horizon", collision_horizon_, 1.2);
    loadPlannerParam(planner_nh, legacy_nh, "path_resolution", path_resolution_, 0.04);
    loadPlannerParam(planner_nh, legacy_nh, "collision_check_step", collision_check_step_, 0.025);
    loadPlannerParam(planner_nh, legacy_nh, "collision_yaw_step", collision_yaw_step_, 0.07);
    loadPlannerParam(planner_nh, legacy_nh, "transform_timeout", transform_timeout_, 0.10);

    loadPlannerParam(planner_nh, legacy_nh, "lookahead_min", lookahead_min_, 0.20);
    loadPlannerParam(planner_nh, legacy_nh, "lookahead_max", lookahead_max_, 0.65);
    loadPlannerParam(planner_nh, legacy_nh, "lookahead_time", lookahead_time_, 0.80);
    loadPlannerParam(
        planner_nh, legacy_nh, "tracking_lateral_kp", tracking_lateral_kp_, 1.80);
    loadPlannerParam(
        planner_nh, legacy_nh, "tracking_lateral_kd", tracking_lateral_kd_, 0.12);
    loadPlannerParam(
        planner_nh, legacy_nh, "tracking_heading_kp", tracking_heading_kp_, 1.20);
    loadPlannerParam(
        planner_nh, legacy_nh, "tracking_heading_kd", tracking_heading_kd_, 0.08);

    loadPlannerParam(planner_nh, legacy_nh, "max_vel_x", max_vel_x_, 0.15);
    loadPlannerParam(planner_nh, legacy_nh, "max_vel_theta", max_vel_theta_, 0.80);
    loadPlannerParam(planner_nh, legacy_nh, "acc_lim_x", acc_lim_x_, 0.45);
    loadPlannerParam(planner_nh, legacy_nh, "dec_lim_x", dec_lim_x_, 0.80);
    loadPlannerParam(planner_nh, legacy_nh, "acc_lim_theta", acc_lim_theta_, 1.50);
    loadPlannerParam(planner_nh, legacy_nh, "dec_lim_theta", dec_lim_theta_, 2.00);
    loadPlannerParam(
        planner_nh, legacy_nh, "max_lateral_acceleration", max_lateral_acceleration_, 0.50);

    loadPlannerParam(planner_nh, legacy_nh, "final_xy_tolerance", final_xy_tolerance_, 0.05);
    loadPlannerParam(planner_nh, legacy_nh, "final_yaw_tolerance", final_yaw_tolerance_, 0.10);
    loadPlannerParam(planner_nh, legacy_nh, "final_yaw_gain", final_yaw_gain_, 2.0);
    loadPlannerParam(planner_nh, legacy_nh, "final_yaw_max_vel", final_yaw_max_vel_, 0.80);

    loadPlannerParam(planner_nh, legacy_nh, "no_path_grace_time", no_path_grace_time_, 0.50);
    loadPlannerParam(planner_nh, legacy_nh, "no_path_timeout", no_path_timeout_, 1.00);
    loadPlannerParam(planner_nh, legacy_nh, "carry_speed_scale", carry_speed_scale_, 0.80);

    loadPlannerParam(planner_nh, legacy_nh, "offset_step", offset_step_, 0.05);
    loadPlannerParam(planner_nh, legacy_nh, "max_lateral_offset", max_lateral_offset_, 0.50);
    loadPlannerParam(planner_nh, legacy_nh, "shift_in_distance", shift_in_distance_, 0.60);
    loadPlannerParam(planner_nh, legacy_nh, "shift_out_distance", shift_out_distance_, 0.80);
    loadPlannerParam(planner_nh, legacy_nh, "obstacle_pass_margin", obstacle_pass_margin_, 0.35);
    loadPlannerParam(planner_nh, legacy_nh, "desired_clearance", desired_clearance_, 0.18);
    loadPlannerParam(planner_nh, legacy_nh, "hard_clearance", hard_clearance_, 0.08);
    loadPlannerParam(
        planner_nh, legacy_nh, "distance_gradient_step", distance_gradient_step_, 0.05);
    loadPlannerParam(
        planner_nh,
        legacy_nh,
        "distance_field_update_period",
        distance_field_update_period_,
        0.10);
    loadPlannerParam(
        planner_nh, legacy_nh, "optimization_iterations", optimization_iterations_, 12);
    loadPlannerParam(planner_nh, legacy_nh, "optimization_step", optimization_step_, 0.05);
    loadPlannerParam(planner_nh, legacy_nh, "weight_reference", weight_reference_, 1.0);
    loadPlannerParam(planner_nh, legacy_nh, "weight_smooth", weight_smooth_, 8.0);
    loadPlannerParam(planner_nh, legacy_nh, "weight_obstacle", weight_obstacle_, 12.0);
    loadPlannerParam(planner_nh, legacy_nh, "weight_temporal", weight_temporal_, 5.0);
    loadPlannerParam(planner_nh, legacy_nh, "weight_curvature", weight_curvature_, 8.0);
    loadPlannerParam(
        planner_nh, legacy_nh, "weight_side_cost", weight_side_cost_, 1.0);
    loadPlannerParam(
        planner_nh, legacy_nh, "weight_side_clearance", weight_side_clearance_, 0.8);
    loadPlannerParam(
        planner_nh, legacy_nh, "weight_side_distance", weight_side_distance_, 0.25);
    loadPlannerParam(
        planner_nh, legacy_nh, "offset_score_weight", offset_score_weight_, 1.0);
    loadPlannerParam(
        planner_nh, legacy_nh, "curvature_score_weight", curvature_score_weight_, 0.50);
    loadPlannerParam(
        planner_nh, legacy_nh, "clearance_score_weight", clearance_score_weight_, 0.50);
    loadPlannerParam(planner_nh, legacy_nh, "side_change_penalty", side_change_penalty_, 2.0);
    loadPlannerParam(
        planner_nh, legacy_nh, "obstacle_trigger_cycles", obstacle_trigger_cycles_, 2);
    loadPlannerParam(planner_nh, legacy_nh, "side_lock_time", side_lock_time_, 1.0);
    loadPlannerParam(planner_nh, legacy_nh, "clear_hold_time", clear_hold_time_, 0.5);
    loadPlannerParam(planner_nh, legacy_nh, "return_time", return_time_, 0.80);
    loadPlannerParam(planner_nh, legacy_nh, "escape_hold_time", escape_hold_time_, 0.60);

    planning_horizon_ = std::max(0.50, planning_horizon_);
    collision_horizon_ = clampValue(collision_horizon_, 0.10, planning_horizon_);
    path_resolution_ = std::max(0.01, path_resolution_);
    collision_check_step_ = std::max(0.002, collision_check_step_);
    collision_check_step_ = std::min(
        collision_check_step_,
        std::max(0.002, 0.5 * costmap_ros_->getCostmap()->getResolution()));
    collision_yaw_step_ = clampValue(collision_yaw_step_, 0.01, 0.35);
    transform_timeout_ = clampValue(transform_timeout_, 0.01, 1.0);

    lookahead_min_ = std::max(path_resolution_, lookahead_min_);
    lookahead_max_ = std::max(lookahead_min_, lookahead_max_);
    lookahead_time_ = std::max(0.0, lookahead_time_);
    tracking_lateral_kp_ = std::max(0.0, tracking_lateral_kp_);
    tracking_lateral_kd_ = std::max(0.0, tracking_lateral_kd_);
    tracking_heading_kp_ = std::max(0.0, tracking_heading_kp_);
    tracking_heading_kd_ = std::max(0.0, tracking_heading_kd_);

    max_vel_x_ = std::max(0.0, max_vel_x_);
    max_vel_theta_ = std::max(0.0, max_vel_theta_);
    acc_lim_x_ = std::max(0.01, acc_lim_x_);
    dec_lim_x_ = std::max(0.01, dec_lim_x_);
    acc_lim_theta_ = std::max(0.01, acc_lim_theta_);
    dec_lim_theta_ = std::max(0.01, dec_lim_theta_);
    max_lateral_acceleration_ = std::max(0.01, max_lateral_acceleration_);

    final_xy_tolerance_ = std::max(0.01, final_xy_tolerance_);
    final_yaw_tolerance_ = clampValue(final_yaw_tolerance_, 0.01, M_PI);
    final_yaw_gain_ = std::max(0.0, final_yaw_gain_);
    final_yaw_max_vel_ = std::max(0.0, final_yaw_max_vel_);
    no_path_grace_time_ = std::max(0.0, no_path_grace_time_);
    no_path_timeout_ = std::max(no_path_grace_time_, no_path_timeout_);
    carry_speed_scale_ = clampValue(carry_speed_scale_, 0.05, 1.0);

    offset_step_ = clampValue(offset_step_, 0.01, 0.25);
    max_lateral_offset_ = std::max(offset_step_, max_lateral_offset_);
    shift_in_distance_ = std::max(0.0, shift_in_distance_);
    shift_out_distance_ = std::max(0.05, shift_out_distance_);
    obstacle_pass_margin_ = std::max(0.0, obstacle_pass_margin_);
    desired_clearance_ = std::max(0.0, desired_clearance_);
    hard_clearance_ = clampValue(hard_clearance_, 0.0, desired_clearance_);
    distance_gradient_step_ = std::max(0.01, distance_gradient_step_);
    distance_field_update_period_ = std::max(0.02, distance_field_update_period_);
    optimization_iterations_ = std::max(0, optimization_iterations_);
    optimization_step_ = clampValue(optimization_step_, 0.001, 0.50);
    weight_reference_ = std::max(0.0, weight_reference_);
    weight_smooth_ = std::max(0.0, weight_smooth_);
    weight_obstacle_ = std::max(0.0, weight_obstacle_);
    weight_temporal_ = std::max(0.0, weight_temporal_);
    weight_curvature_ = std::max(0.0, weight_curvature_);
    weight_side_cost_ = std::max(0.0, weight_side_cost_);
    weight_side_clearance_ = std::max(0.0, weight_side_clearance_);
    weight_side_distance_ = std::max(0.0, weight_side_distance_);
    offset_score_weight_ = std::max(0.0, offset_score_weight_);
    curvature_score_weight_ = std::max(0.0, curvature_score_weight_);
    clearance_score_weight_ = std::max(0.0, clearance_score_weight_);
    side_change_penalty_ = std::max(0.0, side_change_penalty_);
    obstacle_trigger_cycles_ = std::max(1, obstacle_trigger_cycles_);
    side_lock_time_ = std::max(0.0, side_lock_time_);
    clear_hold_time_ = std::max(0.0, clear_hold_time_);
    return_time_ = std::max(0.05, return_time_);
    escape_hold_time_ = clampValue(escape_hold_time_, 0.20, 1.50);

    ros::NodeHandle public_nh;
    carry_mode_sub_ = public_nh.subscribe(
        "/sim_task3/carry_mode", 1, &CymPlanner::carryModeCallback, this);

    ros::NodeHandle debug_nh("/cym_planner");
    reference_path_pub_ = debug_nh.advertise<nav_msgs::Path>("reference_path", 1);
    left_seed_path_pub_ = debug_nh.advertise<nav_msgs::Path>("left_seed_path", 1);
    right_seed_path_pub_ = debug_nh.advertise<nav_msgs::Path>("right_seed_path", 1);
    selected_path_pub_ = debug_nh.advertise<nav_msgs::Path>("selected_path", 1);
    collision_footprints_pub_ =
        debug_nh.advertise<visualization_msgs::MarkerArray>("predicted_footprints", 1);
    planner_state_pub_ = debug_nh.advertise<std_msgs::String>("planner_state", 1, true);

    previous_cmd_ = geometry_msgs::Twist();
    previous_control_time_ = ros::Time(0);
    distance_field_time_ = ros::Time(0);
    no_path_since_ = ros::Time(0);
    state_enter_time_ = ros::Time::now();
    path_clear_since_ = ros::Time(0);
    state_ = PlannerState::TRACK;
    initialized_ = true;

    ROS_INFO(
        "cym_planner initialized | frame=%s footprint radii=%.3f/%.3f | horizon=%.2f/%.2f | "
        "resolution=%.3f collision_step=%.3f | lookahead=%.2f..%.2f | max_v/w=%.2f/%.2f",
        local_frame_.c_str(),
        inscribed_radius_,
        circumscribed_radius_,
        planning_horizon_,
        collision_horizon_,
        path_resolution_,
        collision_check_step_,
        lookahead_min_,
        lookahead_max_,
        max_vel_x_,
        max_vel_theta_);
    ROS_INFO(
        "cym_planner elastic lateral force enabled | side cost/clearance/distance weights=%.2f/%.2f/%.2f",
        weight_side_cost_,
        weight_side_clearance_,
        weight_side_distance_);
    ROS_INFO(
        "cym_planner tracking PD | lateral kp/kd=%.2f/%.2f heading kp/kd=%.2f/%.2f | escape hold=%.2fs",
        tracking_lateral_kp_,
        tracking_lateral_kd_,
        tracking_heading_kp_,
        tracking_heading_kd_,
        escape_hold_time_);
}

void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
{
    if (carry_mode_ == message->data)
    {
        return;
    }
    carry_mode_ = message->data;
    ROS_INFO(
        "cym_planner carry mode %s; speed scale %.2f",
        carry_mode_ ? "enabled" : "disabled",
        carry_mode_ ? carry_speed_scale_ : 1.0);
}

bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    if (!initialized_)
    {
        ROS_ERROR("cym_planner: setPlan called before initialize");
        return false;
    }

    global_plan_ = plan;
    selected_local_path_.clear();
    escape_path_.clear();
    last_collision_poses_.clear();
    last_left_candidate_.clear();
    last_right_candidate_.clear();
    previous_offsets_.clear();
    nearest_global_index_ = 0;
    locked_side_ = 0;
    blocked_cycles_ = 0;
    state_ = PlannerState::TRACK;
    state_enter_time_ = ros::Time::now();
    path_clear_since_ = ros::Time(0);
    no_path_since_ = ros::Time(0);
    tracking_lateral_error_ = 0.0;
    tracking_heading_error_ = 0.0;
    tracking_error_time_ = ros::Time(0);
    escape_active_until_ = ros::Time(0);
    escape_target_cmd_ = geometry_msgs::Twist();
    last_escape_direction_ = 0;
    last_escape_was_rotation_ = false;
    return_scale_ = 1.0;
    goal_reached_ = false;
    return !global_plan_.empty();
}

bool CymPlanner::transformPose(
    const geometry_msgs::PoseStamped& input,
    const std::string& target_frame,
    geometry_msgs::PoseStamped& output) const
{
    if (tf_buffer_ == nullptr || target_frame.empty() || input.header.frame_id.empty())
    {
        return false;
    }

    if (input.header.frame_id == target_frame)
    {
        output = input;
        output.header.stamp = ros::Time::now();
        return true;
    }

    geometry_msgs::PoseStamped source = input;
    source.header.stamp = ros::Time(0);
    try
    {
        tf_buffer_->transform(
            source, output, target_frame, ros::Duration(transform_timeout_));
        return true;
    }
    catch (const tf2::TransformException& exception)
    {
        ROS_WARN_THROTTLE(
            1.0,
            "cym_planner: cannot transform pose from %s to %s: %s",
            input.header.frame_id.c_str(),
            target_frame.c_str(),
            exception.what());
        return false;
    }
}

int CymPlanner::findNearestGlobalIndex(
    const geometry_msgs::PoseStamped& robot_pose)
{
    if (global_plan_.empty())
    {
        return -1;
    }

    geometry_msgs::PoseStamped robot_in_plan_frame;
    if (!transformPose(
            robot_pose,
            global_plan_.front().header.frame_id,
            robot_in_plan_frame))
    {
        return -1;
    }

    const int plan_size = static_cast<int>(global_plan_.size());
    // The local planner is strictly forward-only.  Re-acquiring a point from
    // behind the monotonic nearest index makes a corner that was already
    // passed re-enter the local collision horizon and can produce a false
    // COLLISION after the robot is physically clear of it.
    const int begin = std::max(0, std::min(nearest_global_index_, plan_size - 1));
    const int end = std::min(plan_size, begin + 220);

    double best_distance = std::numeric_limits<double>::infinity();
    int best_index = begin;
    for (int index = begin; index < end; ++index)
    {
        const double dx =
            global_plan_[index].pose.position.x - robot_in_plan_frame.pose.position.x;
        const double dy =
            global_plan_[index].pose.position.y - robot_in_plan_frame.pose.position.y;
        const double distance = std::hypot(dx, dy);
        if (distance < best_distance)
        {
            best_distance = distance;
            best_index = index;
        }
    }

    nearest_global_index_ = best_index;
    return best_index;
}

bool CymPlanner::cropGlobalPlan(
    int start_index,
    double horizon,
    std::vector<PathPoint>& cropped_path) const
{
    cropped_path.clear();
    if (start_index < 0 || start_index >= static_cast<int>(global_plan_.size()))
    {
        return false;
    }

    double accumulated = 0.0;
    for (int index = start_index;
         index < static_cast<int>(global_plan_.size());
         ++index)
    {
        geometry_msgs::PoseStamped local_pose;
        if (!transformPose(global_plan_[index], local_frame_, local_pose))
        {
            return false;
        }

        PathPoint point;
        point.x = local_pose.pose.position.x;
        point.y = local_pose.pose.position.y;
        point.yaw = tf2::getYaw(local_pose.pose.orientation);

        if (!cropped_path.empty())
        {
            const double segment_length = std::hypot(
                point.x - cropped_path.back().x,
                point.y - cropped_path.back().y);
            if (segment_length < 1e-4)
            {
                continue;
            }
            accumulated += segment_length;
        }

        point.s = accumulated;
        cropped_path.push_back(point);
        if (accumulated >= horizon)
        {
            break;
        }
    }
    return !cropped_path.empty();
}

std::vector<CymPlanner::PathPoint> CymPlanner::resamplePath(
    const std::vector<PathPoint>& path,
    double resolution) const
{
    if (path.size() < 2 || path.back().s <= kEpsilon)
    {
        return path;
    }

    std::vector<PathPoint> sampled;
    sampled.reserve(
        static_cast<std::size_t>(std::ceil(path.back().s / resolution)) + 2U);
    // GlobalPlanner can occasionally emit a tiny local loop around a cost
    // gradient saddle.  The arc length of that loop is large enough to create
    // several regular samples even though their Cartesian separation is only
    // a few millimetres.  Keeping those samples makes the derived yaw flip by
    // nearly pi radians and produces a false swept-footprint collision while
    // the robot is otherwise following a clear corridor.
    const double minimum_spatial_spacing = std::max(1e-4, 0.5 * resolution);

    std::size_t segment = 0;
    for (double sample_s = 0.0;
         sample_s < path.back().s;
         sample_s += resolution)
    {
        while (segment + 1 < path.size() && path[segment + 1].s < sample_s)
        {
            ++segment;
        }
        if (segment + 1 >= path.size())
        {
            break;
        }

        const PathPoint& first = path[segment];
        const PathPoint& second = path[segment + 1];
        const double segment_length = second.s - first.s;
        const double ratio = segment_length > kEpsilon
            ? clampValue((sample_s - first.s) / segment_length, 0.0, 1.0)
            : 0.0;

        PathPoint point;
        point.x = first.x + ratio * (second.x - first.x);
        point.y = first.y + ratio * (second.y - first.y);
        point.yaw = angles::normalize_angle(
            first.yaw + ratio * angles::shortest_angular_distance(first.yaw, second.yaw));
        point.s = sample_s;
        if (!sampled.empty() &&
            std::hypot(point.x - sampled.back().x, point.y - sampled.back().y) <
                minimum_spatial_spacing)
        {
            continue;
        }
        sampled.push_back(point);
    }

    if (sampled.empty())
    {
        sampled.push_back(path.back());
    }
    else
    {
        const double endpoint_distance = std::hypot(
            sampled.back().x - path.back().x,
            sampled.back().y - path.back().y);
        if (endpoint_distance >= minimum_spatial_spacing)
        {
            sampled.push_back(path.back());
        }
        else if (endpoint_distance > 1e-4)
        {
            // Preserve the exact global-plan endpoint without reintroducing a
            // millimetre-scale terminal segment.
            sampled.back() = path.back();
        }
    }

    computePathGeometry(sampled);
    return sampled;
}

void CymPlanner::computePathGeometry(std::vector<PathPoint>& path) const
{
    if (path.empty())
    {
        return;
    }

    path.front().s = 0.0;
    for (std::size_t index = 1; index < path.size(); ++index)
    {
        path[index].s = path[index - 1].s + std::hypot(
            path[index].x - path[index - 1].x,
            path[index].y - path[index - 1].y);
    }

    if (path.size() == 1)
    {
        path.front().curvature = 0.0;
        return;
    }

    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const std::size_t previous = index == 0 ? 0 : index - 1;
        const std::size_t next = std::min(index + 1, path.size() - 1);
        path[index].yaw = std::atan2(
            path[next].y - path[previous].y,
            path[next].x - path[previous].x);
    }

    path.front().curvature = 0.0;
    path.back().curvature = 0.0;
    for (std::size_t index = 1; index + 1 < path.size(); ++index)
    {
        const double distance = path[index + 1].s - path[index - 1].s;
        path[index].curvature = distance > kEpsilon
            ? angles::shortest_angular_distance(
                  path[index - 1].yaw, path[index + 1].yaw) / distance
            : 0.0;
    }
    if (path.size() > 2)
    {
        path.front().curvature = path[1].curvature;
        path.back().curvature = path[path.size() - 2].curvature;
    }
}

bool CymPlanner::checkPoseCollision(double x, double y, double yaw) const
{
    if (costmap_ros_ == nullptr || world_model_ == nullptr)
    {
        return true;
    }

    unsigned int map_x = 0;
    unsigned int map_y = 0;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (!costmap->worldToMap(x, y, map_x, map_y))
    {
        return true;
    }
    if (costmap->getCost(map_x, map_y) >= costmap_2d::LETHAL_OBSTACLE)
    {
        return true;
    }

    const double footprint_cost = world_model_->footprintCost(
        x,
        y,
        yaw,
        footprint_,
        inscribed_radius_,
        circumscribed_radius_);
    return footprint_cost < 0.0;
}

CymPlanner::CollisionResult CymPlanner::evaluatePathCollision(
    const std::vector<PathPoint>& path,
    double horizon)
{
    CollisionResult result;
    last_collision_poses_.clear();
    if (path.size() < 2)
    {
        result.collision = true;
        return result;
    }

    // A rolling costmap can report a transient footprint overlap at the exact
    // current pose (especially after a recovery rotation).  All lateral
    // candidates share that first pose, so treating it as a hard collision
    // makes every escape candidate invalid.  Allow only a short, bounded
    // escape window; any collision after the robot has left that window is
    // still rejected normally.
    const bool starts_in_collision = checkPoseCollision(
        path.front().x, path.front().y, path.front().yaw);
    bool escaped_start_collision = !starts_in_collision;
    const double start_escape_distance = std::max(
        0.20, circumscribed_radius_ + 2.0 * collision_check_step_);

    for (std::size_t index = 0; index + 1 < path.size(); ++index)
    {
        if (path[index].s > horizon)
        {
            break;
        }

        const double dx = path[index + 1].x - path[index].x;
        const double dy = path[index + 1].y - path[index].y;
        const double distance = std::hypot(dx, dy);
        const double dyaw = angles::shortest_angular_distance(
            path[index].yaw, path[index + 1].yaw);
        // Treat in-place rotation as swept distance as well.  Without this
        // small equivalent arc length, every sample of a pure rotation has
        // s == 0 and the bounded start-overlap escape window can never be
        // exited even when the changed footprint is already clear.
        const double segment_progress = std::max(
            distance, circumscribed_radius_ * std::abs(dyaw));

        const int translation_steps = static_cast<int>(
            std::ceil(distance / collision_check_step_));
        const int rotation_steps = static_cast<int>(
            std::ceil(std::abs(dyaw) / collision_yaw_step_));
        const int steps = std::max(1, std::max(translation_steps, rotation_steps));

        for (int step = 0; step <= steps; ++step)
        {
            const double ratio = static_cast<double>(step) / static_cast<double>(steps);
            const double sample_s = path[index].s + ratio * segment_progress;
            if (sample_s > horizon + collision_check_step_)
            {
                break;
            }

            PathPoint sample;
            sample.x = path[index].x + ratio * dx;
            sample.y = path[index].y + ratio * dy;
            sample.yaw = angles::normalize_angle(path[index].yaw + ratio * dyaw);
            sample.s = sample_s;

            const bool sample_collision = checkPoseCollision(
                sample.x, sample.y, sample.yaw);
            if (!sample_collision)
            {
                if (starts_in_collision && sample_s <= start_escape_distance)
                {
                    escaped_start_collision = true;
                    continue;
                }
                continue;
            }

            if (starts_in_collision && !escaped_start_collision &&
                sample_s <= start_escape_distance)
            {
                continue;
            }

            if (!result.collision)
            {
                result.collision = true;
                result.first_collision_index = static_cast<int>(index);
                result.first_collision_distance = sample_s;
            }
            result.last_collision_index = static_cast<int>(index);

            if (last_collision_poses_.empty() ||
                std::hypot(
                    sample.x - last_collision_poses_.back().x,
                    sample.y - last_collision_poses_.back().y) >= 0.05 ||
                std::abs(angles::shortest_angular_distance(
                    sample.yaw, last_collision_poses_.back().yaw)) >= 0.10)
            {
                last_collision_poses_.push_back(sample);
            }
        }
    }

    if (starts_in_collision && !escaped_start_collision)
    {
        result.collision = true;
        result.first_collision_index = 0;
        result.first_collision_distance = 0.0;
        result.last_collision_index = 0;
    }
    return result;
}

double CymPlanner::smoothStep5(double value) const
{
    const double u = clampValue(value, 0.0, 1.0);
    return 10.0 * std::pow(u, 3) - 15.0 * std::pow(u, 4) + 6.0 * std::pow(u, 5);
}

double CymPlanner::computeSeedOffset(
    double path_s,
    double collision_start,
    double collision_end,
    int side,
    double peak_offset,
    double path_end) const
{
    if (side == 0 || peak_offset <= 0.0 || collision_end < collision_start)
    {
        return 0.0;
    }

    const double shift_start = std::max(0.0, collision_start - shift_in_distance_);
    const double hold_end = collision_end + obstacle_pass_margin_;
    const double return_end = hold_end + shift_out_distance_;
    const double sign = side > 0 ? 1.0 : -1.0;

    if (path_s <= shift_start)
    {
        return 0.0;
    }
    if (path_s < collision_start && collision_start > shift_start + kEpsilon)
    {
        return sign * peak_offset * smoothStep5(
            (path_s - shift_start) / (collision_start - shift_start));
    }
    if (path_s <= hold_end)
    {
        return sign * peak_offset;
    }
    if (return_end <= path_end + kEpsilon && path_s < return_end)
    {
        return sign * peak_offset * (1.0 - smoothStep5(
            (path_s - hold_end) / (return_end - hold_end)));
    }
    if (return_end > path_end + kEpsilon)
    {
        return sign * peak_offset;
    }
    return 0.0;
}

CymPlanner::CandidatePath CymPlanner::generatePathFromOffsets(
    const std::vector<PathPoint>& reference,
    const std::vector<double>& offsets,
    int side) const
{
    CandidatePath candidate;
    candidate.side = side;
    candidate.points = reference;
    if (reference.empty())
    {
        return candidate;
    }

    double peak_offset = 0.0;
    for (std::size_t index = 0; index < reference.size(); ++index)
    {
        double offset = index < offsets.size() ? offsets[index] : 0.0;
        if (side > 0)
        {
            offset = std::max(0.0, offset);
        }
        else if (side < 0)
        {
            offset = std::min(0.0, offset);
        }
        offset = clampValue(offset, -max_lateral_offset_, max_lateral_offset_);

        const double nx = -std::sin(reference[index].yaw);
        const double ny = std::cos(reference[index].yaw);
        candidate.points[index].x = reference[index].x + offset * nx;
        candidate.points[index].y = reference[index].y + offset * ny;
        candidate.points[index].offset = offset;
        peak_offset = std::max(peak_offset, std::abs(offset));
    }

    const double first_yaw = reference.front().yaw;
    computePathGeometry(candidate.points);
    candidate.points.front().yaw = first_yaw;
    candidate.peak_offset = peak_offset;
    candidate.minimum_clearance = std::numeric_limits<double>::infinity();
    for (const PathPoint& point : candidate.points)
    {
        candidate.minimum_clearance = std::min(
            candidate.minimum_clearance,
            getObstacleDistance(point.x, point.y));
    }
    return candidate;
}

CymPlanner::CandidatePath CymPlanner::generateSeedCandidate(
    const std::vector<PathPoint>& reference,
    const CollisionResult& collision,
    int side,
    double peak_offset) const
{
    CandidatePath candidate;
    candidate.side = side;
    if (reference.empty() || !collision.collision || side == 0)
    {
        return candidate;
    }

    const double collision_start = collision.first_collision_distance;
    const int last_index = std::max(
        0,
        std::min(
            collision.last_collision_index,
            static_cast<int>(reference.size()) - 1));
    const double collision_end = std::max(
        collision_start,
        reference[static_cast<std::size_t>(last_index)].s);

    std::vector<double> offsets(reference.size(), 0.0);
    for (std::size_t index = 0; index < reference.size(); ++index)
    {
        offsets[index] = computeSeedOffset(
            reference[index].s,
            collision_start,
            collision_end,
            side,
            peak_offset,
            reference.back().s);
    }
    return generatePathFromOffsets(reference, offsets, side);
}

CymPlanner::CandidatePath CymPlanner::findFirstFeasibleCandidate(
    const std::vector<PathPoint>& reference,
    const CollisionResult& collision,
    int side)
{
    CandidatePath best;
    if (!collision.collision || side == 0)
    {
        return best;
    }

    for (double magnitude = offset_step_;
         magnitude <= max_lateral_offset_ + kEpsilon;
         magnitude += offset_step_)
    {
        CandidatePath candidate = generateSeedCandidate(
            reference, collision, side, std::min(magnitude, max_lateral_offset_));
        const CollisionResult candidate_collision =
            evaluatePathCollision(candidate.points, collision_horizon_);
        if (candidate_collision.collision)
        {
            continue;
        }

        candidate.valid = true;
        candidate.score = scoreCandidate(candidate);
        return candidate;
    }
    return best;
}

double CymPlanner::sampleCost(double x, double y) const
{
    if (costmap_ros_ == nullptr)
    {
        return 255.0;
    }
    unsigned int map_x = 0;
    unsigned int map_y = 0;
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    if (!costmap->worldToMap(x, y, map_x, map_y))
    {
        return 255.0;
    }
    return static_cast<double>(costmap->getCost(map_x, map_y));
}

void CymPlanner::rebuildDistanceField()
{
    if (costmap_ros_ == nullptr || costmap_ros_->getCostmap() == nullptr)
    {
        obstacle_distance_field_.clear();
        distance_field_width_ = 0;
        distance_field_height_ = 0;
        return;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    distance_field_width_ = costmap->getSizeInCellsX();
    distance_field_height_ = costmap->getSizeInCellsY();
    distance_field_resolution_ = costmap->getResolution();
    const std::size_t cell_count = static_cast<std::size_t>(
        distance_field_width_) * static_cast<std::size_t>(distance_field_height_);
    obstacle_distance_field_.assign(
        cell_count, std::numeric_limits<double>::infinity());

    using QueueEntry = std::pair<double, std::size_t>;
    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        std::greater<QueueEntry>> queue;

    for (unsigned int y = 0; y < distance_field_height_; ++y)
    {
        for (unsigned int x = 0; x < distance_field_width_; ++x)
        {
            const std::size_t index = static_cast<std::size_t>(y) *
                distance_field_width_ + x;
            if (costmap->getCost(x, y) >= costmap_2d::LETHAL_OBSTACLE)
            {
                obstacle_distance_field_[index] = 0.0;
                queue.emplace(0.0, index);
            }
        }
    }

    static const int directions[8][2] = {
        {-1, 0}, {1, 0}, {0, -1}, {0, 1},
        {-1, -1}, {-1, 1}, {1, -1}, {1, 1}};
    while (!queue.empty())
    {
        const QueueEntry current = queue.top();
        queue.pop();
        if (current.first > obstacle_distance_field_[current.second] + kEpsilon)
        {
            continue;
        }

        const unsigned int x = static_cast<unsigned int>(
            current.second % distance_field_width_);
        const unsigned int y = static_cast<unsigned int>(
            current.second / distance_field_width_);
        for (const auto& direction : directions)
        {
            const int neighbor_x = static_cast<int>(x) + direction[0];
            const int neighbor_y = static_cast<int>(y) + direction[1];
            if (neighbor_x < 0 || neighbor_y < 0 ||
                neighbor_x >= static_cast<int>(distance_field_width_) ||
                neighbor_y >= static_cast<int>(distance_field_height_))
            {
                continue;
            }

            const std::size_t neighbor_index = static_cast<std::size_t>(neighbor_y) *
                distance_field_width_ + static_cast<unsigned int>(neighbor_x);
            const double step_distance = distance_field_resolution_ *
                ((direction[0] != 0 && direction[1] != 0) ? std::sqrt(2.0) : 1.0);
            const double candidate_distance = current.first + step_distance;
            if (candidate_distance + kEpsilon < obstacle_distance_field_[neighbor_index])
            {
                obstacle_distance_field_[neighbor_index] = candidate_distance;
                queue.emplace(candidate_distance, neighbor_index);
            }
        }
    }
}

double CymPlanner::getObstacleDistance(double x, double y) const
{
    if (costmap_ros_ == nullptr || obstacle_distance_field_.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    unsigned int map_x = 0;
    unsigned int map_y = 0;
    if (!costmap_ros_->getCostmap()->worldToMap(x, y, map_x, map_y))
    {
        return 0.0;
    }
    const std::size_t index = static_cast<std::size_t>(map_y) *
        distance_field_width_ + map_x;
    if (index >= obstacle_distance_field_.size())
    {
        return 0.0;
    }
    return obstacle_distance_field_[index];
}

double CymPlanner::computeLateralObstacleForce(
    const PathPoint& reference,
    double offset) const
{
    const double nx = -std::sin(reference.yaw);
    const double ny = std::cos(reference.yaw);
    const double x = reference.x + offset * nx;
    const double y = reference.y + offset * ny;

    // Probe both sides of the elastic-band node.  Positive force means
    // "move left" (the +normal direction), negative means "move right".
    // Keeping both samples in the same force calculation is important: the
    // optimizer should move toward the cheaper side, not commit to a binary
    // left/right branch before the band has converged.
    const double probe = std::max(distance_gradient_step_, 0.5 * path_resolution_);
    const double left_x = x + probe * nx;
    const double left_y = y + probe * ny;
    const double right_x = x - probe * nx;
    const double right_y = y - probe * ny;

    const double left_cost = sampleCost(left_x, left_y);
    const double right_cost = sampleCost(right_x, right_y);
    const double left_distance = getObstacleDistance(left_x, left_y);
    const double right_distance = getObstacleDistance(right_x, right_y);
    const double current_distance = getObstacleDistance(x, y);

    const double cost_force = clampValue(
        (right_cost - left_cost) / 255.0, -1.0, 1.0);

    auto clearancePenalty = [this](double distance) {
        if (!std::isfinite(distance))
        {
            return 0.0;
        }
        const double scale = std::max(desired_clearance_, 0.01);
        return clampValue((desired_clearance_ - distance) / scale, 0.0, 1.0);
    };
    // A side with the larger penalty is more expensive, so the difference is
    // deliberately right-minus-left to preserve the force sign convention.
    const double clearance_force = clampValue(
        clearancePenalty(right_distance) - clearancePenalty(left_distance),
        -1.0,
        1.0);

    double distance_force = 0.0;
    if (std::isfinite(left_distance) && std::isfinite(right_distance))
    {
        // Larger clearance on the left produces a positive force.  Normalize
        // by the probe spacing so this term remains useful outside the
        // inflated costmap plateau.
        distance_force = clampValue(
            (left_distance - right_distance) /
                std::max(2.0 * probe, kEpsilon),
            -1.0,
            1.0);
    }

    const double total_weight = weight_side_cost_ +
        weight_side_clearance_ + weight_side_distance_;
    double force = 0.0;
    if (total_weight > kEpsilon)
    {
        force = (
            weight_side_cost_ * cost_force +
            weight_side_clearance_ * clearance_force +
            weight_side_distance_ * distance_force) / total_weight;
    }

    // When the node itself is already inside the hard clearance band, retain
    // a small amount of repulsion instead of allowing the reference/smoothness
    // terms to pull it back through the obstacle.  The direction still comes
    // from the relative cost of both sides.
    if (current_distance < hard_clearance_)
    {
        force *= 1.5;
    }
    return clampValue(force, -1.0, 1.0);
}

void CymPlanner::refineOffsets(
    const std::vector<PathPoint>& reference,
    std::vector<double>& offsets,
    int /*locked_side*/) const
{
    if (reference.size() < 5 || offsets.size() != reference.size())
    {
        return;
    }

    std::vector<double> next = offsets;
    for (int iteration = 0; iteration < optimization_iterations_; ++iteration)
    {
        for (std::size_t index = 2; index + 2 < offsets.size(); ++index)
        {
            const double smooth_force =
                offsets[index - 1] + offsets[index + 1] - 2.0 * offsets[index];
            const double reference_force = -offsets[index];
            const double previous_offset = index < previous_offsets_.size()
                ? previous_offsets_[index]
                : offsets[index];
            const double temporal_force = previous_offset - offsets[index];
            const double obstacle_force = computeLateralObstacleForce(
                reference[index], offsets[index]);

            double update =
                weight_smooth_ * smooth_force +
                weight_reference_ * reference_force +
                weight_temporal_ * temporal_force +
                weight_obstacle_ * obstacle_force;

            // Curvature is evaluated after rebuilding the path.  This local
            // first-order term keeps large changes from accumulating.
            update -= weight_curvature_ *
                (offsets[index] - 0.5 * (offsets[index - 1] + offsets[index + 1]));

            next[index] = clampValue(
                offsets[index] + optimization_step_ * update,
                -max_lateral_offset_,
                max_lateral_offset_);
        }
        next.front() = 0.0;
        next.back() = 0.0;
        offsets.swap(next);
    }
}

double CymPlanner::scoreCandidate(const CandidatePath& candidate) const
{
    if (!candidate.valid || candidate.points.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    double score = offset_score_weight_ * std::abs(candidate.peak_offset);
    if (std::isfinite(candidate.minimum_clearance))
    {
        score += clearance_score_weight_ /
            std::max(candidate.minimum_clearance, 0.01);
    }
    for (std::size_t index = 1; index < candidate.points.size(); ++index)
    {
        const double curvature = candidate.points[index].curvature;
        score += curvature_score_weight_ * curvature * curvature;
        if (index < previous_offsets_.size())
        {
            score += weight_temporal_ * std::abs(
                candidate.points[index].offset - previous_offsets_[index]);
        }
    }
    if (locked_side_ != 0 && candidate.side != locked_side_)
    {
        score += side_change_penalty_;
    }
    return score;
}

CymPlanner::CandidatePath CymPlanner::generateReturnCandidate(
    const std::vector<PathPoint>& reference,
    int side)
{
    CandidatePath candidate;
    candidate.side = side;
    if (reference.empty() || previous_offsets_.empty())
    {
        candidate.points = reference;
        candidate.valid = !reference.empty();
        return candidate;
    }

    std::vector<double> offsets(reference.size(), 0.0);
    for (std::size_t index = 0; index < reference.size(); ++index)
    {
        const double normalized = reference.size() > 1
            ? static_cast<double>(index) / static_cast<double>(reference.size() - 1)
            : 0.0;
        const double old_position = normalized *
            static_cast<double>(previous_offsets_.size() - 1);
        const std::size_t first = static_cast<std::size_t>(old_position);
        const std::size_t second = std::min(first + 1, previous_offsets_.size() - 1);
        const double ratio = old_position - static_cast<double>(first);
        offsets[index] = return_scale_ * (
            previous_offsets_[first] + ratio *
            (previous_offsets_[second] - previous_offsets_[first]));
    }
    return generatePathFromOffsets(reference, offsets, side);
}

bool CymPlanner::selectAvoidancePath(
    const std::vector<PathPoint>& reference,
    const CollisionResult& collision,
    const ros::Time& now)
{
    last_left_candidate_.clear();
    last_right_candidate_.clear();

    CandidatePath left;
    CandidatePath right;
    if (locked_side_ == 0)
    {
        left = findFirstFeasibleCandidate(reference, collision, 1);
        right = findFirstFeasibleCandidate(reference, collision, -1);
    }
    else
    {
        const double current_peak = previous_offsets_.empty()
            ? offset_step_
            : *std::max_element(
                  previous_offsets_.begin(), previous_offsets_.end(),
                  [](double first, double second) {
                      return std::abs(first) < std::abs(second);
                  });
        CandidatePath locked_candidate = generateSeedCandidate(
            reference,
            collision,
            locked_side_,
            std::max(offset_step_, std::abs(current_peak)));
        if (!locked_candidate.points.empty())
        {
            const CollisionResult locked_collision =
                evaluatePathCollision(locked_candidate.points, collision_horizon_);
            if (!locked_collision.collision)
            {
                locked_candidate.valid = true;
                locked_candidate.score = scoreCandidate(locked_candidate);
                if (locked_side_ > 0)
                {
                    left = locked_candidate;
                }
                else
                {
                    right = locked_candidate;
                }
            }
        }

        const double locked_duration = (now - state_enter_time_).toSec();
        if (!left.valid && !right.valid && locked_duration >= side_lock_time_)
        {
            const int other_side = -locked_side_;
            CandidatePath alternative = findFirstFeasibleCandidate(
                reference, collision, other_side);
            if (other_side > 0)
            {
                left = alternative;
            }
            else
            {
                right = alternative;
            }
        }
    }

    if (!left.points.empty())
    {
        last_left_candidate_ = left.points;
    }
    if (!right.points.empty())
    {
        last_right_candidate_ = right.points;
    }

    CandidatePath* chosen = nullptr;
    if (left.valid && (!right.valid || left.score <= right.score))
    {
        chosen = &left;
    }
    else if (right.valid)
    {
        chosen = &right;
    }
    if (chosen == nullptr)
    {
        return false;
    }

    std::vector<double> offsets;
    offsets.reserve(chosen->points.size());
    for (const PathPoint& point : chosen->points)
    {
        offsets.push_back(point.offset);
    }
    // The collision-free left/right candidate is only a safe initialization.
    // From here on the elastic band is unconstrained in sign: every node can
    // move continuously according to the cost difference between its two
    // lateral probes.  This allows a path to bend through a non-symmetric
    // corridor instead of remaining locked to one side of the seed.
    refineOffsets(reference, offsets, 0);
    CandidatePath optimized = generatePathFromOffsets(reference, offsets, 0);
    const CollisionResult optimized_collision =
        evaluatePathCollision(optimized.points, collision_horizon_);
    if (!optimized_collision.collision)
    {
        double signed_offset = 0.0;
        double signed_weight = 0.0;
        for (const PathPoint& point : optimized.points)
        {
            const double weight = std::max(point.s, path_resolution_);
            signed_offset += weight * point.offset;
            signed_weight += weight;
        }
        const double mean_offset = signed_weight > kEpsilon
            ? signed_offset / signed_weight
            : 0.0;
        // Keep side_lock as hysteresis metadata only.  It no longer constrains
        // the elastic optimizer, but it prevents rapid state toggling when a
        // converged band is nearly centered.
        optimized.side = std::abs(mean_offset) > 0.5 * offset_step_
            ? (mean_offset > 0.0 ? 1 : -1)
            : chosen->side;
        optimized.valid = true;
        optimized.score = scoreCandidate(optimized);
        chosen = &optimized;
    }

    locked_side_ = chosen->side;
    state_ = chosen->side > 0
        ? PlannerState::AVOID_LEFT
        : PlannerState::AVOID_RIGHT;
    state_enter_time_ = now;
    return_scale_ = 1.0;
    selected_local_path_ = chosen->points;
    previous_offsets_.clear();
    for (const PathPoint& point : selected_local_path_)
    {
        previous_offsets_.push_back(point.offset);
    }
    return true;
}

bool CymPlanner::selectReturnPath(
    const std::vector<PathPoint>& reference,
    const ros::Time& now)
{
    if (locked_side_ == 0)
    {
        selected_local_path_ = reference;
        return true;
    }

    if (path_clear_since_.isZero())
    {
        path_clear_since_ = now;
        return false;
    }
    if ((now - path_clear_since_).toSec() < clear_hold_time_)
    {
        return false;
    }

    double dt = 0.05;
    if (!previous_control_time_.isZero())
    {
        dt = clampValue((now - previous_control_time_).toSec(), 0.005, 0.20);
    }
    return_scale_ *= std::exp(-dt / return_time_);
    CandidatePath returning = generateReturnCandidate(reference, locked_side_);
    const CollisionResult collision = evaluatePathCollision(
        returning.points, collision_horizon_);
    if (collision.collision)
    {
        path_clear_since_ = ros::Time(0);
        return false;
    }

    returning.valid = true;
    selected_local_path_ = returning.points;
    previous_offsets_.clear();
    for (const PathPoint& point : selected_local_path_)
    {
        previous_offsets_.push_back(point.offset);
    }

    if (return_scale_ < 0.05)
    {
        locked_side_ = 0;
        state_ = PlannerState::TRACK;
        state_enter_time_ = now;
        path_clear_since_ = ros::Time(0);
        previous_offsets_.clear();
        selected_local_path_ = reference;
    }
    return true;
}

bool CymPlanner::shouldEnterGoalAlign(
    const geometry_msgs::PoseStamped& robot_pose) const
{
    return !global_plan_.empty() && distanceToGoal(robot_pose) <= final_xy_tolerance_;
}

bool CymPlanner::computeGoalAlignCommand(geometry_msgs::Twist& cmd_vel)
{
    geometry_msgs::PoseStamped goal_in_base;
    if (!transformPose(global_plan_.back(), base_link_frame_, goal_in_base))
    {
        cmd_vel = geometry_msgs::Twist();
        return false;
    }

    const double yaw_error = angles::normalize_angle(
        tf2::getYaw(goal_in_base.pose.orientation));
    if (std::abs(yaw_error) <= final_yaw_tolerance_)
    {
        goal_reached_ = true;
        previous_cmd_ = geometry_msgs::Twist();
        previous_control_time_ = ros::Time::now();
        cmd_vel = geometry_msgs::Twist();
        ROS_INFO("cym_planner: goal reached");
        return true;
    }

    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    geometry_msgs::Twist target;
    target.angular.z = clampValue(
        final_yaw_gain_ * yaw_error,
        -final_yaw_max_vel_ * motion_scale,
        final_yaw_max_vel_ * motion_scale);
    cmd_vel = applyAccelerationLimits(target, ros::Time::now());
    return true;
}

geometry_msgs::Twist CymPlanner::computePurePursuitCommand(
    const geometry_msgs::PoseStamped& robot_pose,
    const std::vector<PathPoint>& path)
{
    geometry_msgs::Twist command;
    if (path.empty())
    {
        return command;
    }

    std::size_t nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < path.size(); ++index)
    {
        const double distance = std::hypot(
            path[index].x - robot_pose.pose.position.x,
            path[index].y - robot_pose.pose.position.y);
        if (distance < nearest_distance)
        {
            nearest_distance = distance;
            nearest = index;
        }
    }

    const double lookahead = clampValue(
        lookahead_min_ + lookahead_time_ * std::abs(previous_cmd_.linear.x),
        lookahead_min_,
        lookahead_max_);
    const double target_s = path[nearest].s + lookahead;
    std::size_t target_index = nearest;
    while (target_index + 1 < path.size() && path[target_index].s < target_s)
    {
        ++target_index;
    }

    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double dx = path[target_index].x - robot_pose.pose.position.x;
    const double dy = path[target_index].y - robot_pose.pose.position.y;
    const double target_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
    const double target_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
    const double bearing_error = std::atan2(target_y, target_x);
    const double path_heading_error = angles::shortest_angular_distance(
        robot_yaw, path[target_index].yaw);
    const double heading_error = 0.5 * bearing_error +
        0.5 * path_heading_error;

    const double nearest_dx = path[nearest].x - robot_pose.pose.position.x;
    const double nearest_dy = path[nearest].y - robot_pose.pose.position.y;
    const double lateral_error = -std::sin(robot_yaw) * nearest_dx +
        std::cos(robot_yaw) * nearest_dy;
    double lateral_error_rate = 0.0;
    double heading_error_rate = 0.0;
    if (!tracking_error_time_.isZero())
    {
        const double error_dt = clampValue(
            (ros::Time::now() - tracking_error_time_).toSec(), 0.005, 0.20);
        lateral_error_rate = (lateral_error - tracking_lateral_error_) / error_dt;
        heading_error_rate = angles::shortest_angular_distance(
            tracking_heading_error_, heading_error) / error_dt;
    }
    tracking_lateral_error_ = lateral_error;
    tracking_heading_error_ = heading_error;
    tracking_error_time_ = ros::Time::now();

    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    if (target_x <= 0.0 || std::abs(bearing_error) > 1.10)
    {
        // Rotate toward the path when the lookahead is not in front.  Include
        // the lateral term so the vehicle does not wait for a pure heading
        // alignment before correcting a large cross-track error.
        const double pd_angular =
            tracking_lateral_kp_ * lateral_error +
            tracking_lateral_kd_ * lateral_error_rate +
            tracking_heading_kp_ * heading_error +
            tracking_heading_kd_ * heading_error_rate;
        command.angular.z = clampValue(
            pd_angular,
            -max_vel_theta_ * motion_scale,
            max_vel_theta_ * motion_scale);
        return command;
    }

    const double distance_squared = std::max(
        target_x * target_x + target_y * target_y, 1e-4);
    const double curvature = 2.0 * target_y / distance_squared;
    const double curve_speed = std::sqrt(
        max_lateral_acceleration_ / std::max(std::abs(curvature), 1e-3));
    const double remaining_goal_distance = distanceToGoal(robot_pose);
    const double goal_speed = std::isfinite(remaining_goal_distance)
        ? std::sqrt(std::max(0.0, 2.0 * dec_lim_x_ * remaining_goal_distance))
        : max_vel_x_;
    const double heading_scale = clampValue(
        std::cos(std::abs(bearing_error)), 0.35, 1.0);
    const double lateral_scale = clampValue(
        1.0 - 0.75 * std::abs(lateral_error), 0.45, 1.0);

    command.linear.x = std::min(
        max_vel_x_ * motion_scale,
        std::min(curve_speed, goal_speed)) * heading_scale * lateral_scale;
    const double pd_angular =
        tracking_lateral_kp_ * lateral_error +
        tracking_lateral_kd_ * lateral_error_rate +
        tracking_heading_kp_ * heading_error +
        tracking_heading_kd_ * heading_error_rate;
    command.angular.z = clampValue(
        command.linear.x * curvature + pd_angular,
        -max_vel_theta_ * motion_scale,
        max_vel_theta_ * motion_scale);
    return command;
}

double CymPlanner::clampVelocityDelta(
    double target,
    double previous,
    double increase_limit,
    double decrease_limit,
    double dt) const
{
    if (target > previous)
    {
        return std::min(target, previous + increase_limit * dt);
    }
    return std::max(target, previous - decrease_limit * dt);
}

geometry_msgs::Twist CymPlanner::applyAccelerationLimits(
    const geometry_msgs::Twist& target_cmd,
    const ros::Time& now)
{
    double dt = 0.05;
    if (!previous_control_time_.isZero())
    {
        dt = clampValue((now - previous_control_time_).toSec(), 0.005, 0.20);
    }

    geometry_msgs::Twist limited;
    limited.linear.x = clampVelocityDelta(
        target_cmd.linear.x,
        previous_cmd_.linear.x,
        acc_lim_x_,
        dec_lim_x_,
        dt);
    limited.angular.z = clampVelocityDelta(
        target_cmd.angular.z,
        previous_cmd_.angular.z,
        acc_lim_theta_,
        dec_lim_theta_,
        dt);

    previous_cmd_ = limited;
    previous_control_time_ = now;
    return limited;
}

geometry_msgs::Twist CymPlanner::computeSafeStopCommand(const ros::Time& now)
{
    return applyAccelerationLimits(geometry_msgs::Twist(), now);
}

bool CymPlanner::computeEscapeCommand(
    const geometry_msgs::PoseStamped& robot_pose,
    const ros::Time& now,
    geometry_msgs::Twist& cmd_vel)
{
    escape_path_.clear();
    const double yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double probe_distance = std::max(
        0.45, circumscribed_radius_ + 0.25);
    const double sample_step = std::max(0.025, collision_check_step_);
    const double probe_speed = 0.12;
    const double rotation_duration = 1.20;

    // At a tight inner corner the robot's rear usually points back into the
    // open corridor.  Do not, however, issue the same reverse command on
    // every cycle: after an escape, try the opposite translation first so a
    // blocked robot cannot oscillate forward/backward at one pose.
    struct EscapeArc
    {
        double direction;
        double angular;
    };
    std::vector<EscapeArc> arcs;
    if (last_escape_was_rotation_ && last_escape_direction_ < 0)
    {
        // A rotation is followed by the opposite translation direction.  This
        // gives the chassis a chance to change its heading before translating
        // back through the same corner.
        arcs = {
            {1.0, 0.0}, {1.0, 0.45}, {1.0, -0.45},
            {-1.0, 0.45}, {-1.0, -0.45}, {-1.0, 0.0},
            {0.0, 0.70}, {0.0, -0.70},
        };
    }
    else if (last_escape_was_rotation_ && last_escape_direction_ > 0)
    {
        arcs = {
            {-1.0, 0.0}, {-1.0, 0.45}, {-1.0, -0.45},
            {1.0, 0.45}, {1.0, -0.45}, {1.0, 0.0},
            {0.0, 0.70}, {0.0, -0.70},
        };
    }
    else if (last_escape_was_rotation_)
    {
        arcs = {
            {-1.0, 0.0}, {-1.0, 0.45}, {-1.0, -0.45},
            {1.0, 0.0}, {1.0, 0.45}, {1.0, -0.45},
            {0.0, 0.70}, {0.0, -0.70},
        };
    }
    else if (last_escape_direction_ < 0)
    {
        arcs = {
            {0.0, 0.70}, {0.0, -0.70},
            {1.0, 0.0}, {1.0, 0.45}, {1.0, -0.45},
            {-1.0, 0.45}, {-1.0, -0.45}, {-1.0, 0.0},
        };
    }
    else if (last_escape_direction_ > 0)
    {
        arcs = {
            {0.0, 0.70}, {0.0, -0.70},
            {-1.0, 0.0}, {-1.0, 0.45}, {-1.0, -0.45},
            {1.0, 0.45}, {1.0, -0.45}, {1.0, 0.0},
        };
    }
    else
    {
        arcs = {
            {-1.0, 0.0}, {-1.0, 0.45}, {-1.0, -0.45},
            {1.0, 0.0}, {1.0, 0.45}, {1.0, -0.45},
            {0.0, 0.70}, {0.0, -0.70},
        };
    }

    for (const EscapeArc& arc : arcs)
    {
        std::vector<PathPoint> path;
        path.reserve(static_cast<std::size_t>(probe_distance / sample_step) + 2U);

        PathPoint start;
        start.x = robot_pose.pose.position.x;
        start.y = robot_pose.pose.position.y;
        start.yaw = yaw;
        start.s = 0.0;
        path.push_back(start);

        double x = start.x;
        double y = start.y;
        double heading = yaw;
        double travelled = 0.0;
        const double signed_speed = arc.direction * probe_speed;
        const int steps = arc.direction == 0.0
            ? static_cast<int>(std::ceil(rotation_duration / 0.05))
            : static_cast<int>(std::ceil(probe_distance / sample_step));
        for (int step = 1; step <= steps; ++step)
        {
            double next_heading = heading;
            if (arc.direction == 0.0)
            {
                const double dt = 0.05;
                next_heading = angles::normalize_angle(
                    heading + arc.angular * dt);
                travelled += circumscribed_radius_ *
                    std::abs(angles::shortest_angular_distance(
                        heading, next_heading));
            }
            else
            {
                const double ds = std::min(
                    sample_step, probe_distance - travelled);
                if (ds <= 0.0)
                {
                    break;
                }
                next_heading = angles::normalize_angle(
                    heading + arc.angular * (ds / probe_speed));
                const double midpoint_heading = angles::normalize_angle(
                    0.5 * (heading + next_heading));
                x += arc.direction * ds * std::cos(midpoint_heading);
                y += arc.direction * ds * std::sin(midpoint_heading);
                travelled += ds;
            }

            PathPoint point;
            point.x = x;
            point.y = y;
            point.yaw = next_heading;
            point.s = travelled;
            path.push_back(point);
            heading = next_heading;
        }

        // Escape arcs are allowed to start in the inflated edge of a wall.
        // The live costmap can disagree by one cell between the reference
        // sweep and this probe: at one cycle the current pose is reported
        // clear, while the first 2--3 cm of a reverse arc are reported
        // lethal.  Running the normal path evaluator here rejects exactly
        // the recovery arc that would leave the corner.  For this bounded
        // probe, allow a short near-start overlap, require the footprint to
        // become clear, and reject every collision after that recovery.
        const double start_escape_distance = std::max(
            0.20, circumscribed_radius_ + 2.0 * collision_check_step_);
        bool near_start_collision = false;
        bool recovered = false;
        bool arc_valid = true;
        for (const PathPoint& sample : path)
        {
            const bool sample_collision = checkPoseCollision(
                sample.x, sample.y, sample.yaw);
            if (!recovered)
            {
                if (!sample_collision)
                {
                    recovered = !near_start_collision;
                    continue;
                }
                if (sample.s <= start_escape_distance)
                {
                    near_start_collision = true;
                    continue;
                }
                arc_valid = false;
                break;
            }
            if (sample_collision)
            {
                arc_valid = false;
                break;
            }
        }
        if (!arc_valid || (near_start_collision && !recovered))
        {
            continue;
        }

        geometry_msgs::Twist target;
        target.linear.x = signed_speed;
        target.angular.z = arc.angular;
        escape_path_ = path;
        escape_target_cmd_ = target;
        escape_active_until_ = now + ros::Duration(escape_hold_time_);
        if (arc.direction < -0.5)
        {
            last_escape_direction_ = -1;
            last_escape_was_rotation_ = false;
        }
        else if (arc.direction > 0.5)
        {
            last_escape_direction_ = 1;
            last_escape_was_rotation_ = false;
        }
        else
        {
            last_escape_was_rotation_ = true;
        }
        cmd_vel = applyAccelerationLimits(target, now);
        ROS_WARN_THROTTLE(
            1.0,
            "cym_planner: executing a near-term collision escape with %.2f m/s, "
            "%.2f rad/s",
            target.linear.x,
            target.angular.z);
        return true;
    }

    return false;
}

double CymPlanner::distanceToGoal(
    const geometry_msgs::PoseStamped& robot_pose) const
{
    if (global_plan_.empty())
    {
        return std::numeric_limits<double>::infinity();
    }

    geometry_msgs::PoseStamped goal_in_local;
    if (!transformPose(global_plan_.back(), robot_pose.header.frame_id, goal_in_local))
    {
        return std::numeric_limits<double>::infinity();
    }
    return std::hypot(
        goal_in_local.pose.position.x - robot_pose.pose.position.x,
        goal_in_local.pose.position.y - robot_pose.pose.position.y);
}

bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    if (!initialized_ || global_plan_.empty() || costmap_ros_ == nullptr || tf_buffer_ == nullptr)
    {
        return false;
    }

    geometry_msgs::PoseStamped robot_pose;
    if (!costmap_ros_->getRobotPose(robot_pose))
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot get robot pose");
        return false;
    }

    const ros::Time now = ros::Time::now();

    if (goal_reached_)
    {
        return true;
    }

    if (shouldEnterGoalAlign(robot_pose))
    {
        state_ = PlannerState::GOAL_ALIGN;
        no_path_since_ = ros::Time(0);
        selected_local_path_.clear();
        last_collision_poses_.clear();
        last_left_candidate_.clear();
        last_right_candidate_.clear();
        publishDebugPaths({}, {}, ros::Time::now());
        publishPlannerState();
        return computeGoalAlignCommand(cmd_vel);
    }

    // Hold a validated escape arc for a short, bounded interval.  Rebuilding
    // the escape candidate every control cycle made acceleration limiting
    // alternate between reverse and forward before the robot could leave the
    // corner.  The arc was already swept-footprint checked; keeping its
    // command stable lets the platform actually clear the obstacle.
    if (!escape_active_until_.isZero() &&
        now < escape_active_until_ &&
        !escape_path_.empty())
    {
        state_ = PlannerState::TRACK;
        selected_local_path_ = escape_path_;
        publishDebugPaths({}, selected_local_path_, now);
        publishPlannerState();
        cmd_vel = applyAccelerationLimits(escape_target_cmd_, now);
        return true;
    }
    escape_active_until_ = ros::Time(0);
    escape_target_cmd_ = geometry_msgs::Twist();

    const int nearest_index = findNearestGlobalIndex(robot_pose);
    std::vector<PathPoint> cropped_path;
    if (nearest_index < 0 ||
        !cropGlobalPlan(nearest_index, planning_horizon_, cropped_path))
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot build a local reference path");
        cmd_vel = computeSafeStopCommand(ros::Time::now());
        return false;
    }

    // Drop every local-plan sample whose projection is behind the robot.  The
    // global plan can contain a short backward-looking tail around a sharp
    // corner, even after nearest-index tracking has advanced.  That tail is
    // historical geometry: it must not be optimized, collision-checked, or
    // drawn as the active local path.  A small tolerance keeps the current
    // sample stable on the costmap grid boundary.
    const double robot_yaw = tf2::getYaw(robot_pose.pose.orientation);
    const double robot_cosine = std::cos(robot_yaw);
    const double robot_sine = std::sin(robot_yaw);
    const double forward_tolerance = 0.02;
    std::vector<PathPoint> forward_path;
    forward_path.reserve(cropped_path.size());
    for (const PathPoint& point : cropped_path)
    {
        const double dx = point.x - robot_pose.pose.position.x;
        const double dy = point.y - robot_pose.pose.position.y;
        const double forward_projection = robot_cosine * dx + robot_sine * dy;
        if (forward_projection >= -forward_tolerance)
        {
            forward_path.push_back(point);
        }
    }
    cropped_path.swap(forward_path);
    if (cropped_path.empty())
    {
        PathPoint current_pose;
        current_pose.x = robot_pose.pose.position.x;
        current_pose.y = robot_pose.pose.position.y;
        current_pose.yaw = robot_yaw;
        cropped_path.push_back(current_pose);
    }

    const double robot_to_path = std::hypot(
        cropped_path.front().x - robot_pose.pose.position.x,
        cropped_path.front().y - robot_pose.pose.position.y);
    if (robot_to_path > 1e-4)
    {
        PathPoint current_pose;
        current_pose.x = robot_pose.pose.position.x;
        current_pose.y = robot_pose.pose.position.y;
        current_pose.yaw = robot_yaw;
        cropped_path.insert(cropped_path.begin(), current_pose);
        computePathGeometry(cropped_path);
    }

    std::vector<PathPoint> reference_path = resamplePath(cropped_path, path_resolution_);
    if (reference_path.size() < 2)
    {
        cmd_vel = computeSafeStopCommand(ros::Time::now());
        return false;
    }
    reference_path.front().yaw = tf2::getYaw(robot_pose.pose.orientation);
    if (obstacle_distance_field_.empty() ||
        (now - distance_field_time_).toSec() >= distance_field_update_period_)
    {
        rebuildDistanceField();
        distance_field_time_ = now;
    }

    const CollisionResult collision =
        evaluatePathCollision(reference_path, collision_horizon_);
    const std::vector<PathPoint> reference_collision_poses = last_collision_poses_;
    if (collision.collision)
    {
        if (no_path_since_.isZero())
        {
            no_path_since_ = now;
        }
        path_clear_since_ = ros::Time(0);
        ++blocked_cycles_;

        bool avoidance_selected = false;
        if (blocked_cycles_ >= obstacle_trigger_cycles_)
        {
            avoidance_selected = selectAvoidancePath(reference_path, collision, now);
            last_collision_poses_ = reference_collision_poses;
        }

        if (avoidance_selected)
        {
            no_path_since_ = ros::Time(0);
            publishDebugPaths(reference_path, selected_local_path_, now);
            publishPlannerState();
            const geometry_msgs::Twist target_command =
                computePurePursuitCommand(robot_pose, selected_local_path_);
            cmd_vel = applyAccelerationLimits(target_command, now);
            return true;
        }

        // A transient rolling-costmap overlap at the current pose can make
        // every lateral avoidance candidate fail at its shared start point.
        // Before braking into STOPPING, try a bounded swept-footprint escape
        // (reverse is preferred for the inner-corner geometry).
        const double escape_trigger_distance = std::max(
            0.45, circumscribed_radius_ + 0.10);
        if (collision.first_collision_distance <= escape_trigger_distance &&
            computeEscapeCommand(robot_pose, now, cmd_vel))
        {
            state_ = PlannerState::TRACK;
            selected_local_path_ = escape_path_;
            publishDebugPaths(reference_path, selected_local_path_, now);
            publishPlannerState();
            return true;
        }

        state_ = PlannerState::STOPPING;
        selected_local_path_.clear();
        publishDebugPaths(reference_path, selected_local_path_, now);
        publishPlannerState();
        cmd_vel = computeSafeStopCommand(now);

        const double blocked_duration = (now - no_path_since_).toSec();
        ROS_WARN_THROTTLE(
            1.0,
            "cym_planner: no safe swept-footprint candidate at %.2f m; braking "
            "(blocked %.2f s, cycles %d)",
            collision.first_collision_distance,
            blocked_duration,
            blocked_cycles_);
        if (blocked_duration > no_path_timeout_)
        {
            cmd_vel = geometry_msgs::Twist();
            previous_cmd_ = cmd_vel;
            return false;
        }
        return true;
    }

    no_path_since_ = ros::Time(0);
    blocked_cycles_ = 0;
    escape_path_.clear();
    escape_active_until_ = ros::Time(0);
    escape_target_cmd_ = geometry_msgs::Twist();
    last_collision_poses_.clear();

    if (locked_side_ != 0)
    {
        state_ = PlannerState::RETURN_PATH;
        if (!selectReturnPath(reference_path, now))
        {
            CandidatePath holding = generateReturnCandidate(reference_path, locked_side_);
            const CollisionResult holding_collision = evaluatePathCollision(
                holding.points, collision_horizon_);
            if (holding_collision.collision)
            {
                // The near reference segment was already checked above and is
                // clear.  A stale avoidance offset can nevertheless collide
                // farther down the planning horizon while returning from the
                // previous obstacle.  Stopping here deadlocks the robot well
                // before that future corner.  Drop the old side lock and keep
                // tracking the safe reference; the normal collision horizon
                // will create a fresh left/right decision when needed.
                locked_side_ = 0;
                return_scale_ = 0.0;
                path_clear_since_ = ros::Time(0);
                previous_offsets_.clear();
                selected_local_path_ = reference_path;
                last_collision_poses_.clear();
                state_ = PlannerState::TRACK;
                state_enter_time_ = now;
                publishDebugPaths(reference_path, selected_local_path_, now);
                publishPlannerState();
                const geometry_msgs::Twist target_command =
                    computePurePursuitCommand(robot_pose, selected_local_path_);
                cmd_vel = applyAccelerationLimits(target_command, now);
                return true;
            }
            selected_local_path_ = holding.points;
            previous_offsets_.clear();
            for (const PathPoint& point : selected_local_path_)
            {
                previous_offsets_.push_back(point.offset);
            }
        }
    }
    else
    {
        state_ = PlannerState::TRACK;
        selected_local_path_ = reference_path;
        previous_offsets_.clear();
        for (const PathPoint& point : selected_local_path_)
        {
            previous_offsets_.push_back(point.offset);
        }
    }

    if (locked_side_ == 0)
    {
        last_left_candidate_.clear();
        last_right_candidate_.clear();
    }
    publishDebugPaths(reference_path, selected_local_path_, now);
    publishPlannerState();

    const geometry_msgs::Twist target_command =
        computePurePursuitCommand(robot_pose, selected_local_path_);
    cmd_vel = applyAccelerationLimits(target_command, now);
    return true;
}

void CymPlanner::publishPath(
    const ros::Publisher& publisher,
    const std::vector<PathPoint>& path,
    const ros::Time& stamp) const
{
    nav_msgs::Path message;
    message.header.frame_id = local_frame_;
    message.header.stamp = stamp;
    message.poses.reserve(path.size());
    for (const PathPoint& point : path)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = message.header;
        pose.pose.position.x = point.x;
        pose.pose.position.y = point.y;
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, point.yaw);
        pose.pose.orientation = tf2::toMsg(orientation);
        message.poses.push_back(pose);
    }
    publisher.publish(message);
}

void CymPlanner::publishDebugPaths(
    const std::vector<PathPoint>& reference,
    const std::vector<PathPoint>& selected,
    const ros::Time& stamp)
{
    publishPath(reference_path_pub_, reference, stamp);
    publishPath(left_seed_path_pub_, last_left_candidate_, stamp);
    publishPath(right_seed_path_pub_, last_right_candidate_, stamp);
    publishPath(selected_path_pub_, selected, stamp);
    publishCollisionFootprints(stamp);
}

void CymPlanner::publishCollisionFootprints(const ros::Time& stamp) const
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const PathPoint& pose : last_collision_poses_)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = local_frame_;
        marker.header.stamp = stamp;
        marker.ns = "collision_footprints";
        marker.id = marker_id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.015;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.85;

        const double cosine = std::cos(pose.yaw);
        const double sine = std::sin(pose.yaw);
        for (const geometry_msgs::Point& footprint_point : footprint_)
        {
            geometry_msgs::Point world_point;
            world_point.x = pose.x + cosine * footprint_point.x - sine * footprint_point.y;
            world_point.y = pose.y + sine * footprint_point.x + cosine * footprint_point.y;
            world_point.z = 0.03;
            marker.points.push_back(world_point);
        }
        if (!marker.points.empty())
        {
            marker.points.push_back(marker.points.front());
        }
        markers.markers.push_back(marker);
    }
    collision_footprints_pub_.publish(markers);
}

std::string CymPlanner::plannerStateName() const
{
    switch (state_)
    {
        case PlannerState::TRACK:
            return "TRACK";
        case PlannerState::SELECT_SIDE:
            return "SELECT_SIDE";
        case PlannerState::AVOID_LEFT:
            return "AVOID_LEFT";
        case PlannerState::AVOID_RIGHT:
            return "AVOID_RIGHT";
        case PlannerState::RETURN_PATH:
            return "RETURN_PATH";
        case PlannerState::STOPPING:
            return "STOPPING";
        case PlannerState::GOAL_ALIGN:
            return "GOAL_ALIGN";
    }
    return "UNKNOWN";
}

void CymPlanner::publishPlannerState() const
{
    std_msgs::String message;
    message.data = plannerStateName();
    planner_state_pub_.publish(message);
}

bool CymPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace cym_planner
