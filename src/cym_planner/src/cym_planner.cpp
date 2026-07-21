#include "cym_planner.h"

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace
{
constexpr double kPi = 3.14159265358979323846;

double clampValue(double value, double lower, double upper)
{
    return std::max(lower, std::min(value, upper));
}

double normalizeAngle(double angle)
{
    while(angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while(angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

template <typename T>
void readPlannerParam(const ros::NodeHandle& planner_nh,
                      const ros::NodeHandle& legacy_nh,
                      const std::string& key,
                      T& value,
                      const T& default_value)
{
    if(!planner_nh.getParam(key, value))
    {
        legacy_nh.param<T>(key, value, default_value);
    }
}
}  // namespace

namespace cym_planner
{

CymPlanner::CymPlanner()
    : initialized_(false),
      tf_listener_(nullptr),
      costmap_ros_(nullptr),
      target_index_(0),
      pose_adjusting_(false),
      goal_reached_(false),
      carry_mode_(false),
      have_scan_(false)
{
}

CymPlanner::~CymPlanner()
{
    delete tf_listener_;
}

void CymPlanner::initialize(std::string name, tf2_ros::Buffer* /* tf */,
                            costmap_2d::Costmap2DROS* costmap_ros)
{
    if(initialized_)
    {
        ROS_WARN("cym_planner: initialize called more than once; ignoring duplicate call");
        return;
    }

    costmap_ros_ = costmap_ros;
    tf_listener_ = new tf::TransformListener();

    ros::NodeHandle planner_nh("~/" + name);
    ros::NodeHandle legacy_nh("~/CymPlanner");
    readPlannerParam(planner_nh, legacy_nh, "base_link_frame", base_link_frame_,
                     std::string("base_link"));
    readPlannerParam(planner_nh, legacy_nh, "scan_topic", scan_topic_,
                     std::string("/scan"));
    readPlannerParam(planner_nh, legacy_nh, "lookahead_distance", lookahead_distance_, 0.50);
    readPlannerParam(planner_nh, legacy_nh, "linear_x_gain", linear_x_gain_, 1.50);
    readPlannerParam(planner_nh, legacy_nh, "angular_gain", angular_gain_, 2.0);
    readPlannerParam(planner_nh, legacy_nh, "max_vel_x", max_vel_x_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "max_vel_theta", max_vel_theta_, 1.5);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_gain", final_yaw_gain_, 2.0);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_max_vel", final_yaw_max_vel_, 1.2);
    readPlannerParam(planner_nh, legacy_nh, "final_yaw_tolerance", final_yaw_tolerance_, 0.10);
    readPlannerParam(planner_nh, legacy_nh, "final_linear_x_gain", final_linear_x_gain_, 1.5);
    readPlannerParam(planner_nh, legacy_nh, "goal_position_tolerance",
                     goal_position_tolerance_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "carry_speed_scale", carry_speed_scale_, 0.80);

    readPlannerParam(planner_nh, legacy_nh, "scan_timeout", scan_timeout_, 0.25);
    readPlannerParam(planner_nh, legacy_nh, "scan_min_range", scan_min_range_, 0.03);
    readPlannerParam(planner_nh, legacy_nh, "scan_max_range", scan_max_range_, 4.0);
    readPlannerParam(planner_nh, legacy_nh, "safety_margin", safety_margin_, 0.035);
    readPlannerParam(planner_nh, legacy_nh, "braking_deceleration", braking_deceleration_, 3.0);
    readPlannerParam(planner_nh, legacy_nh, "reaction_time", reaction_time_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "simulation_time", simulation_time_, 0.30);
    readPlannerParam(planner_nh, legacy_nh, "simulation_step", simulation_step_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "v_samples", v_samples_, 7);
    readPlannerParam(planner_nh, legacy_nh, "w_samples", w_samples_, 9);
    readPlannerParam(planner_nh, legacy_nh, "path_distance_weight",
                     path_distance_weight_, 4.0);
    readPlannerParam(planner_nh, legacy_nh, "heading_weight", heading_weight_, 0.8);
    readPlannerParam(planner_nh, legacy_nh, "clearance_weight", clearance_weight_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "velocity_weight", velocity_weight_, 0.5);
    readPlannerParam(planner_nh, legacy_nh, "angular_velocity_weight",
                     angular_velocity_weight_, 0.05);
    readPlannerParam(planner_nh, legacy_nh, "obstacle_lookahead_distance",
                     obstacle_lookahead_distance_, 0.30);
    readPlannerParam(planner_nh, legacy_nh, "obstacle_cost_threshold",
                     obstacle_cost_threshold_, 253);

    lookahead_distance_ = std::max(0.05, lookahead_distance_);
    max_vel_x_ = std::max(0.0, max_vel_x_);
    max_vel_theta_ = std::max(0.0, max_vel_theta_);
    final_yaw_gain_ = std::max(0.0, final_yaw_gain_);
    final_yaw_max_vel_ = std::max(0.0, final_yaw_max_vel_);
    final_yaw_tolerance_ = clampValue(final_yaw_tolerance_, 0.01, kPi);
    final_linear_x_gain_ = std::max(0.0, final_linear_x_gain_);
    goal_position_tolerance_ = std::max(0.01, goal_position_tolerance_);
    carry_speed_scale_ = clampValue(carry_speed_scale_, 0.05, 1.0);
    scan_timeout_ = std::max(0.05, scan_timeout_);
    scan_min_range_ = std::max(0.0, scan_min_range_);
    scan_max_range_ = std::max(scan_min_range_ + 0.01, scan_max_range_);
    safety_margin_ = std::max(0.0, safety_margin_);
    braking_deceleration_ = std::max(0.01, braking_deceleration_);
    reaction_time_ = std::max(0.0, reaction_time_);
    simulation_time_ = std::max(0.05, simulation_time_);
    simulation_step_ = clampValue(simulation_step_, 0.01, simulation_time_);
    v_samples_ = std::max(2, v_samples_);
    w_samples_ = std::max(3, w_samples_);
    obstacle_lookahead_distance_ = std::max(0.0, obstacle_lookahead_distance_);
    obstacle_cost_threshold_ = static_cast<int>(
        clampValue(static_cast<double>(obstacle_cost_threshold_), 0.0, 255.0));

    footprint_min_x_ = std::numeric_limits<double>::infinity();
    footprint_max_x_ = -std::numeric_limits<double>::infinity();
    footprint_min_y_ = std::numeric_limits<double>::infinity();
    footprint_max_y_ = -std::numeric_limits<double>::infinity();
    const std::vector<geometry_msgs::Point>& footprint = costmap_ros_->getRobotFootprint();
    for(const geometry_msgs::Point& point : footprint)
    {
        footprint_min_x_ = std::min(footprint_min_x_, point.x);
        footprint_max_x_ = std::max(footprint_max_x_, point.x);
        footprint_min_y_ = std::min(footprint_min_y_, point.y);
        footprint_max_y_ = std::max(footprint_max_y_, point.y);
    }
    if(footprint.empty())
    {
        ROS_WARN("cym_planner: costmap footprint is empty; using 0.30 m x 0.20 m fallback");
        footprint_min_x_ = -0.15;
        footprint_max_x_ = 0.15;
        footprint_min_y_ = -0.10;
        footprint_max_y_ = 0.10;
    }

    ros::NodeHandle public_nh;
    carry_mode_sub_ = public_nh.subscribe(
        "/sim_task3/carry_mode", 1, &CymPlanner::carryModeCallback, this);
    scan_sub_ = public_nh.subscribe(scan_topic_, 1, &CymPlanner::scanCallback, this);
    laser_points_pub_ = planner_nh.advertise<sensor_msgs::PointCloud2>("laser_points", 1);
    candidate_trajectories_pub_ = planner_nh.advertise<visualization_msgs::MarkerArray>(
        "candidate_trajectories", 1);
    selected_trajectory_pub_ = planner_nh.advertise<visualization_msgs::Marker>(
        "selected_trajectory", 1);
    lookahead_footprint_pub_ = planner_nh.advertise<visualization_msgs::Marker>(
        "lookahead_footprint", 1, true);
    safety_state_pub_ = planner_nh.advertise<std_msgs::String>("safety_state", 1, true);

    initialized_ = true;
    ROS_INFO("cym_planner initialized: direct laser input=%s, scan timeout=%.2f s, "
             "trajectory rollout=%.2f s / %.2f s, samples=%d x %d",
             scan_topic_.c_str(), scan_timeout_, simulation_time_, simulation_step_,
             v_samples_, w_samples_);
}

void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
{
    if(carry_mode_ == message->data)
    {
        return;
    }
    carry_mode_ = message->data;
    ROS_INFO("cym_planner carry mode %s; speed scale %.2f",
             carry_mode_ ? "enabled" : "disabled",
             carry_mode_ ? carry_speed_scale_ : 1.0);
}

void CymPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    if(!initialized_ || scan->header.frame_id.empty())
    {
        return;
    }

    tf::StampedTransform laser_to_base;
    try
    {
        tf_listener_->lookupTransform(base_link_frame_, scan->header.frame_id,
                                      scan->header.stamp, laser_to_base);
    }
    catch(const tf::TransformException&)
    {
        try
        {
            // The laser is rigidly mounted.  During startup the exact scan stamp
            // can precede TF reception by one cycle, while the latest transform is
            // still geometrically correct for this fixed link.
            tf_listener_->lookupTransform(base_link_frame_, scan->header.frame_id,
                                          ros::Time(0), laser_to_base);
        }
        catch(const tf::TransformException& ex)
        {
            ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform laser %s to %s: %s",
                              scan->header.frame_id.c_str(), base_link_frame_.c_str(), ex.what());
            return;
        }
    }

    std::vector<LaserPoint> filtered_points;
    filtered_points.reserve(scan->ranges.size());
    const double max_range = std::min(static_cast<double>(scan->range_max), scan_max_range_);
    for(std::size_t index = 0; index < scan->ranges.size(); ++index)
    {
        const double range = scan->ranges[index];
        if(!std::isfinite(range) || range < scan_min_range_ || range > max_range)
        {
            continue;
        }
        const double angle = scan->angle_min + index * scan->angle_increment;
        const tf::Vector3 laser_point(range * std::cos(angle), range * std::sin(angle), 0.0);
        const tf::Vector3 base_point = laser_to_base * laser_point;
        filtered_points.push_back({base_point.x(), base_point.y()});
    }

    const ros::Time stamp = scan->header.stamp.isZero() ? ros::Time::now() : scan->header.stamp;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_);
        laser_points_ = filtered_points;
        last_scan_stamp_ = stamp;
        have_scan_ = true;
    }
    publishLaserPoints(filtered_points, stamp);
}

bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
    global_plan_ = plan;
    target_index_ = 0;
    pose_adjusting_ = false;
    goal_reached_ = false;
    return !global_plan_.empty();
}

bool CymPlanner::transformPlanPose(const geometry_msgs::PoseStamped& source,
                                   const std::string& target_frame,
                                   geometry_msgs::PoseStamped& result) const
{
    geometry_msgs::PoseStamped stamped_source = source;
    stamped_source.header.stamp = ros::Time(0);
    try
    {
        tf_listener_->transformPose(target_frame, stamped_source, result);
        return true;
    }
    catch(const tf::TransformException& ex)
    {
        ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform plan from %s to %s: %s",
                          source.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        return false;
    }
}

bool CymPlanner::selectTargetPose(geometry_msgs::PoseStamped& target_pose)
{
    for(int index = target_index_; index < static_cast<int>(global_plan_.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_base;
        if(!transformPlanPose(global_plan_[index], base_link_frame_, pose_base))
        {
            return false;
        }

        target_pose = pose_base;
        const double distance = std::hypot(pose_base.pose.position.x, pose_base.pose.position.y);
        if(distance >= lookahead_distance_ || index == static_cast<int>(global_plan_.size()) - 1)
        {
            target_index_ = index;
            return true;
        }
    }
    return false;
}

bool CymPlanner::isCostmapPathBlocked()
{
    if(obstacle_lookahead_distance_ <= 0.0)
    {
        return false;
    }

    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
    const int start_index = std::max(0, std::min(
        target_index_, static_cast<int>(global_plan_.size()) - 1));
    bool have_previous_pose = false;
    double previous_x = 0.0;
    double previous_y = 0.0;
    double checked_distance = 0.0;
    geometry_msgs::PoseStamped lookahead_pose;
    bool have_lookahead_pose = false;

    for(int index = start_index; index < static_cast<int>(global_plan_.size()); ++index)
    {
        geometry_msgs::PoseStamped pose_costmap;
        if(!transformPlanPose(global_plan_[index], costmap_frame, pose_costmap))
        {
            return true;
        }

        if(have_previous_pose)
        {
            checked_distance += std::hypot(pose_costmap.pose.position.x - previous_x,
                                           pose_costmap.pose.position.y - previous_y);
        }
        previous_x = pose_costmap.pose.position.x;
        previous_y = pose_costmap.pose.position.y;
        have_previous_pose = true;
        if(checked_distance > obstacle_lookahead_distance_)
        {
            break;
        }

        lookahead_pose = pose_costmap;
        have_lookahead_pose = true;
        unsigned int map_x = 0;
        unsigned int map_y = 0;
        if(costmap->worldToMap(pose_costmap.pose.position.x, pose_costmap.pose.position.y,
                               map_x, map_y) &&
           costmap->getCost(map_x, map_y) >= obstacle_cost_threshold_)
        {
            publishLookaheadFootprint(lookahead_pose, costmap_frame);
            ROS_WARN_THROTTLE(1.0,
                              "cym_planner: auxiliary costmap reports blocked global path; requesting replan");
            return true;
        }
    }

    if(have_lookahead_pose)
    {
        publishLookaheadFootprint(lookahead_pose, costmap_frame);
    }
    return false;
}

bool CymPlanner::copyFreshLaserPoints(std::vector<LaserPoint>& points,
                                      ros::Time& scan_stamp) const
{
    std::lock_guard<std::mutex> lock(scan_mutex_);
    if(!have_scan_)
    {
        return false;
    }
    const double age = (ros::Time::now() - last_scan_stamp_).toSec();
    if(age > scan_timeout_)
    {
        return false;
    }
    points = laser_points_;
    scan_stamp = last_scan_stamp_;
    return true;
}

double CymPlanner::clearanceToFootprint(double point_x, double point_y,
                                        double robot_x, double robot_y,
                                        double robot_yaw) const
{
    const double translated_x = point_x - robot_x;
    const double translated_y = point_y - robot_y;
    const double cos_yaw = std::cos(robot_yaw);
    const double sin_yaw = std::sin(robot_yaw);
    const double local_x = cos_yaw * translated_x + sin_yaw * translated_y;
    const double local_y = -sin_yaw * translated_x + cos_yaw * translated_y;

    const double min_x = footprint_min_x_ - safety_margin_;
    const double max_x = footprint_max_x_ + safety_margin_;
    const double min_y = footprint_min_y_ - safety_margin_;
    const double max_y = footprint_max_y_ + safety_margin_;
    const double dx = std::max(std::max(min_x - local_x, 0.0), local_x - max_x);
    const double dy = std::max(std::max(min_y - local_y, 0.0), local_y - max_y);
    return std::hypot(dx, dy);
}

double CymPlanner::forwardClearance(const std::vector<LaserPoint>& points) const
{
    const double lateral_limit = std::max(std::abs(footprint_min_y_),
                                          std::abs(footprint_max_y_)) + safety_margin_;
    double nearest_clearance = std::numeric_limits<double>::infinity();
    for(const LaserPoint& point : points)
    {
        if(point.x >= footprint_max_x_ && std::abs(point.y) <= lateral_limit)
        {
            nearest_clearance = std::min(nearest_clearance, point.x - footprint_max_x_);
        }
    }
    return nearest_clearance;
}

CymPlanner::CandidateTrajectory CymPlanner::simulateTrajectory(
    double linear_velocity, double angular_velocity, const std::vector<LaserPoint>& points,
    double front_clearance) const
{
    CandidateTrajectory candidate;
    candidate.linear_velocity = linear_velocity;
    candidate.angular_velocity = angular_velocity;
    candidate.clearance = scan_max_range_;
    candidate.score = -std::numeric_limits<double>::infinity();
    candidate.valid = true;

    const double stopping_distance = safety_margin_ + linear_velocity * reaction_time_ +
        linear_velocity * linear_velocity / (2.0 * braking_deceleration_);
    if(linear_velocity > 0.0 && front_clearance < stopping_distance)
    {
        candidate.valid = false;
        return candidate;
    }

    double robot_x = 0.0;
    double robot_y = 0.0;
    double robot_yaw = 0.0;
    const int steps = std::max(1, static_cast<int>(std::ceil(simulation_time_ / simulation_step_)));
    for(int step = 0; step < steps; ++step)
    {
        robot_x += linear_velocity * std::cos(robot_yaw) * simulation_step_;
        robot_y += linear_velocity * std::sin(robot_yaw) * simulation_step_;
        robot_yaw = normalizeAngle(robot_yaw + angular_velocity * simulation_step_);
        candidate.poses.push_back({robot_x, robot_y, robot_yaw});

        for(const LaserPoint& point : points)
        {
            const double clearance = clearanceToFootprint(
                point.x, point.y, robot_x, robot_y, robot_yaw);
            candidate.clearance = std::min(candidate.clearance, clearance);
            if(clearance <= 0.0)
            {
                candidate.valid = false;
                return candidate;
            }
        }
    }
    return candidate;
}

void CymPlanner::publishLaserPoints(const std::vector<LaserPoint>& points,
                                    const ros::Time& stamp) const
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = base_link_frame_;
    cloud.header.stamp = stamp;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x_iterator(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y_iterator(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z_iterator(cloud, "z");
    for(const LaserPoint& point : points)
    {
        *x_iterator = static_cast<float>(point.x);
        *y_iterator = static_cast<float>(point.y);
        *z_iterator = 0.03F;
        ++x_iterator;
        ++y_iterator;
        ++z_iterator;
    }
    laser_points_pub_.publish(cloud);
}

void CymPlanner::publishTrajectoryDebug(
    const std::vector<CandidateTrajectory>& candidates, int selected_index) const
{
    const ros::Time now = ros::Time::now();
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = base_link_frame_;
    clear_marker.header.stamp = now;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);

    for(std::size_t index = 0; index < candidates.size(); ++index)
    {
        const CandidateTrajectory& candidate = candidates[index];
        visualization_msgs::Marker marker;
        marker.header.frame_id = base_link_frame_;
        marker.header.stamp = now;
        marker.ns = "cym_planner_candidates";
        marker.id = static_cast<int>(index);
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.008;
        marker.color.a = candidate.valid ? 0.35 : 0.25;
        marker.color.r = candidate.valid ? 0.15F : 1.0F;
        marker.color.g = candidate.valid ? 0.65F : 0.10F;
        marker.color.b = candidate.valid ? 1.0F : 0.10F;
        marker.lifetime = ros::Duration(0.25);
        for(const TrajectoryPose& pose : candidate.poses)
        {
            geometry_msgs::Point point;
            point.x = pose.x;
            point.y = pose.y;
            point.z = 0.04;
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    candidate_trajectories_pub_.publish(marker_array);

    visualization_msgs::Marker selected_marker;
    selected_marker.header.frame_id = base_link_frame_;
    selected_marker.header.stamp = now;
    selected_marker.ns = "cym_planner_selected";
    selected_marker.id = 0;
    selected_marker.type = visualization_msgs::Marker::LINE_STRIP;
    selected_marker.scale.x = 0.025;
    selected_marker.color.r = 0.0F;
    selected_marker.color.g = 1.0F;
    selected_marker.color.b = 0.10F;
    selected_marker.color.a = 1.0F;
    selected_marker.lifetime = ros::Duration(0.25);
    if(selected_index < 0)
    {
        selected_marker.action = visualization_msgs::Marker::DELETE;
    }
    else
    {
        selected_marker.action = visualization_msgs::Marker::ADD;
        for(const TrajectoryPose& pose : candidates[selected_index].poses)
        {
            geometry_msgs::Point point;
            point.x = pose.x;
            point.y = pose.y;
            point.z = 0.05;
            selected_marker.points.push_back(point);
        }
    }
    selected_trajectory_pub_.publish(selected_marker);
}

void CymPlanner::publishLookaheadFootprint(const geometry_msgs::PoseStamped& lookahead_pose,
                                           const std::string& costmap_frame) const
{
    const std::vector<geometry_msgs::Point>& footprint = costmap_ros_->getRobotFootprint();
    if(footprint.empty())
    {
        return;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = costmap_frame;
    marker.header.stamp = ros::Time::now();
    marker.ns = "cym_planner_costmap";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = lookahead_pose.pose;
    marker.pose.position.z += 0.03;
    marker.scale.x = 0.02;
    marker.color.r = 0.05F;
    marker.color.g = 0.95F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker.points = footprint;
    marker.points.push_back(footprint.front());
    lookahead_footprint_pub_.publish(marker);
}

void CymPlanner::publishSafetyState(const std::string& state) const
{
    std_msgs::String message;
    message.data = state;
    safety_state_pub_.publish(message);
}

bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
    cmd_vel = geometry_msgs::Twist();
    if(!initialized_ || global_plan_.empty())
    {
        publishSafetyState("STOP: empty global plan");
        return false;
    }

    std::vector<LaserPoint> laser_points;
    ros::Time scan_stamp;
    if(!copyFreshLaserPoints(laser_points, scan_stamp))
    {
        publishSafetyState("STOP: laser scan unavailable or stale");
        ROS_WARN_THROTTLE(1.0, "cym_planner: refusing to move without a fresh %s scan",
                          scan_topic_.c_str());
        return false;
    }
    if(laser_points.empty())
    {
        publishSafetyState("STOP: laser scan has no valid points");
        return false;
    }

    geometry_msgs::PoseStamped final_pose;
    if(!transformPlanPose(global_plan_.back(), base_link_frame_, final_pose))
    {
        publishSafetyState("STOP: cannot transform final plan pose");
        return false;
    }
    const double final_distance = std::hypot(final_pose.pose.position.x, final_pose.pose.position.y);
    if(final_distance < goal_position_tolerance_)
    {
        pose_adjusting_ = true;
    }

    if(pose_adjusting_ &&
       std::abs(tf::getYaw(final_pose.pose.orientation)) < final_yaw_tolerance_)
    {
        goal_reached_ = true;
        publishSafetyState("GOAL_REACHED");
        return true;
    }

    geometry_msgs::PoseStamped target_pose;
    double desired_linear_velocity = 0.0;
    double desired_angular_velocity = 0.0;
    const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
    if(pose_adjusting_)
    {
        target_pose = final_pose;
        desired_linear_velocity = clampValue(
            final_pose.pose.position.x * final_linear_x_gain_ * motion_scale,
            0.0, max_vel_x_ * motion_scale);
        desired_angular_velocity = clampValue(
            tf::getYaw(final_pose.pose.orientation) * final_yaw_gain_ * motion_scale,
            -final_yaw_max_vel_ * motion_scale, final_yaw_max_vel_ * motion_scale);
    }
    else
    {
        if(!selectTargetPose(target_pose))
        {
            publishSafetyState("STOP: cannot select local path target");
            return false;
        }
        const double heading_error = std::atan2(target_pose.pose.position.y,
                                                target_pose.pose.position.x);
        desired_linear_velocity = clampValue(
            target_pose.pose.position.x * linear_x_gain_ * motion_scale,
            0.0, max_vel_x_ * motion_scale);
        desired_angular_velocity = clampValue(
            heading_error * angular_gain_ * motion_scale,
            -max_vel_theta_ * motion_scale, max_vel_theta_ * motion_scale);
    }

    const double max_angular_velocity = pose_adjusting_
        ? final_yaw_max_vel_ * motion_scale : max_vel_theta_ * motion_scale;
    const double front_clearance = forwardClearance(laser_points);
    std::vector<CandidateTrajectory> candidates;
    candidates.reserve(static_cast<std::size_t>(v_samples_ * w_samples_));
    int selected_index = -1;
    double best_score = -std::numeric_limits<double>::infinity();
    const double target_heading = std::atan2(target_pose.pose.position.y,
                                             target_pose.pose.position.x);

    for(int v_index = 0; v_index < v_samples_; ++v_index)
    {
        const double fraction = static_cast<double>(v_index) /
            static_cast<double>(v_samples_ - 1);
        const double candidate_linear_velocity = desired_linear_velocity * fraction;
        for(int w_index = 0; w_index < w_samples_; ++w_index)
        {
            const double center = 0.5 * static_cast<double>(w_samples_ - 1);
            const double angular_offset = (static_cast<double>(w_index) - center) /
                std::max(1.0, center) * max_angular_velocity;
            const double candidate_angular_velocity = clampValue(
                desired_angular_velocity + angular_offset,
                -max_angular_velocity, max_angular_velocity);
            CandidateTrajectory candidate = simulateTrajectory(
                candidate_linear_velocity, candidate_angular_velocity, laser_points,
                front_clearance);
            if(candidate.valid && !candidate.poses.empty())
            {
                const TrajectoryPose& end_pose = candidate.poses.back();
                const double path_error = std::hypot(
                    end_pose.x - target_pose.pose.position.x,
                    end_pose.y - target_pose.pose.position.y);
                const double heading_error = std::abs(normalizeAngle(target_heading - end_pose.yaw));
                const double normalized_speed = desired_linear_velocity > 1e-4
                    ? candidate.linear_velocity / desired_linear_velocity : 0.0;
                candidate.score =
                    -path_distance_weight_ * path_error
                    -heading_weight_ * heading_error
                    +clearance_weight_ * std::min(candidate.clearance, scan_max_range_)
                    +velocity_weight_ * normalized_speed
                    -angular_velocity_weight_ * std::abs(
                        candidate.angular_velocity - desired_angular_velocity);
                if(candidate.score > best_score)
                {
                    best_score = candidate.score;
                    selected_index = static_cast<int>(candidates.size());
                }
            }
            candidates.push_back(candidate);
        }
    }

    publishTrajectoryDebug(candidates, selected_index);
    if(selected_index < 0)
    {
        publishSafetyState("STOP: laser point cloud rejects every local trajectory");
        ROS_WARN_THROTTLE(1.0,
                          "cym_planner: no collision-free command from direct laser trajectory rollout");
        return false;
    }

    // The raw laser point cloud has already generated and selected a safe local
    // command above.  Costmap is deliberately evaluated afterwards as a secondary
    // route-level constraint: it may request a new global path, but never replaces
    // laser-based collision checking or velocity selection.
    if(isCostmapPathBlocked())
    {
        publishSafetyState("STOP: auxiliary costmap requests global replan");
        return false;
    }

    const CandidateTrajectory& selected = candidates[selected_index];
    cmd_vel.linear.x = selected.linear_velocity;
    cmd_vel.angular.z = selected.angular_velocity;
    std::ostringstream state;
    state.setf(std::ios::fixed);
    state.precision(2);
    state << "ACTIVE: direct laser rollout selected v=" << selected.linear_velocity
          << " w=" << selected.angular_velocity
          << " clearance=" << selected.clearance;
    publishSafetyState(state.str());
    return true;
}

bool CymPlanner::isGoalReached()
{
    return goal_reached_;
}

}  // namespace cym_planner
