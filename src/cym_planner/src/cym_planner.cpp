#include "cym_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

PLUGINLIB_EXPORT_CLASS(cym_planner::CymPlanner, nav_core::BaseLocalPlanner)

namespace cym_planner
{
    CymPlanner::CymPlanner()
    {
        setlocale(LC_ALL, "");
    }
    CymPlanner::~CymPlanner()
    {}

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    // ===== TF 坐标系名称（通过 ROS 参数配置，无需重新编译即可适配不同机器人）=====
    std::string base_link_frame_;   // 机器人本体坐标系 (默认: base_link)
    std::string odom_frame_;        // 里程计坐标系      (默认: odom)
    double linear_x_gain_;
    double linear_x_kd_;
    double angular_gain_;
    double max_vel_x_;
    double max_vel_theta_;
    // Final pose alignment uses its own gains.  It must not inherit the
    // path-following gain because the vehicle is already at the goal here.
    double final_yaw_gain_;
    double final_yaw_max_vel_;
    double final_yaw_tolerance_;
    double final_linear_x_gain_;
    double heading_tolerance_;
    double obstacle_lookahead_distance_;
    int obstacle_cost_threshold_;
    double previous_linear_error_;
    ros::Time previous_control_time_;
    bool linear_derivative_initialized_;
    ros::Publisher lookahead_footprint_pub_;

    void publishLookaheadFootprint(const geometry_msgs::PoseStamped& lookahead_pose,
                                   const std::string& costmap_frame)
    {
        const std::vector<geometry_msgs::Point>& footprint =
            costmap_ros_->getRobotFootprint();
        if(footprint.empty())
        {
            return;
        }

        visualization_msgs::Marker marker;
        marker.header.frame_id = costmap_frame;
        marker.header.stamp = ros::Time::now();
        marker.ns = "cym_planner";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = lookahead_pose.pose;
        marker.pose.position.z += 0.03;
        marker.scale.x = 0.025;
        marker.color.r = 0.05;
        marker.color.g = 0.95;
        marker.color.b = 0.95;
        marker.color.a = 1.0;
        marker.points = footprint;
        marker.points.push_back(footprint.front());
        lookahead_footprint_pub_.publish(marker);
    }

    void CymPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        ROS_WARN("%s", u8"\u8be5\u6211\u4e0a\u573a\u8868\u6f14\u4e86!");
        tf_listener_ = new tf::TransformListener();
        costmap_ros_ = costmap_ros;

        // ===== 从 ROS 参数服务器读取坐标系名称 =====
        // name 通常为 "cym_planner/CymPlanner"（即 base_local_planner 参数值）
        // 参数命名空间: ~/cym_planner/CymPlanner/  =>  /move_base/cym_planner/CymPlanner/
        // 在 yaml 中对应: cym_planner/CymPlanner: 下的子项
        ros::NodeHandle planner_nh("~/" + name);
        ros::NodeHandle legacy_nh("~/CymPlanner");
        if(!planner_nh.getParam("base_link_frame", base_link_frame_))
            legacy_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
        if(!planner_nh.getParam("odom_frame", odom_frame_))
            legacy_nh.param<std::string>("odom_frame", odom_frame_, "odom");
        if(!planner_nh.getParam("linear_x_gain", linear_x_gain_))
            legacy_nh.param("linear_x_gain", linear_x_gain_, 1.5);
        if(!planner_nh.getParam("linear_x_kd", linear_x_kd_))
            legacy_nh.param("linear_x_kd", linear_x_kd_, 0.0);
        if(!planner_nh.getParam("angular_gain", angular_gain_))
            legacy_nh.param("angular_gain", angular_gain_, 2.0);
        if(!planner_nh.getParam("max_vel_x", max_vel_x_))
            legacy_nh.param("max_vel_x", max_vel_x_, 1.0);
        if(!planner_nh.getParam("max_vel_theta", max_vel_theta_))
            legacy_nh.param("max_vel_theta", max_vel_theta_, 1.5);
        if(!planner_nh.getParam("final_yaw_gain", final_yaw_gain_))
            legacy_nh.param("final_yaw_gain", final_yaw_gain_, 2.0);
        if(!planner_nh.getParam("final_yaw_max_vel", final_yaw_max_vel_))
            legacy_nh.param("final_yaw_max_vel", final_yaw_max_vel_, 1.2);
        if(!planner_nh.getParam("final_yaw_tolerance", final_yaw_tolerance_))
            legacy_nh.param("final_yaw_tolerance", final_yaw_tolerance_, 0.10);
        if(!planner_nh.getParam("final_linear_x_gain", final_linear_x_gain_))
            legacy_nh.param("final_linear_x_gain", final_linear_x_gain_, 1.5);
        if(!planner_nh.getParam("heading_tolerance", heading_tolerance_))
            legacy_nh.param("heading_tolerance", heading_tolerance_, 0.20);
        if(!planner_nh.getParam("obstacle_lookahead_distance", obstacle_lookahead_distance_))
            legacy_nh.param("obstacle_lookahead_distance", obstacle_lookahead_distance_, 0.36);
        if(!planner_nh.getParam("obstacle_cost_threshold", obstacle_cost_threshold_))
            legacy_nh.param("obstacle_cost_threshold", obstacle_cost_threshold_, 253);
        if(!planner_nh.getParam("carry_speed_scale", carry_speed_scale_))
            legacy_nh.param("carry_speed_scale", carry_speed_scale_, 0.25);
        obstacle_lookahead_distance_ = std::max(0.0, obstacle_lookahead_distance_);
        obstacle_cost_threshold_ = std::max(0, std::min(255, obstacle_cost_threshold_));
        final_yaw_gain_ = std::max(0.0, final_yaw_gain_);
        final_yaw_max_vel_ = std::max(0.0, final_yaw_max_vel_);
        final_yaw_tolerance_ = std::max(0.01, std::min(M_PI, final_yaw_tolerance_));
        final_linear_x_gain_ = std::max(0.0, final_linear_x_gain_);
        carry_speed_scale_ = std::max(0.05, std::min(1.0, carry_speed_scale_));
        carry_mode_ = false;
        ros::NodeHandle public_nh;
        carry_mode_sub_ = public_nh.subscribe(
            "/sim_task3/carry_mode", 1, &CymPlanner::carryModeCallback, this);
        lookahead_footprint_pub_ = planner_nh.advertise<visualization_msgs::Marker>(
            "lookahead_footprint", 1);
        previous_linear_error_ = 0.0;
        previous_control_time_ = ros::Time(0);
        linear_derivative_initialized_ = false;
        ROS_WARN("cym_planner initialized | linear kp/kd: %.2f/%.2f | max_vel_x: %.2f | max_vel_theta: %.2f | final yaw gain/max/tolerance: %.2f/%.2f/%.3f | obstacle lookahead/threshold: %.2f/%d",
                 linear_x_gain_, linear_x_kd_, max_vel_x_, max_vel_theta_,
                 final_yaw_gain_, final_yaw_max_vel_, final_yaw_tolerance_,
                 obstacle_lookahead_distance_, obstacle_cost_threshold_);
    }

    void CymPlanner::carryModeCallback(const std_msgs::Bool::ConstPtr& message)
    {
        if(carry_mode_ == message->data)
            return;
        carry_mode_ = message->data;
        ROS_WARN("cym_planner carry mode %s; speed scale %.2f",
                 carry_mode_ ? "enabled" : "disabled",
                 carry_mode_ ? carry_speed_scale_ : 1.0);
    }
    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool CymPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_ = 0;
        global_plan_ = plan;
        pose_adjusting_ = false;
        goal_reached_ = false;
        linear_derivative_initialized_ = false;
        return true;
    }
    bool CymPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        if(global_plan_.empty())
        {
            cmd_vel = geometry_msgs::Twist();
            return false;
        }

        //get costmap
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        unsigned char* map_data = costmap->getCharMap();
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        cv::Mat map_image(size_y, size_x, CV_8UC3, cv::Scalar(128, 128, 128));
        
        for(unsigned int y = 0; y < size_y; y++)
        {
            for(unsigned int x = 0; x < size_x; x++)
            {
                int map_index = y * size_x + x;
                unsigned char cost = map_data[map_index];
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                if(cost == 0)
                {
                    pixel = cv::Vec3b(128, 128, 128);
                }else if(cost == 253)
                {
                    pixel = cv::Vec3b(255,255,0);
                }else if(cost == 254)
                {
                    pixel = cv::Vec3b(0, 0, 0);
                }else 
                {
                    unsigned char blue = 255 - cost;
                    unsigned char red = 255 - cost;
                    pixel = cv::Vec3b(blue, 0, red);
                }
            }
        }
        const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
        const int check_start_index = std::max(
            0, std::min(target_index_, static_cast<int>(global_plan_.size()) - 1));
        double checked_distance = 0.0;
        double previous_x = 0.0;
        double previous_y = 0.0;
        bool have_previous_point = false;
        geometry_msgs::PoseStamped lookahead_pose;
        bool have_lookahead_pose = false;

        // Draw the plan and reject a blocked segment ahead of the robot.  Returning
        // false intentionally delegates detour planning to move_base/global_planner.
        for(int i = 0; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_costmap;
            global_plan_[i].header.stamp = ros::Time(0);
            try
            {
                tf_listener_->transformPose(costmap_frame, global_plan_[i], pose_costmap);
            }
            catch(tf::TransformException& ex)
            {
                ROS_WARN_THROTTLE(1.0, "cym_planner: cannot transform global plan into %s: %s",
                                  costmap_frame.c_str(), ex.what());
                cmd_vel = geometry_msgs::Twist();
                return false;
            }

            unsigned int x = 0;
            unsigned int y = 0;
            if(!costmap->worldToMap(pose_costmap.pose.position.x,
                                    pose_costmap.pose.position.y, x, y))
            {
                continue;
            }
            cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(255, 0, 255));

            if(i < check_start_index)
            {
                continue;
            }

            if(have_previous_point)
            {
                checked_distance += std::hypot(
                    pose_costmap.pose.position.x - previous_x,
                    pose_costmap.pose.position.y - previous_y);
            }
            previous_x = pose_costmap.pose.position.x;
            previous_y = pose_costmap.pose.position.y;
            have_previous_point = true;

            if(checked_distance > obstacle_lookahead_distance_)
            {
                break;
            }

            lookahead_pose = pose_costmap;
            have_lookahead_pose = true;
            cv::circle(map_image, cv::Point(x, y), 0, cv::Scalar(0, 0, 255));
            const unsigned char cost = costmap->getCost(x, y);
            if(cost >= obstacle_cost_threshold_)
            {
                publishLookaheadFootprint(lookahead_pose, costmap_frame);
                ROS_WARN_THROTTLE(1.0,
                                  "cym_planner: blocked path segment, cost=%u threshold=%d; requesting global replan",
                                  static_cast<unsigned int>(cost), obstacle_cost_threshold_);
                cmd_vel = geometry_msgs::Twist();
                return false;
            }
        }

        if(have_lookahead_pose)
        {
            publishLookaheadFootprint(lookahead_pose, costmap_frame);
        }






        map_image.at<cv::Vec3b>(size_y / 2, size_x / 2) = cv::Vec3b(0, 255, 0);








        //return map
        cv::Mat flipped_image(size_y, size_x, CV_8UC3, cv::Scalar(128,128,128));
        for (unsigned int y = 0; y < size_y; ++y)
        {
            for (unsigned int x = 0; x < size_x; ++x)
            {
                cv::Vec3b& pixel = map_image.at<cv::Vec3b>(y, x);
                flipped_image.at<cv::Vec3b>(size_y - 1 - y, size_x - 1 - x) = pixel;
            }
        }
        map_image = flipped_image;

        //show costmap
        cv::namedWindow("Map");
        cv::resize(map_image, map_image, cv::Size(size_x*5, size_y*5),0,0,cv::INTER_NEAREST);
        cv::resizeWindow("Map", size_x*5, size_y*5);
        cv::imshow("Map", map_image);
        






        int final_index = global_plan_.size() - 1;
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp = ros::Time(0);
        tf_listener_->transformPose(base_link_frame_, global_plan_[final_index], pose_final);
        if(pose_adjusting_ == false)
        {
            double dx = pose_final.pose.position.x;
            double dy = pose_final.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if(dist < 0.05)
            {
                pose_adjusting_ = true;
            }
        }
        const double motion_scale = carry_mode_ ? carry_speed_scale_ : 1.0;
        if(pose_adjusting_ == true)
        {
            double final_yaw = tf::getYaw(pose_final.pose.orientation);
            ROS_WARN("final_yaw: %f", final_yaw);
            cmd_vel.angular.z = std::max(
                -final_yaw_max_vel_ * motion_scale,
                std::min(final_yaw * final_yaw_gain_ * motion_scale,
                         final_yaw_max_vel_ * motion_scale));
            cmd_vel.linear.x = pose_final.pose.position.x * final_linear_x_gain_ * motion_scale;
            if(abs(final_yaw) < final_yaw_tolerance_)
            {
                goal_reached_ = true;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.x = 0;
                ROS_WARN("Goal Reached!");
            }
            return true;
        }









        geometry_msgs::PoseStamped target_pose;
        for(int i = target_index_; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose(base_link_frame_, global_plan_[i], pose_base);
            double dx = pose_base.pose.position.x;
            double dy = pose_base.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            if(dist > 0.2)
            {
                target_pose = pose_base;
                target_index_ = i;
                ROS_WARN("target_index_: %d", target_index_);
                break;
            }
            if (i == global_plan_.size() - 1)
            {
                target_pose = pose_base;
            }
        }
        const double heading_error = std::atan2(
            target_pose.pose.position.y, target_pose.pose.position.x);
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = std::max(
            -max_vel_theta_ * motion_scale,
            std::min(heading_error * angular_gain_ * motion_scale,
                     max_vel_theta_ * motion_scale));
        const double heading_speed_scale = std::max(
            0.25, std::cos(std::min(std::abs(heading_error), 1.57079632679)));
        const double linear_error = target_pose.pose.position.x;
        const ros::Time control_time = ros::Time::now();
        double linear_error_derivative = 0.0;
        if(linear_derivative_initialized_)
        {
            const double control_period = (control_time - previous_control_time_).toSec();
            if(control_period > 1e-3)
            {
                linear_error_derivative = std::max(
                    -2.0,
                    std::min((linear_error - previous_linear_error_) / control_period, 2.0));
            }
        }
        previous_linear_error_ = linear_error;
        previous_control_time_ = control_time;
        linear_derivative_initialized_ = true;
        const double linear_control =
            (linear_error * linear_x_gain_ + linear_error_derivative * linear_x_kd_)
            * motion_scale;
        cmd_vel.linear.x = std::max(
            0.0,
            std::min(linear_control, max_vel_x_ * motion_scale)
                * heading_speed_scale);
        global_plan_[target_index_].header.stamp = ros::Time(0);
        tf_listener_->transformPose(base_link_frame_, global_plan_[target_index_], target_pose);
        double target_x = target_pose.pose.position.x;
        double target_y = target_pose.pose.position.y;
        double dist = std::sqrt(target_x * target_x + target_y * target_y);




        cv::Mat plane_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
        for(int i = 0; i < global_plan_.size(); i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp = ros::Time(0);
            tf_listener_->transformPose(base_link_frame_, global_plan_[i], pose_base);
            int cv_x = 300 - pose_base.pose.position.x * 100;
            int cv_y = 300 - pose_base.pose.position.y * 100;
            cv::circle(plane_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
        }
        cv::circle(plane_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
        cv::line(plane_image, cv::Point(65, 300), cv::Point(510, 300), cv::Scalar(0, 255, 0),1);
        cv::line(plane_image, cv::Point(300, 45), cv::Point(300, 555), cv::Scalar(0, 255, 0),1);
        //cv::namedWindow("Plan");
        //cv::imshow("Plan", plane_image);
        cv::waitKey(1);
        return true;
    }
    bool CymPlanner::isGoalReached()
    {
        return goal_reached_;
    }

} // namespace cym_planner
