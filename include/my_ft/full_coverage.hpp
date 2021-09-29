#ifndef FULL_COVERAGE_HPP
#define FULL_COVERAGE_HPP
#define dmax(a, b)   ((a) >= (b) ? (a):(b))
#define dmin(a, b)   ((a) <= (b) ? (a):(b))
#define clamp(a, lower, upper)    dmax(dmin(a, upper), lower)
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
 struct point
 {
   float x;
   float y;
 };
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

namespace lws_nav{
    class full_coverage
    {
    private:
        /* data */
        ros::NodeHandle nh_, nh2;
        MoveBaseClient ac;

        // void full_coverage::sending_goals();
        std::vector<std::vector<bool>> parse_map(nav_msgs::OccupancyGrid const& cpp_grid_, 
                                float robotRadius = 0.1, float toolRadius = 0.1);
        void bool2goals(std::vector<std::vector<bool>> bool_mp, std::vector<std::vector<point>> &goals, nav_msgs::OccupancyGrid const& cpp_grid_, int remove = 1);
        std::vector<std::vector<bool>> bool_map;                  
        visualization_msgs::MarkerArray my_marker_array;         
        void goals2visualize(visualization_msgs::MarkerArray & markers, std::vector<std::vector<point>> goals);
        bool doneCb(const actionlib::SimpleClientGoalState& state);
        move_base_msgs::MoveBaseGoal constructgoal(point Point, float yaw = 0);
        void status_callback(actionlib_msgs::GoalStatusArray status);
        void odom_callback(nav_msgs::Odometry odometry);

        ros::Publisher publish_markers;
        ros::Subscriber status_subscriber, odom_subscriber;
        ros::NodeHandle nh;
        ros::ServiceServer svs;
        ros::ServiceClient map_client;
        int goal_row, goal_col, last_row;
        geometry_msgs::Pose goal, geo_odom;
        nav_msgs::Odometry odomet;
        
        move_base_msgs::MoveBaseGoal goal_odom;
        std::vector<std::vector<point>> goals;
        bool _initialized_row = false;
        geometry_msgs::TransformStamped transformStamped;
        bool switc = true;
        bool rotated = false;
        bool recover = false;
        float angle;
        int remove = 0;
        bool this_row_initialised = false;
        std::vector<move_base_msgs::MoveBaseGoal> this_row_goals;
        int this_row_count =0 ;
        float radius;
        ros::Duration Slp = ros::Duration(1);
    public:
        full_coverage();
        full_coverage(ros::NodeHandle *nh); 
        
        // full_coverage() : ac("move_base", true) {};
        ~full_coverage();
    };



}//namespace lws_nav
#endif