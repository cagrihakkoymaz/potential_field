#include "ros/ros.h"
#include "../include/potential_field/potential_field.hpp"

#include "std_msgs/String.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>

using namespace std;

void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg, global_planner::PFP *planner_ptr)
{
    ROS_INFO("W: [%d]", msg->info.width);
    ROS_INFO("H: [%d]", msg->info.height);
    ROS_INFO("Resolution: [%f]", msg->info.resolution);
    ROS_INFO("Orig_X: [%f]", msg->info.origin.position.x);
    ROS_INFO("Orig_Y: [%f]", msg->info.origin.position.y);

    // Taking map values for planner class

    planner_ptr->map_width = msg->info.width;
    planner_ptr->map_height = msg->info.height;
    planner_ptr->map_resolution = msg->info.resolution;
    planner_ptr->origin_x = msg->info.origin.position.x;
    planner_ptr->origin_y = msg->info.origin.position.y;

    // Insert obstacles to vector
    vector<vector<int>> map_vector(planner_ptr->map_height, vector<int>(planner_ptr->map_width, 0));

    // Clear post obstacles from obstacle vector
    planner_ptr->obstacles.clear();

    // Push obstacles to the obstacle vector for calculate repulsion force every time new map created
    for (int i = 0; i < planner_ptr->map_height; i++)
    {
        for (int j = 0; j < planner_ptr->map_width; j++)
        {

            if (msg->data[i * planner_ptr->map_width + j] == 100)

            {
                planner_ptr->obstacles.push_back({j, i});
            }
        }
    }

    // Call repulsive force calcluator obstacle for calculation of repulsion forces
    planner_ptr->force_repulsion = planner_ptr->RFCalculator(planner_ptr->obstacles);
}

void start_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg, global_planner::PFP *planner_ptr)
{

    ROS_INFO("Start_X: [%f]", msg->pose.pose.position.x);
    ROS_INFO("Start_Y: [%f]", msg->pose.pose.position.y);
    float start_x = msg->pose.pose.position.x;
    float start_y = msg->pose.pose.position.y;

    // Trap start point is at outside the map
    if (0 < (start_x - planner_ptr->origin_x) / planner_ptr->map_resolution && (start_x - planner_ptr->origin_x) / planner_ptr->map_resolution < planner_ptr->map_height)
    {
        planner_ptr->tf_start_x = (start_x - planner_ptr->origin_x) / planner_ptr->map_resolution;
    }

    else
    {
        planner_ptr->tf_start_x = -1;
        ROS_INFO("START_X OUT OF THE MAP LIMITS START POINT NOT INITILIZED: [%f]", planner_ptr->tf_start_x);
    }

    if (0 < (start_y - planner_ptr->origin_y) / planner_ptr->map_resolution && (start_y - planner_ptr->origin_y) / planner_ptr->map_resolution < planner_ptr->map_width)
    {
        planner_ptr->tf_start_y = (start_y - planner_ptr->origin_y) / planner_ptr->map_resolution;
    }

    else
    {
        planner_ptr->tf_start_y = -1;
        ROS_INFO("START_Y OUT OF THE MAP LIMITS START POINT NOT INITILIZED: [%f]", planner_ptr->tf_start_y);
    }

    ROS_INFO("TF_START_X: [%f]", planner_ptr->tf_start_x);
    ROS_INFO("TF_START_Y: [%f]", planner_ptr->tf_start_y);
}
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg, global_planner::PFP *planner_ptr)
{

    ROS_INFO("Goal_X: [%f]", msg->pose.position.x);
    ROS_INFO("Goal_Y: [%f]", msg->pose.position.y);

    float goal_x = msg->pose.position.x;
    float goal_y = msg->pose.position.y;
    // Trap goal point is at outside the map

    if (0 < (goal_x - planner_ptr->origin_x) / planner_ptr->map_resolution && (goal_x - planner_ptr->origin_x) / planner_ptr->map_resolution < planner_ptr->map_width)
    {
        planner_ptr->tf_goal_x = (goal_x - planner_ptr->origin_x) / planner_ptr->map_resolution;
    }

    else
    {
        planner_ptr->tf_goal_x = -1;
        ROS_INFO("GOAL_X OUT OF THE MAP LIMITS GOAL POINT NOT INITILIZED: [%f]", planner_ptr->tf_goal_x);
    }

    if (0 < (goal_y - planner_ptr->origin_y) / planner_ptr->map_resolution && (goal_y - planner_ptr->origin_y) / planner_ptr->map_resolution < planner_ptr->map_width)
    {
        planner_ptr->tf_goal_y = (goal_y - planner_ptr->origin_y) / planner_ptr->map_resolution;
    }

    else
    {
        planner_ptr->tf_goal_y = -1;
        ROS_INFO("GOAL_Y OUT OF THE MAP LIMITS GOAL POINT NOT INITILIZED: [%f]", planner_ptr->tf_start_y);
    }

    ROS_INFO("TF_GOAL_X: [%f]", planner_ptr->tf_goal_x);
    ROS_INFO("TF_GOAL_Y: [%f]", planner_ptr->tf_goal_y);

    // Every time new goal created

    // Calculate attraction force
    planner_ptr->force_attraction = planner_ptr->AFCalculator(planner_ptr->tf_goal_x, planner_ptr->tf_goal_y);
    ROS_INFO("Attraction forces created ");
    // Calculate total force
    planner_ptr->force_total = planner_ptr->TFCalculator(planner_ptr->force_attraction, planner_ptr->force_repulsion);
    // Clear path waypoint list
    planner_ptr->path.clear();
    // Find new waypoint list
    planner_ptr->Path_Genrator(planner_ptr->tf_start_x, planner_ptr->tf_start_y, planner_ptr->force_total);
}

void assign_pose(nav_msgs::Path *NavPath, global_planner::PFP *planner_ptr)
{
    std::string frame_id = "map";
    NavPath->header.stamp = ros::Time::now();
    NavPath->header.frame_id = frame_id;
    // Create poses vector for assign robot global plan waypoints

    std::vector<geometry_msgs::PoseStamped> robot_poses{};

    for (int i = 0; i < planner_ptr->path.size(); i++)
    {

        geometry_msgs::PoseStamped PS;
        float tf_path_x = (planner_ptr->path[i][0] * planner_ptr->map_resolution) + planner_ptr->origin_x;
        float tf_path_y = (planner_ptr->path[i][1] * planner_ptr->map_resolution) + planner_ptr->origin_y;
        PS.header.stamp = ros::Time::now();
        PS.header.frame_id = frame_id;
        PS.pose.position.x = tf_path_x;
        PS.pose.position.y = tf_path_y;
        PS.pose.position.z = 0.1;
        PS.pose.orientation.w = 1.0;
        robot_poses.push_back(PS);
    }
    // Puslish path
    NavPath->poses = robot_poses;
}

int main(int argc, char **argv)
{

    double KP = 3;                // Attractive potential gain
    double ETA = 1000;            // Repulsive potential gain
    double distance_treshold = 1; // Treshold coralted with robot size choosed dummy number for this purpose

    // ROS subscriber and publishers
    ros::init(argc, argv, "potential_filed_planner");
    ros::NodeHandle n;
    ros::NodeHandle n_("~"); // get a handle to private parameters in ROS

    // PF Params
    n_.getParam("KP", KP);
    n_.getParam("ETA", ETA);
    n_.getParam("distance_treshold", distance_treshold);

    global_planner::PFP pf_planner(ETA, KP, distance_treshold);

    ros::Subscriber sub_map = n.subscribe<nav_msgs::OccupancyGrid>("map", 1000, boost::bind(&map_callback, _1, &pf_planner));
    ros::Subscriber sub_start = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, boost::bind(&start_callback, _1, &pf_planner));
    ros::Subscriber sub_goal = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, boost::bind(&goal_callback, _1, &pf_planner));
    ros::Publisher pub_path = n.advertise<nav_msgs::Path>("robot_path", 1); // For visuilize path regularly at RVIZ publish every 2s
    ros::Rate loop_rate(0.5);

    nav_msgs::Path NavP;
    while (ros::ok())
    {
        assign_pose(&NavP, &pf_planner);
        pub_path.publish(NavP);

        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
