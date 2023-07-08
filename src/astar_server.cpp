#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include "astar_ros/GetPlan.h"

// Object
pathplanning::AstarConfig config;
pathplanning::Astar planner;
OccupancyGridParam OccGridParam;

// Parameters
double InflateRadius;

void MapCallback(const nav_msgs::OccupancyGrid& msg)
{
    // Get parameter
    OccGridParam.GetOccupancyGridParam(msg);

    // Get map
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; // set Unknown to 0
            // The origin of the OccGrid is on the bottom left corner of the map
            Map.at<uchar>(height-i-1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }

    // Initial Astar
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    planner.InitAstar(Map, Mask, config);
}


bool runAStar(astar_ros::GetPlan::Request& req, astar_ros::GetPlan::Response& res) {
  // Perform A*
  double start_time = ros::Time::now().toSec();

  // Preprocessing
  Point startpoint, targetpoint;
  vector<Point> PathList;
  Point2d cv_start = Point2d(req.start.x, req.start.y);
  OccGridParam.Map2ImageTransform(cv_start, startpoint);
  Point2d cv_goal = Point2d(req.goal.x, req.goal.y);
  OccGridParam.Map2ImageTransform(cv_goal, targetpoint);

  // Planning
  planner.PathPlanning(startpoint, targetpoint, PathList);

  if(!PathList.empty())
  {
    res.path.header.stamp = ros::Time::now();
    res.path.header.frame_id = "map";
    res.path.poses.clear();
    for(int i=0;i<PathList.size();i++)
    {
        Point2d dst_point;
        OccGridParam.Image2MapTransform(PathList[i], dst_point);

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        res.path.poses.push_back(pose_stamped);
    }
    double end_time = ros::Time::now().toSec();
    ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
  }

  else
  {
        ROS_ERROR("Can not find a valid path");
  }

  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar_server");
  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");
  ROS_INFO("A* algorithm service is ready.");
  
  // Parameter
  nh_priv.param<bool>("Euclidean", config.Euclidean, true);
  nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
  nh_priv.param<double>("InflateRadius", InflateRadius, -1);

  // Subscriber
  ros::Subscriber map_sub = nh.subscribe("map", 10, MapCallback);

  // Service
  ros::ServiceServer service = nh.advertiseService("/get_astar_plan", runAStar);

  ros::spin();

  return 0;
}
