#include <pluginlib/class_list_macros.h>
#include <pp_plugin.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(pp_plugin::PPPlugin, nav_core::BaseGlobalPlanner)

namespace pp_plugin{

  PPPlugin::PPPlugin(){
    initialized_ = false;
  }

  PPPlugin::PPPlugin(std::string name, costmap_2d::Costmap2DROS *costmap_ros){
    initialized_ = false;
    initialize(name, costmap_ros);
  }

  void PPPlugin::initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros){
    if (!initialized_){
      ros::NodeHandle private_nh("~/" + name);
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros->getCostmap();
      origin_x_ = costmap_->getOriginX();
      origin_y_ = costmap_->getOriginY();

      width_ = costmap_->getSizeInCellsX();
      height_ = costmap_->getSizeInCellsY();
      resolution_ = costmap_->getResolution();
      map_size_ = width_ * height_;

      // creating a client for the path planning service
      makeplan_service_ = private_nh.serviceClient<pp_msgs::PathPlanningPlugin>("make_plan");
      makeplan_service_.waitForExistence();
      // creating a publisher to display the complete path in RViz
      plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

      // place path waypoints at the center of each grid cell
      path_at_node_center = true;
      if (path_at_node_center){
        node_center_offset_ = resolution_ / 2;
      }

      initialized_ = true;
    }
  }

    bool PPPlugin::makePlan(const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal, std::vector<geometry_msgs::PoseStamped> &plan){
      plan.clear();

      std::vector<int> costmap(map_size_);

      for (size_t idx = 0; idx < map_size_; ++idx){
        int x, y;
        x = idx % width_;
        y = std::floor(idx / width_);
        costmap.at(idx) = static_cast<int>(costmap_->getCost(x, y));
      }

      float start_x = start.pose.position.x;
      float start_y = start.pose.position.y;
      float goal_x = goal.pose.position.x;
      float goal_y = goal.pose.position.y;

      if (inGridMap(start_x, start_y) && inGridMap(goal_x, goal_y)){
        worldToGrid(start_x, start_y);
        worldToGrid(goal_x, goal_y);
      } else{
        ROS_WARN("Start or goal position outside of the map's boundaries");
        return false;
      }

      if ((costmap[(static_cast<int>(goal_y) * width_) + static_cast<int>(goal_x)]) != 0){
        ROS_WARN("Goal pose is an obstacle.");
        return false;
      }

      std::vector<float> start_ = {start_x, start_y};
      std::vector<float> goal_ = {goal_x, goal_y};

      pp_msgs::PathPlanningPlugin makeplan;
      makeplan.request.costmap_ros = costmap;
      makeplan.request.start = start_;
      makeplan.request.goal = goal_;
      makeplan.request.width = width_;

      // call path planning service
      makeplan_service_.call(makeplan);

      // response of the path planning service
      std::vector<int> plan_x = makeplan.response.plan_x;
      std::vector<int> plan_y = makeplan.response.plan_y;
    
      if (plan_x.size()){
        for (unsigned int i = 0; i < plan_x.size(); i++){
          float x_path = static_cast<float>(plan_x[i]);
          float y_path = static_cast<float>(plan_y[i]);
        
          gridToWorld(x_path, y_path);

          geometry_msgs::PoseStamped position;
          position.header.stamp = ros::Time::now();
          position.header.frame_id = start.header.frame_id;
          position.pose.position.x = x_path;
          position.pose.position.y = y_path;
          position.pose.position.z = 0.0;
          position.pose.orientation.x = 0.0;
          position.pose.orientation.y = 0.0;
          position.pose.orientation.z = 0.0;
          position.pose.orientation.w = 1.0;

          plan.push_back(position);
        }
        // publish the plan for visualization in RViz
        publishPlan(plan);

        return true;
      }else{
        // no plan found
        return false;
      }
    }

    void PPPlugin::publishPlan(const std::vector<geometry_msgs::PoseStamped> &path){
      nav_msgs::Path rviz_path;
      rviz_path.poses.resize(path.size());

      rviz_path.header.frame_id = path[0].header.frame_id;
      rviz_path.header.stamp = path[0].header.stamp;

      for (unsigned int i = 0; i < path.size(); i++){
        rviz_path.poses[i] = path[i];
      }

      plan_pub_.publish(rviz_path);
    }

    void PPPlugin::worldToGrid(float &x, float &y){
      x = static_cast<size_t>((x - origin_x_) / resolution_);
      y = static_cast<size_t>((y - origin_y_) / resolution_);
    }

    void PPPlugin::gridToWorld(float &x, float &y){
      x = x * resolution_ + origin_x_ + node_center_offset_;
      y = y * resolution_ + origin_y_ + node_center_offset_;
    }

    bool PPPlugin::inGridMap(float &x, float &y){
      if (x < origin_x_ || y < origin_y_ || x > origin_x_ + (width_ * resolution_) || y > origin_y_ + (height_ * resolution_))
        return false;
      return true;
    }

  }; // namespace pp_plugin