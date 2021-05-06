#ifndef PP_PLUGIN_H_
#define PP_PLUGIN_H_
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>

#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include "pp_msgs/PathPlanningPlugin.h"


namespace pp_plugin{
  /**
   * @class PPPlugin
   * @brief A global planner that creates service request for a plan and forwards the response to the move_base global planner module
   */
  class PPPlugin : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Constructor for the PPPlugin
       */
      PPPlugin();
      /**
       * @brief  Constructor for the PPPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      PPPlugin(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the PPPlugin
       * @param  name The name of this planner
       * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      void worldToGrid(float &x, float &y);

      /**
      * @brief Converts x,y grid cell coordinates to world coordinates (in meters)
      *        This transformation is derived from the map resolution, adjusts
      *        w.r.t the location of the map origin and can include an offset
      *        to place the world coordinate at the center point of a grid cell
      * @param x Grid cell map x coordinate value
      * @param y Grid cell map y coordinate value
      */
      void gridToWorld(float &x, float &y); 

      /**
      * @brief Checks if world coordinates are inside grid map bounds
      * @param x X-Axis value in the world frame of reference (in meters)
      * @param y Y-Axis value in the world frame of reference (in meters)
      * @return true if a index is in map bounds, otherwise false
      */
      bool inGridMap(float &x, float &y);

    private:
      costmap_2d::Costmap2DROS* costmap_ros_;
      costmap_2d::Costmap2D* costmap_;
      bool initialized_;
      // origin of the grid map w.r.t world's origin (meters)
      float origin_x_;
      float origin_y_;
      // resolution of the map (meters / pixel)
      float resolution_;
      
      bool path_at_node_center = false;
      float node_center_offset_ = 0;
      int width_;
      int height_;
      int map_size_;
      // declaring a service client
      ros::ServiceClient makeplan_service_;
       /**
       * @brief Publishes the global plan to display in RViz
       * @param path The plan as filled by the planner
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped> &path);
      ros::Publisher plan_pub_;
  };
}  
#endif