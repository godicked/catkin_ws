#ifndef CAR_PLANNER_HPP
#define CAR_PLANNER_HPP


#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/static_layer.h>
#include <costmap_2d/costmap_2d_ros.h>


namespace fub
{
namespace planner
{

class CarPlanner
{

public:
    void init(ros::NodeHandle n);
    void run();

private:
    void goalCallback();
    void poseCallback();

    
    

}; // class CarPlanner


}; // namespace planner

}; // namespace fub

#endif