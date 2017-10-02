#ifndef RRTX_PUBLISHER_HPP
#define RRTX_PUBLISHER_HPP

#include <ros/ros.h>

#include <rrtx/rrtx_struct.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

// #include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <rrtx/reeds_shepp_config.hpp>
#include <ompl/base/SpaceInformation.h>

#include <ompl/base/PlannerData.h>

#define GOAL_ID 0
#define VBOT_ID 1
#define PATH_NODE_ID 2
#define PATH_EDGE_ID 3
#define TREE_NODE_ID 4
#define TREE_EDGE_ID 5
#define INF_TRAJ_ID 6

using namespace ompl::base;

namespace rrt
{

    class RRTxPublisher
    {
        public:
        RRTxPublisher()
        {
        }

        void initialize(ros::NodeHandle *ros_node, std::string map_frame, ::SpaceInformationPtr si)
        {
            nh = ros_node;
            map_frame_ = map_frame;
            marker_pub = nh->advertise<visualization_msgs::Marker>("tree", 1000);
            si_ = si;

            std::cout << "initialize" << std::endl;
            std::cout << si_->getStateDimension() << std::endl;
        }


        void publish(PlannerData &data)
        {
            std::vector<const State *> states;
            int size = data.numVertices();

            for(int i = 0; i < size; i++)
            {
                states.push_back(data.getVertex(i).getState());
            }

            std::vector<std::pair<const State *, const State *> > edges;
            size = data.numEdges();
            for(unsigned int i = 0; i < size; i++)
            {
                std::vector<unsigned int> nbhs;
                data.getEdges(i, nbhs);
                
                for(auto nb : nbhs)
                {
                    edges.push_back(std::make_pair(
                        data.getVertex(i).getState(), 
                        data.getVertex(nb).getState())
                    );
                }
            }

            publishTree(states, edges);
        }

        void publishPath(std::vector<State *> &path)
        {
            visualization_msgs::Marker states, edges;

            states.header.frame_id  = map_frame_;
            states.header.stamp     = ros::Time::now();
            states.ns               = "path";
            states.id               = PATH_NODE_ID; 
            states.action           = visualization_msgs::Marker::ADD;
            states.type             = visualization_msgs::Marker::POINTS;

            states.scale.x          = 0.02;
            states.scale.y          = 0.02;

            states.color.a          = 1;
            states.color.r          = 1;
            states.color.g          = 0.5;


            edges.header.frame_id  = map_frame_;
            edges.header.stamp     = ros::Time::now();
            edges.ns               = "path";
            edges.id               = PATH_EDGE_ID; 
            edges.action           = visualization_msgs::Marker::ADD;
            edges.type             = visualization_msgs::Marker::LINE_LIST;

            edges.scale.x          = 0.05;
            edges.scale.y          = 0.05;

            edges.color.a          = 1;
            edges.color.r          = 0.5;
            edges.color.g          = 1;

            for(auto it = path.begin(); it+1 != path.end(); it++)
            {
                std::vector<geometry_msgs::Pose> poses;
                trajectoryPose(*it, *(it+1), poses);
                poseToLinesRviz(poses, edges.points);

                geometry_msgs::Point p1, p2;
                p1.x = getX(*it);
                p1.y = getY(*it);
                p2.x = getX(*(it+1));
                p2.y = getY(*(it+1));

                states.points.push_back(p1);
                edges.points.push_back(p1);
                edges.points.push_back(p2);
            }


            marker_pub.publish(states);
            marker_pub.publish(edges);
        }

        void publishPoses(State *start, State *goal)
        {
            visualization_msgs::Marker goal_;
            goal_.header.frame_id       = map_frame_;
            goal_.header.stamp          = ros::Time::now();
            goal_.ns                    = "pose";
            goal_.id                    = GOAL_ID;
            goal_.action                = visualization_msgs::Marker::ADD;
            goal_.type                  = visualization_msgs::Marker::POINTS;

            goal_.pose.orientation.w    = 1.0;
            goal_.scale.x               = 0.1;
            goal_.scale.y               = 0.1;

            goal_.color.a               = 1;
            goal_.color.r               = 1;

            geometry_msgs::Point pgoal;
            pgoal.x = getX(goal);
            pgoal.y = getY(goal);
            pgoal.z = 1;
            goal_.points.push_back(pgoal);

            marker_pub.publish(goal_);

            // publish Start (vbot)
            visualization_msgs::Marker vbot;
            vbot.header.frame_id       = map_frame_;
            vbot.header.stamp          = ros::Time::now();
            vbot.ns                    = "pose";
            vbot.id                    = VBOT_ID;
            vbot.action                = visualization_msgs::Marker::ADD;
            vbot.type                  = visualization_msgs::Marker::POINTS;

            vbot.pose.orientation.w    = 1.0;
            vbot.scale.x               = 0.1;
            vbot.scale.y               = 0.1;

            vbot.color.a               = 1;
            vbot.color.r               = 1;
            vbot.color.g               = 1;

            geometry_msgs::Point pvbot;
            pvbot.x = getX(start);
            pvbot.y = getY(start);
            pvbot.z = 1;
            vbot.points.push_back(pvbot);
            
            marker_pub.publish(vbot);
        }

        void publishTree(std::vector<const State *> &states, std::vector<std::pair<const State *, const State *> > edges)
        {
            visualization_msgs::Marker states_, edges_;

            states_.header.frame_id  = map_frame_;
            states_.header.stamp     = ros::Time::now();
            states_.ns               = "tree";
            states_.id               = TREE_NODE_ID; 
            states_.action           = visualization_msgs::Marker::ADD;
            states_.type             = visualization_msgs::Marker::POINTS;

            states_.scale.x          = 0.05;
            states_.scale.y          = 0.05;

            states_.color.a          = 1;
            states_.color.r          = 0;
            states_.color.g          = 0;
            states_.color.b          = 1;


            edges_.header.frame_id  = map_frame_;
            edges_.header.stamp     = ros::Time::now();
            edges_.ns               = "tree";
            edges_.id               = TREE_EDGE_ID; 
            edges_.action           = visualization_msgs::Marker::ADD;
            edges_.type             = visualization_msgs::Marker::LINE_LIST;

            edges_.scale.x          = 0.02;
            edges_.scale.y          = 0.02;

            edges_.color.a          = 1;
            edges_.color.g          = 1;

            for(auto state : states)
            {
                geometry_msgs::Point p1;
                p1.x = getX(state);
                p1.y = getY(state);

                states_.points.push_back(p1);
            }

            for(auto edge : edges)
            {
                std::vector<geometry_msgs::Pose> poses;
                trajectoryPose(edge.first, edge.second, poses);
                poseToLinesRviz(poses, edges_.points);
            }

            marker_pub.publish(states_);
            marker_pub.publish(edges_);
        }

        // void publishInfTrajectories(std::vector<Trajectory *> trajectories)
        // {
        //     visualization_msgs::Marker edges;

        //     edges.header.frame_id  = map_frame_;
        //     edges.header.stamp     = ros::Time::now();
        //     edges.ns               = "infTrajectories";
        //     edges.id               = INF_TRAJ_ID; 
        //     edges.action           = visualization_msgs::Marker::ADD;
        //     edges.type             = visualization_msgs::Marker::LINE_LIST;

        //     edges.scale.x          = 0.01;
        //     edges.scale.y          = 0.01;

        //     edges.color.a          = 1;
        //     edges.color.r          = 1;

        //     for(auto traj : trajectories)
        //     {
        //         geometry_msgs::Point p1, p2;
        //         p1.x = getX(traj->source->state);
        //         p1.y = getY(traj->source->state);
                
        //         p2.x = getX(traj->target->state);
        //         p2.y = getY(traj->target->state);

        //         edges.points.push_back(p1);
        //         edges.points.push_back(p2);
        //     }

        //     marker_pub.publish(edges);
        // }

    private:

        void trajectoryPose(const State *v, const State *u, std::vector<geometry_msgs::Pose> &poses)
        {
            double dist = si_->distance(v, u);
            auto ss = si_->getStateSpace()->as<ReedsSheppStateSpace>();
            double t = 1 / (dist * 4);
            ReedsSheppStateSpace::ReedsSheppPath path;
            bool firstTime = true;

            // std::cout << "dist " << dist << std::endl;
            // std::cout << "t " << t << std::endl;


            for(double i = 0; i < 1; i += t)
            {
                // std::cout << "i: " << i << std::endl;

                geometry_msgs::Pose p;
                ReedsSheppStateSpace::StateType *state = ss->allocState()->as<ReedsSheppStateSpace::StateType>();

                ss->interpolate(v, u, i, firstTime, path, state);
                p.position.x = state->getX();
                p.position.y = state->getY();
                poses.push_back(p);
            }

            // for(int i = 0; i < 5; i++)
            // {
            //     std::cout << " " << path.length_[i];
            // }
            
            // std::cout << std::endl;
        }

        void poseToLinesRviz(std::vector<geometry_msgs::Pose> &poses, std::vector<geometry_msgs::Point> &lines)
        {
            int size = poses.size() - 1;
            for(int i = 0; i < size; i++)
            {
                lines.push_back(poses[i].position);
                lines.push_back(poses[i+1].position);
            }
        }

        ros::NodeHandle *nh;
        ros::Publisher marker_pub;
        ros::Publisher path_pub;

        SpaceInformationPtr si_;

        std::string map_frame_;

    }; // class RRTxPublisher


}; // namespace rrt

#endif