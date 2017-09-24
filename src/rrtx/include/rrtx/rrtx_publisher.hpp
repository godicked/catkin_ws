#ifndef RRTX_PUBLISHER_HPP
#define RRTX_PUBLISHER_HPP

#include <ros/ros.h>

#include <rrtx/rrtx_struct.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/SpaceInformation.h>

#define GOAL_ID 0
#define VBOT_ID 1
#define PATH_NODE_ID 2
#define PATH_EDGE_ID 3
#define TREE_NODE_ID 4
#define TREE_EDGE_ID 5
#define INF_TRAJ_ID 6


namespace rrt
{

    double getX(Node *v)
    {
        return v->state->as<ompl::base::SE2StateSpace::StateType>()->getX();
    }

    double getY(Node *v)
    {
        return v->state->as<ompl::base::SE2StateSpace::StateType>()->getY();
    }

    class RRTxPublisher
    {
        public:
        RRTxPublisher()
        {
        }

        void initialize(ros::NodeHandle *ros_node, std::string map_frame, ompl::base::SpaceInformationPtr si)
        {
            nh = ros_node;
            map_frame_ = map_frame;
            marker_pub = nh->advertise<visualization_msgs::Marker>("tree", 1000);
            si_ = si;

            std::cout << "initialize" << std::endl;
            std::cout << si_->getStateDimension() << std::endl;
        }

        void publishTree(Node *vbot_, Node *goal_, std::list<Node> nodeContainer, bool full = false)
        {
            visualization_msgs::Marker goal;
            goal.header.frame_id       = map_frame_;
            goal.header.stamp          = ros::Time::now();
            goal.ns                    = "pose";
            goal.id                    = GOAL_ID;
            goal.action                = visualization_msgs::Marker::ADD;
            goal.type                  = visualization_msgs::Marker::POINTS;

            goal.pose.orientation.w    = 1.0;
            goal.scale.x               = 0.51;
            goal.scale.y               = 0.51;

            goal.color.a               = 1;
            goal.color.r               = 1;

            geometry_msgs::Point pgoal;
            pgoal.x = getX(goal_);
            pgoal.y = getY(goal_);
            pgoal.z = 1;
            goal.points.push_back(pgoal);

            marker_pub.publish(goal);

            // publish Start (vbot)
            visualization_msgs::Marker vbot;
            vbot.header.frame_id       = map_frame_;
            vbot.header.stamp          = ros::Time::now();
            vbot.ns                    = "pose";
            vbot.id                    = VBOT_ID;
            vbot.action                = visualization_msgs::Marker::ADD;
            vbot.type                  = visualization_msgs::Marker::POINTS;

            vbot.pose.orientation.w    = 1.0;
            vbot.scale.x               = 0.51;
            vbot.scale.y               = 0.51;

            vbot.color.a               = 1;
            vbot.color.r               = 1;
            vbot.color.g               = 1;

            geometry_msgs::Point pvbot;
            pvbot.x = getX(vbot_);
            pvbot.y = getY(vbot_);
            pvbot.z = 1;
            vbot.points.push_back(pvbot);
            std::cout << "goal" << std::endl;
            // pvbot.x = getX(vbot_->parent);
            // pvbot.y = getY(vbot_->parent);

            // vbot.points.push_back(pvbot);

            marker_pub.publish(vbot);

            

            visualization_msgs::Marker nodes, edges;

            nodes.header.frame_id  = map_frame_;
            nodes.header.stamp     = ros::Time::now();
            nodes.ns               = "path";
            nodes.id               = PATH_NODE_ID; 
            nodes.action           = visualization_msgs::Marker::ADD;
            nodes.type             = visualization_msgs::Marker::POINTS;

            nodes.scale.x          = 0.2;
            nodes.scale.y          = 0.2;

            nodes.color.a          = 1;
            nodes.color.r          = 1;
            nodes.color.g          = 0.5;


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


            for(auto v = vbot_; v->parent != nullptr; v = v->parent)
            {
                std::vector<geometry_msgs::Pose> poses;
                trajectoryPose(*v, *v->parent, poses);
                poseToLinesRviz(poses, edges.points);

                geometry_msgs::Point p;
                p.x = getX(v);
                p.y = getY(v);

                nodes.points.push_back(p);
            }

            // for(int i = 0; i < path.size(); i++)
            // {
            //     geometry_msgs::Point p;
            //     p.x = getX(path[i]);
            //     p.y = getY(path[i]);

            //     nodes.points.push_back(p);

            //     if (i < path.size() - 1)
            //     {
            //         edges.points.push_back(p);
            //         p.x = getX(path[i+1]);
            //         p.y = getY(path[i+1]);
            //         edges.points.push_back(p);
            //     }

                

            // }

            marker_pub.publish(nodes);
            marker_pub.publish(edges);

            if(full)
            {
                nodes.points.clear();
                edges.points.clear();

                nodes.header.frame_id  = map_frame_;
                nodes.header.stamp     = ros::Time::now();
                nodes.ns               = "tree";
                nodes.id               = TREE_NODE_ID; 
                nodes.action           = visualization_msgs::Marker::ADD;
                nodes.type             = visualization_msgs::Marker::POINTS;

                nodes.scale.x          = 0.5;
                nodes.scale.y          = 0.5;

                nodes.color.a          = 1;
                nodes.color.r          = 0;
                nodes.color.g          = 0;
                nodes.color.b          = 1;


                edges.header.frame_id  = map_frame_;
                edges.header.stamp     = ros::Time::now();
                edges.ns               = "tree";
                edges.id               = TREE_EDGE_ID; 
                edges.action           = visualization_msgs::Marker::ADD;
                edges.type             = visualization_msgs::Marker::LINE_LIST;

                edges.scale.x          = 0.2;
                edges.scale.y          = 0.2;

                edges.color.a          = 1;
                edges.color.g          = 1;

                for (auto node : nodeContainer)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = getX(&node);
                    p1.y = getY(&node);

                    nodes.points.push_back(p1);

                    if (node.parent == nullptr)
                         continue;

                    // edges.points.push_back(p1);
                    // p2.x = getX(node.parent);
                    // p2.y = getY(node.parent);
                    // edges.points.push_back(p2);

                    std::vector<geometry_msgs::Pose> poses;
                    trajectoryPose(node, *node.parent, poses);
                    poseToLinesRviz(poses, edges.points);
                }

                marker_pub.publish(nodes);
                marker_pub.publish(edges);
            }
        }

        void publishInfTrajectories(std::vector<Trajectory *> trajectories)
        {
            visualization_msgs::Marker edges;

            edges.header.frame_id  = map_frame_;
            edges.header.stamp     = ros::Time::now();
            edges.ns               = "infTrajectories";
            edges.id               = INF_TRAJ_ID; 
            edges.action           = visualization_msgs::Marker::ADD;
            edges.type             = visualization_msgs::Marker::LINE_LIST;

            edges.scale.x          = 0.01;
            edges.scale.y          = 0.01;

            edges.color.a          = 1;
            edges.color.r          = 1;

            for(auto traj : trajectories)
            {
                geometry_msgs::Point p1, p2;
                p1.x = getX(traj->source);
                p1.y = getY(traj->source);
                
                p2.x = getX(traj->target);
                p2.y = getY(traj->target);

                edges.points.push_back(p1);
                edges.points.push_back(p2);
            }

            marker_pub.publish(edges);
        }

    private:

        void trajectoryPose(rrt::Node v, rrt::Node u, std::vector<geometry_msgs::Pose> &poses)
        {
            double dist = si_->distance(v.state, u.state);
            auto ss = si_->getStateSpace()->as<ompl::base::ReedsSheppStateSpace>();
            double t = 1 / (dist * 4);
            ompl::base::ReedsSheppStateSpace::ReedsSheppPath path;
            bool firstTime = true;

            // std::cout << "dist " << dist << std::endl;
            // std::cout << "t " << t << std::endl;


            for(double i = 0; i < 1; i += t)
            {
                // std::cout << "i: " << i << std::endl;

                geometry_msgs::Pose p;
                ompl::base::ReedsSheppStateSpace::StateType *state = ss->allocState()->as<ompl::base::ReedsSheppStateSpace::StateType>();

                ss->interpolate(v.state, u.state, i, firstTime, path, state);
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

        ompl::base::SpaceInformationPtr si_;

        std::string map_frame_;

    }; // class RRTxPublisher


}; // namespace rrt

#endif