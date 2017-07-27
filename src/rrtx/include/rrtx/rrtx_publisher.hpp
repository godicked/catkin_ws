#ifndef RRTX_PUBLISHER_HPP
#define RRTX_PUBLISHER_HPP

#include <ros/ros.h>

#include <rrtx/rrtx_struct.hpp>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#define GOAL_ID 0
#define VBOT_ID 1
#define PATH_NODE_ID 2
#define PATH_EDGE_ID 3
#define TREE_NODE_ID 4
#define TREE_EDGE_ID 5
#define INF_TRAJ_ID 6


namespace rrt
{
    class RRTxPublisher
    {
        public:
        RRTxPublisher()
        {
        }

        void initialize(ros::NodeHandle *ros_node, std::string map_frame)
        {
            nh = ros_node;
            map_frame_ = map_frame;
            marker_pub = nh->advertise<visualization_msgs::Marker>("tree", 1000);
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
            goal.scale.x               = 0.1;
            goal.scale.y               = 0.1;

            goal.color.a               = 1;
            goal.color.r               = 1;

            geometry_msgs::Point pgoal;
            pgoal.x = goal_->x;
            pgoal.y = goal_->y;
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
            vbot.scale.x               = 0.1;
            vbot.scale.y               = 0.1;

            vbot.color.a               = 1;
            vbot.color.r               = 1;
            vbot.color.g               = 1;

            geometry_msgs::Point pvbot;
            pvbot.x = vbot_->x;
            pvbot.y = vbot_->y;
            pvbot.z = 1;
            vbot.points.push_back(pvbot);

            pvbot.x = vbot_->parent->x;
            pvbot.y = vbot_->parent->y;
            vbot.points.push_back(pvbot);

            marker_pub.publish(vbot);

            visualization_msgs::Marker nodes, edges;

            nodes.header.frame_id  = map_frame_;
            nodes.header.stamp     = ros::Time::now();
            nodes.ns               = "path";
            nodes.id               = PATH_NODE_ID; 
            nodes.action           = visualization_msgs::Marker::ADD;
            nodes.type             = visualization_msgs::Marker::POINTS;

            nodes.scale.x          = 0.02;
            nodes.scale.y          = 0.02;

            nodes.color.a          = 1;
            nodes.color.r          = 1;
            nodes.color.g          = 0.5;


            edges.header.frame_id  = map_frame_;
            edges.header.stamp     = ros::Time::now();
            edges.ns               = "path";
            edges.id               = PATH_EDGE_ID; 
            edges.action           = visualization_msgs::Marker::ADD;
            edges.type             = visualization_msgs::Marker::LINE_LIST;

            edges.scale.x          = 0.01;
            edges.scale.y          = 0.01;

            edges.color.a          = 1;
            edges.color.r          = 0.5;
            edges.color.g          = 1;


            std::vector<Node *> path;
            for(auto v = vbot_; v != nullptr; v = v->parent)
            {
                path.push_back(v);
            }

            for(int i = 0; i < path.size(); i++)
            {
                geometry_msgs::Point p;
                p.x = path[i]->x;
                p.y = path[i]->y;

                nodes.points.push_back(p);

                if (i < path.size() - 1)
                {
                    edges.points.push_back(p);
                    p.x = path[i+1]->x;
                    p.y = path[i+1]->y;
                    edges.points.push_back(p);
                }

            }

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

                nodes.scale.x          = 0.05;
                nodes.scale.y          = 0.05;

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

                edges.scale.x          = 0.02;
                edges.scale.y          = 0.02;

                edges.color.a          = 1;
                edges.color.g          = 1;

                for (auto node : nodeContainer)
                {
                    geometry_msgs::Point p1, p2;
                    p1.x = node.x;
                    p1.y = node.y;

                    nodes.points.push_back(p1);

                    if (node.parent == nullptr)
                        continue;

                    edges.points.push_back(p1);
                    p2.x = node.parent->x;
                    p2.y = node.parent->y;
                    edges.points.push_back(p2);
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
                p1.x = traj->source->x;
                p1.y = traj->source->y;
                
                p2.x = traj->target->x;
                p2.y = traj->target->y;

                edges.points.push_back(p1);
                edges.points.push_back(p2);
            }

            marker_pub.publish(edges);
        }

        private:
        ros::NodeHandle *nh;
        ros::Publisher marker_pub;
        ros::Publisher path_pub;

        std::string map_frame_;

    }; // class RRTxPublisher


}; // namespace rrt

#endif