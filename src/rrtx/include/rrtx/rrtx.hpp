#ifndef RRTX_HPP
#define RRTX_HPP

#include <iostream>

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>


#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Polygon.h>
#include <tf/tf.h>

#include "spline_curve.hpp"

#include <ros/ros.h>

#include <rrtx/rrtx_struct.hpp>
#include <rrtx/kinematic_path_builder.hpp>
#include <rrtx/rrtx_publisher.hpp>


namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;


using namespace boost;

namespace rrt
{

class RRTx
{
    typedef bg::model::point<double, 2, bg::cs::cartesian> point;
    typedef bg::model::segment<point> segment;
    typedef bg::model::box<point> box;

    typedef std::pair<point, Node *> RTreePoint;
    typedef bgi::rtree<RTreePoint, bgi::rstar<16> > NodeRTree;


    typedef boost::heap::fibonacci_heap<Node *, 
            boost::heap::compare<node_compare> > Queue;
    typedef Queue::handle_type handle_t;

    typedef boost::unordered_map<Node *, handle_t> NodeHash;
    typedef boost::unordered_map<Node *, bool> OrphanHash;


    public:

        typedef std::list<Node> NodeContainer;
        typedef std::vector<geometry_msgs::Pose> Path;
    
        
                            RRTx            ();
                            RRTx            (costmap_2d::Costmap2D *costmap);
                            ~RRTx           (){}

        void                setCostmap      (costmap_2d::Costmap2D *costmap);
        void                setMaxDist      (double max_dist);
        void                init            (geometry_msgs::Pose start,
                                             geometry_msgs::Pose goal);
        void                init            (double sx, double sy, double stheta, double gx, double gy);
        void                grow            (unsigned int iteration);
        void                updateTree      (double origin_x, double origin_y, double size_x, double size_y);
        Node                rootNode        ();
        NodeContainer       getContainer    ();
        bool                computePath     (Path &path);
        void                setConstraint   (double steering_angle, double wheelbase);
        void                publish     (bool path = true, bool tree = false);
        void                updateRobot (geometry_msgs::Pose robot);

    private:
        
        void                addVertex       (Node *v);
        

        std::vector<Node *> inN             (Node v);
        std::vector<Node *> outN            (Node v);
        void                removeN         (std::vector<Node *> * vec, Node *u);
        
       
        Node               *nearest         (Node v);
        std::vector<Node *> near            (Node v);
        double              distance        (Node v, Node u);
        Node                saturate        (Node v, Node u);
        void                findParent      (Node *v, std::vector<Node *>);
        std::vector<Node *> findTrajectories(Node *v, std::vector<Node *> nodes);
        bool                trajectoryExist (Node *v, Node *u);
        void                extend          (Node *v);
        void                cullNeighbors   (Node *v);
        void                verrifyQueue    (Node *v);
        void                rewireNeighbors (Node *v);
        void                reduceInconsist ();
        void                updateLMC       (Node *v);
        void                updateRadius    ();
        Node                randomNode      ();
        bool                isObstacle      (Node v);
        bool                isOutOfBound    (unsigned int mx, unsigned int my);
        void                grow            ();
        double              getAngle        (Node a, Node b, Node c);
        double              cost            (Node *a, Node *b);
        void                setCost         (Node *a, Node *b, double cost);
        Trajectory         *trajectory      (Node *a, Node *b);
        void                makeParentOf    (Node *parent, Node *v);
        Trajectory         *makeTrajectory  (Node *v, Node *u);
        void                insertValidTrajectory(Node *v, Node *u);

        //  Priority Queue related functions
        void                queueInsert     (Node *v);
        void                queueUpdate     (Node *v);
        void                queueRemove     (Node *v);
        bool                queueContains   (Node *v);
        void                updateKey       (Node *v);
        Node               *queuePop        ();

        // Dynamic part of RRTx
        void addObstacle(double origin_x, double origin_y, double size_x, double size_y);
        void findFreeTrajectories(costmap_2d::Costmap2D map);
        void propogateDescendants();
        void verrifyOrphan(Node *v);
        void insertOrphanChildren(Node *v);

        //  Member variables

        //  Container to hold the Node structure
        NodeContainer nodeContainer;
        //  Hash container for trajectories between neighbors
        TrajectoryHash trajectories;

        //  R* Tree for efficient spacial k nearest neighbors (KNN) search
        //  and helps for the radius search
        NodeRTree   rtree;

        // Orphan Nodes after adding obstacles
        OrphanHash orphanHash;

        //  The priority queue needed by the RRTx algorithm
        //  And a NodeHash table to save the handle of the inserted object
        //  Allows contains/updade/remove operations
        Queue   queue;
        NodeHash    nodeHash;

        //  A costmap for the collision tests
        costmap_2d::Costmap2D *costmap_;

        //  max distance between 2 a new point and its nearest neighbor
        double maxDist;
        // min distance before first curve
        double minDist = 0;


        double  epsilon = 0.05;
        double  radius;
        double  y;

        Node    *vbot_;
        double  vbot_theta;
        Node    *goal_;

        std::vector<geometry_msgs::Point> smooth_path;

        ros::NodeHandle nh_;
        ros::Publisher  marker_pub;
        ros::Publisher  path_pub;

        RRTxPublisher publisher;

        std::string map_frame;

        BSplinePathSmoother smoother;
        KinematicPathBuilder builder;
        bool constraint = false;
        double kmax;

        std::vector<Node *> lastPath;
        geometry_msgs::Pose lastPose;

        std::vector<Trajectory *> infTrajectories;

};

};

#endif
