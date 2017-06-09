#ifndef SPLINE_CURVE_HPP
#define SPLINE_CURVE_HPP

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

#define SEGMENT_SIZE 5
#define ORDER 3

namespace rrt
{

    typedef geometry_msgs::Point Point;
    typedef geometry_msgs::Pose Pose;


    double U[SEGMENT_SIZE + ORDER + 1] = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};

    double N(unsigned int i, unsigned int p, double u)
    {
        if(u == 1 && i == SEGMENT_SIZE-1)
            return 1.0;

        if(p == 0)
        {
            if(U[i] <= u && u < U[i+1] && U[i] < U[i+1])
                return 1.0;
            return 0.0;
        }

        double term1 = u - U[i];
        double term2 = U[i+p] - U[i];
        if(term2 == 0)
        {
            term1 = 0;
            term2 = 1;
        }

        double term3 = U[i+p+1] - u;
        double term4 = U[i+p+1] - U[i+1];
        if(term4 == 0)
        {
            term3 = 0;
            term4 = 1;
        }

        return (term1 / term2) * N(i, p-1, u) + (term3 / term4) * N(i+1, p-1, u);
    }

    std::vector<Point> get_segment(std::vector<Pose> path, unsigned int offset)
    {
        std::vector<Point> segment;
        for(int i = offset; i < offset + SEGMENT_SIZE; i++)
        {
            segment.push_back(path[i].position);
        }
        return segment;
    }

    Point curve_point(std::vector<Point> segment, double u)
    {
        Point p;
        p.x = 0;
        p.y = 0;

        for(unsigned int i = 0; i < SEGMENT_SIZE; i++)
        {
            p.x += N(i, ORDER, u) * segment[i].x;
            p.y += N(i, ORDER, u) * segment[i].y;
        }

        return p;
    }

    void insert_midpoints(std::vector<Pose> &path)
    {
        for(auto it = path.begin(); it != path.end() - 1; it++)
        {
            Pose a = *it;
            Pose b = *(it++);

            Pose mid;
            mid.position.x = (a.position.x + b.position.x) / 2.0;
            mid.position.y = (a.position.y + b.position.y) / 2.0;

            it = path.insert(it++, mid);
        }
    }
    
    std::vector<Pose> curve_path(std::vector<Pose> path, double resolution)
    {
        insert_midpoints(path);

        Pose pose = path.back();

        std::vector<Pose> curved_path;
        std::vector<Point> segment;

        for(unsigned int i = 0; i < path.size() - (SEGMENT_SIZE-1) ; i += 2)
        {
            segment = get_segment(path, i);

            double u_min = 0.25, u_max = 0.75;

            if(i == 0)
                u_min = 0.0;
            if(i == path.size() - SEGMENT_SIZE)
                u_max = 1.0;

            for(double u = u_min; u < u_max; u += resolution / 4)
            {
                pose.position = curve_point(segment, u);
                curved_path.push_back(pose);
            }

            if(u_max == 1.0)
            {
                pose.position = curve_point(segment, u_max);
                curved_path.push_back(pose);
            }
            
        }

        return curved_path;
    }



}; //namespace rrt
#endif 
