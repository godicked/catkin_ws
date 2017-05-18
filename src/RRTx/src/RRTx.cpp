#include "RRTx.hpp"
#include <math.h>

using namespace std;
using namespace boost;




namespace rrt
{
    
    RRTx::RRTx(costmap_2d::Costmap2D *costmap)
    {
        costmap_ = costmap;
    }

    
    void RRTx::addVertex(Node v)
    {
        v.self = add_vertex(G_s);
        vhash[v.self] = v; 
        
        point v_point(v.x, v.y);
        rtree.insert(make_pair(v_point, v.self));
    }

    vector<Node> RRTx::inN(Node v)
    {
        auto edges_s = inN_s(v);
        auto edges_r = inN_r(v);

        edges_s.insert(std::end(edges_s), std::begin(edges_r), std::end(edges_r));
        return edges_s;
    }

    vector<Node> RRTx::outN(Node v)
    {
        auto edges_s = outN_s(v);
        auto edges_r = outN_r(v);

        edges_s.insert(std::end(edges_s), std::begin(edges_r), std::end(edges_r));
        return edges_s;
    }

    vector<Node> RRTx::inN__(Node v, Graph g)
    {
        Graph::in_edge_iterator it, end;
        tie(it, end) = in_edges(v.self, g);
        
        vector<Node> neighbours;
        for(; it != end; it++)
        {
            auto u = source(*it, g);
            Node u_data = vhash[u]; 
            neighbours.push_back(u_data);
        }
        
        return neighbours;
    }

    vector<Node> RRTx::outN__(Node v, Graph g)
    {
        Graph::out_edge_iterator it, end;
        tie(it, end) = out_edges(v.self, g);
        
        vector<Node> neighbours;
        for(; it != end; it++)
        {
            auto u = target(*it, g);
            Node u_data = vhash[u]; 
            neighbours.push_back(u_data);
        }
        
        return neighbours;
    }

    vector<Node> RRTx::inN_r(Node v)
    {
        return inN__(v, G_r);
    }

    vector<Node> RRTx::inN_s(Node v)
    {
        return inN__(v, G_s);
    }

    vector<Node> RRTx::outN_r(Node v)
    {
        return outN__(v, G_r);
    }

    vector<Node> RRTx::outN_s(Node v)
    {
        return outN__(v, G_s);
    }

    void RRTx::addEdge_r(Node v, Node u)
    {
        add_edge(v.self, u.self, G_r);
    }

    void RRTx::addEdge_s(Node v, Node u)
    {
        add_edge(v.self, u.self, G_s);
    }

    Node RRTx::parent(Node v)
    {
        return v; vhash[v.parent];
    }

    Node RRTx::nearest(Node v)
    {
        vector<value> result;
        point p(v.x, v.y);

        rtree.query(bgi::nearest(p, 1), back_inserter(result));
        auto vertex = result[0].second;

        return vhash[vertex]; 
    }

    vector<Node> RRTx::near(Node v, float radius)
    {
        vector<value> search;
        point p1(max((float)0, v.x - radius), max((float)0, v.y - radius)), 
              p2(v.x + radius, v.y + radius);
        box b(p1, p2);

        vector<Node> nodes;
        rtree.query(bgi::intersects(b), back_inserter(search));
        for(int i = 0; i < nodes.size(); i++)
        {
            auto vertex = search[i].second;
            Node node   = vhash[vertex];

            if(distance(node, v) > radius)
                continue;

            nodes.push_back(node);
        }

        return nodes;
    }

    float RRTx::distance(Node v, Node u)
    {
        float dx2 = (v.x - u.x) * (v.x - u.x);
        float dy2 = (v.y - u.y) * (v.y - u.y);

        return sqrt(dx2 + dy2);
    }


    Node RRTx::saturate(Node v, Node u)
    {
        float dist = distance(v, u);

        float dx = (v.x - u.x) / dist;
        float dy = (v.y - u.y) / dist;

        v.x = u.x + dx * maxDist;
        v.y = u.y + dy * maxDist;

        return v;
    }

    void RRTx::findParent(Node &v, vector<Node> vnear)
    {
        
    }

    

};

int main()
{
    cout << "test " << endl;
    return 1;
}





