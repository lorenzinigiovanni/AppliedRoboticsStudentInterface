#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "CDT/CDT.h"
#include "boost/graph/adjacency_list.hpp"

#include <iterator>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <algorithm>

class GraphMap
{
    // 1) How to store edges 2) How to store vertices 3) Type of graph (directed, undirected, bidirectional)
    // 4) Vertex property 5) Edge property type
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, Point, boost::property<boost::edge_weight_t, float>> GraphType;
    GraphType graph;

    std::map<size_t, GraphType::vertex_descriptor> triangle_vertex_map;
    std::map<std::set<size_t>, GraphType::vertex_descriptor> line_vertex_map;

public:
    void create_graph(std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        // For each triangle in our vector of triangles
        for (size_t i = 0; i < triangles.size(); i++)
        {
            // Add triangle baricenter
            GraphType::vertex_descriptor triangle_baricenter;
            if (triangle_vertex_map.find(i) == triangle_vertex_map.end())
            {
                Point triangle_center = get_triangle_center(points[triangles[i].vertices[0]], points[triangles[i].vertices[1]], points[triangles[i].vertices[2]]);
                triangle_baricenter = boost::add_vertex(triangle_center, graph);
                triangle_vertex_map[i] = triangle_baricenter;
            }
            else
            {
                triangle_baricenter = triangle_vertex_map[i];
            }

            // For each neighbour of the triangle
            for (int j = 0; j < triangles[i].neighbors.size(); j++)
            {
                size_t neighbor = triangles[i].neighbors[j];

                // If there is a valid neighbouring triangle
                if (neighbor != 4294967295)
                {
                    // Add neighbour baricenter, if...
                    GraphType::vertex_descriptor neighbor_triangle_baricenter;
                    if (triangle_vertex_map.find(neighbor) == triangle_vertex_map.end())
                    {
                        Point neighbor_center = get_triangle_center(points[triangles[neighbor].vertices[0]], points[triangles[neighbor].vertices[1]], points[triangles[neighbor].vertices[2]]);
                        neighbor_triangle_baricenter = boost::add_vertex(neighbor_center, graph);
                        triangle_vertex_map[neighbor] = neighbor_triangle_baricenter;
                    }
                    else
                    {
                        neighbor_triangle_baricenter = triangle_vertex_map[neighbor];
                    }

                    // Find common points between triangles[i] and triangles[neighbour] (Comparison)
                    std::vector<size_t> common_points;
                    for (int k = 0; k < 3; k++)
                    {
                        for (int l = 0; l < 3; l++)
                        {
                            if (triangles[i].vertices[k] == triangles[neighbor].vertices[l])
                            {
                                common_points.push_back(triangles[i].vertices[k]);
                            }
                        }
                    }

                    std::set<size_t> triangle_neighbor_set = {i, neighbor};

                    // If there's not already a point, add it
                    GraphType::vertex_descriptor triangles_middlepoint;
                    if (line_vertex_map.find(triangle_neighbor_set) == line_vertex_map.end())
                    {
                        Point neighbor_line_center = get_line_center(points[common_points[0]], points[common_points[1]]);
                        triangles_middlepoint = boost::add_vertex(neighbor_line_center, graph);
                        line_vertex_map[triangle_neighbor_set] = triangles_middlepoint;
                    }
                    else
                    {
                        // Add midpoint
                        triangles_middlepoint = line_vertex_map[triangle_neighbor_set];
                    }

                    // Add edges, the second parameter returns the result of the operation
                    std::pair<GraphType::edge_descriptor, bool> e1 = boost::add_edge(triangle_baricenter, triangles_middlepoint, graph);
                    std::pair<GraphType::edge_descriptor, bool> e2 = boost::add_edge(neighbor_triangle_baricenter, triangles_middlepoint, graph);
                }
            }
        }
    }

    void show_graph(cv::Mat &img)
    {
        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
           int x = int(graph[*v].x * 500) + 50;
            int y = img.size().height - int(graph[*v].y * 500) - 50;

            cv::circle(img, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), cv::FILLED);
        }

        boost::graph_traits<GraphType>::edge_iterator e, e_end;
        for (boost::tie(e, e_end) = boost::edges(graph); e != e_end; ++e)
        {
            int x1 = int(graph[e->m_source].x * 500) + 50;
            int y1 = img.size().height - int(graph[e->m_source].y * 500) - 50;
            int x2 = int(graph[e->m_target].x * 500) + 50;
            int y2 = img.size().height - int(graph[e->m_target].y * 500) - 50;

            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(50, 255, 50), 1);
        }
    }

    Point get_line_center(Point &p1, Point &p2)
    {
        Point p;
        p.x = (p1.x + p2.x) / 2;
        p.y = (p1.y + p2.y) / 2;
        return p;
    }

    Point get_triangle_center(Point &p1, Point &p2, Point &p3)
    {
        Point p;
        p.x = (p1.x + p2.x + p3.x) / 3;
        p.y = (p1.y + p2.y + p3.y) / 3;
        return p;
    }
};
