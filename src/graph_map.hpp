#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "CDT/CDT.h"
#include "boost/graph/adjacency_list.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>

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
        for (size_t triangle = 0; triangle < triangles.size(); triangle++)
        {
            GraphType::vertex_descriptor triangle_baricenter = add_triangle_center(triangle, triangles, points);

            // For each neighbour of the triangle
            for (int j = 0; j < triangles[triangle].neighbors.size(); j++)
            {
                size_t neighbor = triangles[triangle].neighbors[j];

                // If there is a valid neighbouring triangle
                if (neighbor != 4294967295)
                {
                    GraphType::vertex_descriptor neighbor_baricenter = add_triangle_center(neighbor, triangles, points);

                    GraphType::vertex_descriptor triangles_middlepoint = add_line_between_triangles(triangle, neighbor, triangles, points);

                    // Add edges, the second parameter returns the result of the operation
                    std::pair<GraphType::edge_descriptor, bool> e1 = boost::add_edge(triangle_baricenter, triangles_middlepoint, graph);
                    std::pair<GraphType::edge_descriptor, bool> e2 = boost::add_edge(neighbor_baricenter, triangles_middlepoint, graph);
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

private:
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

    GraphType::vertex_descriptor add_line_between_triangles(size_t t1, size_t t2, std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        std::vector<size_t> common_points;

        for (int k = 0; k < 3; k++)
        {
            for (int l = 0; l < 3; l++)
            {
                if (triangles[t1].vertices[k] == triangles[t2].vertices[l])
                {
                    common_points.push_back(triangles[t1].vertices[k]);
                }
            }
        }

        std::set<size_t> triangle_neighbor_set = {t1, t2};

        GraphType::vertex_descriptor triangles_middlepoint = add_line_center(triangle_neighbor_set, points[common_points[0]], points[common_points[1]]);
        return triangles_middlepoint;
    }

    GraphType::vertex_descriptor add_line_center(std::set<size_t> index, Point &p1, Point &p2)
    {
        GraphType::vertex_descriptor line_baricenter;

        if (line_vertex_map.find(index) == line_vertex_map.end())
        {
            Point neighbor_line_center = get_line_center(p1, p2);
            line_baricenter = boost::add_vertex(neighbor_line_center, graph);
            line_vertex_map[index] = line_baricenter;
        }
        else
        {
            line_baricenter = line_vertex_map[index];
        }

        return line_baricenter;
    }

    GraphType::vertex_descriptor add_triangle_center(size_t index, std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        Point p1 = points[triangles[index].vertices[0]];
        Point p2 = points[triangles[index].vertices[1]];
        Point p3 = points[triangles[index].vertices[2]];

        GraphType::vertex_descriptor triangle_baricenter;

        if (triangle_vertex_map.find(index) == triangle_vertex_map.end())
        {
            Point triangle_center = get_triangle_center(p1, p2, p3);
            triangle_baricenter = boost::add_vertex(triangle_center, graph);
            triangle_vertex_map[index] = triangle_baricenter;
        }
        else
        {
            triangle_baricenter = triangle_vertex_map[index];
        }

        return triangle_baricenter;
    }
};
