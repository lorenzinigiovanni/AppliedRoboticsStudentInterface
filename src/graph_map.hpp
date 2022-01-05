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
#include <iomanip>
#include <stdexcept>
#include <sstream>
#include <map>
#include <set>
#include <vector>
#include <algorithm>
#include <math.h>
#include <string>

class GraphMap
{
    // 1) How to store edges 2) How to store vertices 3) Type of graph (directed, undirected, bidirectional)
    // 4) Vertex property 5) Edge property type
    typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

public:
    enum PointType
    {
        GATE,
        WAYPOINT,
        PURSUER,
        ESCAPER,
    };

private:
    class VertexProperty
    {
    public:
        Point point;
        PointType type;

        VertexProperty() {}
        VertexProperty(Point point, PointType type) : point(point), type(type) {}
    };

    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> GraphType;

    GraphType graph;

    boost::property_map<GraphType, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight, graph);

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
                    float d1 = distance_btw_points(triangle_baricenter, triangles_middlepoint);
                    float d2 = distance_btw_points(neighbor_baricenter, triangles_middlepoint);
                    std::pair<GraphType::edge_descriptor, bool> e1 = boost::add_edge(triangle_baricenter, triangles_middlepoint, {d1}, graph);
                    std::pair<GraphType::edge_descriptor, bool> e2 = boost::add_edge(neighbor_baricenter, triangles_middlepoint, {d2}, graph);
                }
            }
        }
    }

    void add_gates(const std::vector<Polygon> &gates)
    {
        for (size_t gate = 0; gate < gates.size(); gate++)
        {
            GraphType::vertex_descriptor gate_center = add_rectangle_center(gates[gate]);
            GraphType::vertex_descriptor near_waypoint;

            float tmpDistance = 1000.0;

            boost::graph_traits<GraphType>::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
            {
                if (graph[*v].type == WAYPOINT)
                {
                    float distance = distance_btw_points(*v, gate_center);
                    if (distance < tmpDistance)
                    {
                        tmpDistance = distance;
                        near_waypoint = *v;
                    }
                }
            }

            std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(gate_center, near_waypoint, {tmpDistance}, graph);
        }
    }

    void add_robots(const std::vector<float> x, const std::vector<float> y)
    {
        for (int i = 0; i < 2; i++) // Robot number
        {
            PointType robot_type;
            if (i == 0)
            {
                robot_type = PURSUER;
            }
            else
            {
                robot_type = ESCAPER;
            }

            VertexProperty property(Point(x[i], y[i]), robot_type);
            GraphType::vertex_descriptor robot_center = boost::add_vertex(property, graph);
            GraphType::vertex_descriptor near_waypoint;

            float tmpDistance = 1000.0;

            boost::graph_traits<GraphType>::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
            {
                if (graph[*v].type == WAYPOINT)
                {
                    float distance = distance_btw_points(*v, robot_center);
                    if (distance < tmpDistance)
                    {
                        tmpDistance = distance;
                        near_waypoint = *v;
                    }
                }
            }

            if (tmpDistance < 0.05)
            {
                graph[near_waypoint].type = robot_type;
                boost::remove_vertex(robot_center, graph);
            }
            else
            {
                std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(robot_center, near_waypoint, {tmpDistance}, graph);
            }
        }
    }

    void optimize(std::vector<Polygon> obstacles)
    {
        GraphType copia = graph;

        boost::graph_traits<GraphType>::vertex_iterator v1, v1_end;
        for (boost::tie(v1, v1_end) = boost::vertices(copia); v1 != v1_end; ++v1)
        {
            boost::graph_traits<GraphType>::adjacency_iterator near_vertex_i, near_vertex_i_end;
            for (tie(near_vertex_i, near_vertex_i_end) = boost::adjacent_vertices(*v1, copia); near_vertex_i != near_vertex_i_end; ++near_vertex_i)
            {
                if (*v1 != *near_vertex_i)
                {
                    boost::graph_traits<GraphType>::adjacency_iterator dist_2_vertex_i, dist_2_vertex_i_end;
                    for (tie(dist_2_vertex_i, dist_2_vertex_i_end) = boost::adjacent_vertices(*near_vertex_i, copia); dist_2_vertex_i != dist_2_vertex_i_end; ++dist_2_vertex_i)
                    {
                        if (*v1 != *dist_2_vertex_i)
                        {
                            bool intersect = false;
                            for (int i = 0; i < obstacles.size(); i++)
                            {
                                for (int j = 0; j < obstacles[i].size(); j++)
                                {
                                    if (isIntersecting(graph[*v1].point, graph[*dist_2_vertex_i].point, obstacles[i][j], obstacles[i][(j + 1) % obstacles[i].size()]))
                                    {
                                        intersect = true;
                                        goto label;
                                    }
                                }
                            }
                            if (!intersect)
                            {
                                std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(*v1, *dist_2_vertex_i, {distance_btw_points(*v1, *dist_2_vertex_i)}, graph);
                                goto label;
                            }

                        label:
                            (void)0;
                        }
                    }
                }
            }
        }
    }

    bool isIntersecting(Point &p1, Point &p2, Point &q1, Point &q2)
    {
        return (((q1.x - p1.x) * (p2.y - p1.y) - (q1.y - p1.y) * (p2.x - p1.x)) * ((q2.x - p1.x) * (p2.y - p1.y) - (q2.y - p1.y) * (p2.x - p1.x)) < 0) &&
               (((p1.x - q1.x) * (q2.y - q1.y) - (p1.y - q1.y) * (q2.x - q1.x)) * ((p2.x - q1.x) * (q2.y - q1.y) - (p2.y - q1.y) * (q2.x - q1.x)) < 0);
    }

    void show_graph(cv::Mat &img)
    {
        boost::graph_traits<GraphType>::edge_iterator e, e_end;
        for (boost::tie(e, e_end) = boost::edges(graph); e != e_end; ++e)
        {
            int x1 = int(graph[e->m_source].point.x * 500) + 50;
            int y1 = img.size().height - int(graph[e->m_source].point.y * 500) - 50;
            int x2 = int(graph[e->m_target].point.x * 500) + 50;
            int y2 = img.size().height - int(graph[e->m_target].point.y * 500) - 50;

            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(100, 100, 255), 2);
        }

        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            int x = int(graph[*v].point.x * 500) + 50;
            int y = img.size().height - int(graph[*v].point.y * 500) - 50;

            cv::Scalar color = cv::Scalar(255, 255, 255);

            if (graph[*v].type == GATE)
            {
                color = cv::Scalar(4, 255, 4);
            }
            else if (graph[*v].type == WAYPOINT)
            {
                color = cv::Scalar(0, 0, 255);
            }
            else if (graph[*v].type == PURSUER)
            {
                color = cv::Scalar(255, 83, 27);
            }
            else if (graph[*v].type == ESCAPER)
            {
                color = cv::Scalar(0, 200, 255);
            }

            cv::putText(img, std::to_string(*v), cv::Point(x - 10, y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);
            cv::circle(img, cv::Point(x, y), 5, color, cv::FILLED);
        }
    }

    std::string get_locations()
    {
        std::string data;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type != GATE)
            {
                data += "l" + std::to_string(i) + " ";
            }
        }

        data += "- waypoint";

        return data;
    }

    std::string get_robots_locations()
    {
        // (at r1 l0)
        std::string data;

        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            if (graph[*v].type == PURSUER)
            {
                data += "(at r1 l" + std::to_string(*v) + ")\n";
            }
            else if (graph[*v].type == ESCAPER)
            {
                data += "(at r2 l" + std::to_string(*v) + ")\n";
            }
        }
        return data;
    }

    std::string get_gate_locations()
    {
        std::string data;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == GATE)
            {
                data += "l" + std::to_string(i) + " ";
            }
        }

        data += "- gate";

        return data;
    }

    std::map<int, float> get_robot_gate_distances(PointType robot)
    {
        std::vector<GraphType::vertex_descriptor> gate_indexes;
        std::map<int, float> distances;
        GraphType::vertex_descriptor robot_index;

        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            if (graph[*v].type == robot)
            {
                robot_index = *v;
            }
            else if (graph[*v].type == GATE)
            {
                gate_indexes.push_back(*v);
            }
        }

        for (int i = 0; i < gate_indexes.size(); i++)
        {
            distances[gate_indexes[i]] = (int)(distance_btw_points(robot_index, gate_indexes[i]) * 1000);
        }

        return distances;
    }

    std::string get_random_gate()
    {
        std::string data;
        std::vector<int> gate_indexes;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == GATE)
            {
                gate_indexes.push_back(i);
            }
        }

        srand(time(NULL));
        int random = rand() % gate_indexes.size();

        data += "l" + std::to_string(gate_indexes[random]);

        return data;
    }

    std::string get_locations_relations()
    {
        std::string data;

        boost::graph_traits<GraphType>::edge_iterator e, e_end;
        for (boost::tie(e, e_end) = boost::edges(graph); e != e_end; ++e)
        {
            int distance = (int)(EdgeWeightMap[*e] * 1000);
            data += "(near l" + std::to_string(e->m_source) + " l" + std::to_string(e->m_target) + ")\n";
            data += "(= (distance l" + std::to_string(e->m_source) + " l" + std::to_string(e->m_target) + ") " + std::to_string(distance) + ")\n";
            data += "(= (distance l" + std::to_string(e->m_target) + " l" + std::to_string(e->m_source) + ") " + std::to_string(distance) + ")\n";
        }

        return data;
    }

    std::string get_missing_escaper_cost_locations(std::vector<int> escaper_index_path)
    {
        std::string data;

        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            bool inserted = false;
            for (int i = 0; i < escaper_index_path.size(); i++)
            {
                if (*v == escaper_index_path[i])
                {
                    inserted = true;
                }
            }
            if (inserted != true)
            {
                data += "(= (escaper-cost l" + std::to_string(*v) + ") -1)\n";
            }
        }

        return data;
    }

    Point point_from_index(int i)
    {
        return graph[i].point;
    }

    float distance_btw_points(GraphType::vertex_descriptor v1, GraphType::vertex_descriptor v2)
    {
        return sqrt(pow(graph[v2].point.x - graph[v1].point.x, 2) + pow(graph[v2].point.y - graph[v1].point.y, 2));
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

    Point get_rectangle_center(Point &p1, Point &p2, Point &p3, Point &p4)
    {
        Point p;
        p.x = (p1.x + p2.x + p3.x + p4.x) / 4;
        p.y = (p1.y + p2.y + p3.y + p4.y) / 4;
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
            VertexProperty property(neighbor_line_center, WAYPOINT);
            line_baricenter = boost::add_vertex(property, graph);
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
            VertexProperty property(triangle_center, WAYPOINT);
            triangle_baricenter = boost::add_vertex(property, graph);
            triangle_vertex_map[index] = triangle_baricenter;
        }
        else
        {
            triangle_baricenter = triangle_vertex_map[index];
        }

        return triangle_baricenter;
    }

    GraphType::vertex_descriptor add_rectangle_center(const std::vector<Point> &points)
    {
        Point p1 = points[0];
        Point p2 = points[1];
        Point p3 = points[2];
        Point p4 = points[3];

        GraphType::vertex_descriptor rectangle_baricenter;

        Point rectangle_center = get_rectangle_center(p1, p2, p3, p4);
        VertexProperty property(rectangle_center, GATE);
        rectangle_baricenter = boost::add_vertex(property, graph);

        return rectangle_baricenter;
    }
};
