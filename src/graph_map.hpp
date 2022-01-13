#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "CDT/CDT.h"
#include "boost/graph/adjacency_list.hpp"
#include "settings.hpp"

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

/**
 * @brief Store the roadmap as a graph
 *
 */
class GraphMap
{
    /**
     * @brief Type of a vertex in the graph
     *
     */
    enum PointType
    {
        GATE,
        WAYPOINT,
        PURSUER,
        EVADER,
    };

    /**
     * @brief Store the informations of a vertex
     *
     */
    class VertexProperty
    {
    public:
        Point point;
        PointType type;

        VertexProperty() {}
        VertexProperty(Point point, PointType type) : point(point), type(type) {}
    };

    // Property to store the lenght of an egde, so the distance between two vertices
    typedef boost::property<boost::edge_weight_t, float> EdgeWeightProperty;

public:
    // Definition of the graph type
    // 1) How to store edges 2) How to store vertices 3) Type of graph (directed, undirected, bidirectional)
    // 4) Vertex property 5) Edge property type
    typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, VertexProperty, EdgeWeightProperty> GraphType;

private:
    GraphType graph; // The graph were all the vertices and edges are stored

    std::map<size_t, GraphType::vertex_descriptor> triangle_vertex_map;       // Map were the baricenter of the triangles are stored
    std::map<std::set<size_t>, GraphType::vertex_descriptor> line_vertex_map; // Map were the center of the lines between triangles are stored

    boost::property_map<GraphType, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight, graph); // Propery of the graph were edges lenght are stored

public:
    /**
     * @brief Fill the graph with the triangles baricenter and center of the lines between them
     *
     * @param triangles Triangles obtained from the CDT triangulation
     * @param points Coordinates of the vertexes of the triangles
     */
    void create_graph(std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        // For each triangle in our vector of triangles
        for (size_t triangle = 0; triangle < triangles.size(); triangle++)
        {
            // Get the baricenter of the triangle (add it to the graph if not already present)
            GraphType::vertex_descriptor triangle_baricenter = add_triangle_center(triangle, triangles, points);

            // For each neighbor of the triangle
            for (int j = 0; j < triangles[triangle].neighbors.size(); j++)
            {
                // Store the neighbor
                size_t neighbor = triangles[triangle].neighbors[j];

                // If there is a valid neighboring triangle
                if (neighbor != 4294967295)
                {
                    // Get the baricenter of the neighbor (add it to the graph if not already present)
                    GraphType::vertex_descriptor neighbor_baricenter = add_triangle_center(neighbor, triangles, points);

                    // Get the center point of the line between the two triangles (add it to the graph if not already present)
                    GraphType::vertex_descriptor triangles_middlepoint = add_line_between_triangles(triangle, neighbor, triangles, points);

                    // Compute the lenght of the edges between each triangle baricenter and the center of the line between them
                    float d1 = distance_btw_points(triangle_baricenter, triangles_middlepoint);
                    float d2 = distance_btw_points(neighbor_baricenter, triangles_middlepoint);

                    // Add edges to the graph with they lenght property
                    std::pair<GraphType::edge_descriptor, bool> e1 = boost::add_edge(triangle_baricenter, triangles_middlepoint, {d1}, graph);
                    std::pair<GraphType::edge_descriptor, bool> e2 = boost::add_edge(neighbor_baricenter, triangles_middlepoint, {d2}, graph);
                }
            }
        }
    }

    /**
     * @brief Add gates to the graph connecting it to the nearest vertex
     *
     * @param gates A list of gates to add to the graph
     * @param obstacles A list of obstacles to check for interference
     */
    void add_gates(const std::vector<Polygon> &gates, std::vector<Polygon> obstacles, std::vector<Polygon> borders)
    {
        double offset = ((double)Settings::offset) / 1000.0;

        // Search for the maximum and minimum coordinates of the borders
        double x_max = -1;
        double x_min = 1000;
        double y_max = -1;
        double y_min = 1000;

        for (int i = 0; i < borders.size(); i++)
        {
            for (int j = 0; j < borders[i].size(); j++)
            {
                // Since the borders have been offsetted, offset also the max min coordinates
                if (borders[i][j].x > x_max)
                {
                    x_max = borders[i][j].x - offset;
                }
                if (borders[i][j].x < x_min)
                {
                    x_min = borders[i][j].x + offset;
                }
                if (borders[i][j].y > y_max)
                {
                    y_max = borders[i][j].y - offset;
                }
                if (borders[i][j].y < y_min)
                {
                    y_min = borders[i][j].y + offset;
                }
            }
        }

        // For each gate
        for (size_t gate = 0; gate < gates.size(); gate++)
        {
            // Get the gate center (the gate is a rectangle)
            GraphType::vertex_descriptor gate_center = add_rectangle_center(gates[gate]);

            Point gc = graph[gate_center].point;

            Point offsetted_gate;
            offsetted_gate.x = gc.x;
            offsetted_gate.y = gc.y;

            bool inside_borders = false;

            // Check if the gate center is already inside the borders
            if ((gc.y < y_max) && (gc.y > y_min) && (gc.x < x_max) && (gc.x > x_min))
            {
                inside_borders = true;
            }
            else
            {
                // If gate center is over the borders shift down
                if (gc.y > y_max)
                {
                    offsetted_gate.y -= offset;
                }
                // If gate center is down the borders shift up
                else if (gc.y < y_min)
                {
                    offsetted_gate.y += offset;
                }
                // If gate center is right the borders shift left
                if (gc.x > x_max)
                {
                    offsetted_gate.x -= offset;
                }
                // If gate center is left the borders shift right
                else if (gc.x < x_min)
                {
                    offsetted_gate.x += offset;
                }
            }

            GraphType::vertex_descriptor point_to_add = gate_center;

            // If gate center is not inside the borders, create another junnction point by offsetting its value in the right direction
            if (!inside_borders)
            {
                VertexProperty property(offsetted_gate, WAYPOINT);

                // Add the vertex to the graph
                GraphType::vertex_descriptor offsetted_gate_vertex = boost::add_vertex(property, graph);

                float distance = distance_btw_points(gate_center, offsetted_gate_vertex);

                // Add an edge between the gate and the offsetted waypoint
                std::pair<GraphType::edge_descriptor, bool> e_vg = boost::add_edge(offsetted_gate_vertex, gate_center, {distance}, graph);

                point_to_add = offsetted_gate_vertex;
            }

            // Temporary variables to store the nearest vertex and its distance
            GraphType::vertex_descriptor near_waypoint = -1;
            float tmpDistance = 1000.0;

            // Iterate over each vertex of the graph
            boost::graph_traits<GraphType>::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
            {
                // Search for a waypoint that is near the gate/ofsetted gate
                if (graph[*v].type == WAYPOINT && *v != point_to_add)
                {
                    // Check for intersection with all obstacles for the edge between the gate/ofsetted gate and the waypoint
                    bool intersect = false;
                    for (int i = 0; i < obstacles.size(); i++)
                    {
                        if (intersect)
                        {
                            break;
                        }

                        for (int j = 0; j < obstacles[i].size(); j++)
                        {
                            // Check if there is an intersection between the edge connecting the current waypoint and the gate/ofsetted gate and the obstacles in-between
                            if (isIntersecting(graph[*v].point, graph[point_to_add].point, obstacles[i][j], obstacles[i][(j + 1) % obstacles[i].size()]))
                            {
                                intersect = true;
                                break;
                            }
                        }
                    }

                    // Keep the one that is the closest
                    float distance = distance_btw_points(*v, point_to_add);
                    if (!intersect && distance < tmpDistance)
                    {
                        tmpDistance = distance;
                        near_waypoint = *v;
                    }
                }
            }

            // Add an edge between the gate and the nearest waypoint
            if (near_waypoint != -1)
            {
                std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(point_to_add, near_waypoint, {tmpDistance}, graph);
            }
        }
    }

    /**
     * @brief Add robots to the graph connecting it to the nearest vertex
     *
     * @param x X coordinates of the robots
     * @param y Y coordinates of the robots
     */
    void add_robots(const std::vector<float> x, const std::vector<float> y)
    {
        // For each robot
        for (int i = 0; i < 2; i++) // Robot number
        {
            // Assign the right type to the vertex
            PointType robot_type;
            if (i == 0)
            {
                robot_type = PURSUER;
            }
            else
            {
                robot_type = EVADER;
            }

            // Add the robot vertex to the graph
            VertexProperty property(Point(x[i], y[i]), robot_type);
            GraphType::vertex_descriptor robot_center = boost::add_vertex(property, graph);

            // Temporary variables to store the nearest vertex and its distance
            GraphType::vertex_descriptor near_waypoint;
            float tmpDistance = 1000.0;

            boost::graph_traits<GraphType>::vertex_iterator v, v_end;
            for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
            {
                // Search for a waypoint or gate that is near the robot
                if (graph[*v].type == WAYPOINT)
                {
                    // Keep the one that is the closest
                    float distance = distance_btw_points(*v, robot_center);
                    if (distance < tmpDistance)
                    {
                        tmpDistance = distance;
                        near_waypoint = *v;
                    }
                }
            }

            // If the robot is very close to a vertex use the vertex as the robot position
            if (tmpDistance < 0.05)
            {
                graph[near_waypoint].type = robot_type;
                boost::remove_vertex(robot_center, graph);
            }
            // Otherwise make an edge between the robot and the closest vertex
            else
            {
                std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(robot_center, near_waypoint, {tmpDistance}, graph);
            }
        }
    }

    /**
     * @brief Optimize the graph map by connecting distance 2 vertexes that do not interfer with an obstacle
     *
     * @param obstacles A list of obstacles to check for interference
     */
    void optimize(std::vector<Polygon> obstacles)
    {
        // Store a copy of the graph so to cycle through it while modify the original one
        GraphType graph_copy = graph;

        // For each vertex of the graph
        boost::graph_traits<GraphType>::vertex_iterator vertex, vertex_end;
        for (boost::tie(vertex, vertex_end) = boost::vertices(graph_copy); vertex != vertex_end; ++vertex)
        {
            // For each vertex near to the first one
            boost::graph_traits<GraphType>::adjacency_iterator distance_1_vertex, distance_1_vertex_end;
            for (tie(distance_1_vertex, distance_1_vertex_end) = boost::adjacent_vertices(*vertex, graph_copy); distance_1_vertex != distance_1_vertex_end; ++distance_1_vertex)
            {
                // If the two vertexes are not the same
                if (*vertex != *distance_1_vertex)
                {
                    // For every vertex near to the second one
                    boost::graph_traits<GraphType>::adjacency_iterator distance_2_vertex, distance_2_vertex_end;
                    for (tie(distance_2_vertex, distance_2_vertex_end) = boost::adjacent_vertices(*distance_1_vertex, graph_copy); distance_2_vertex != distance_2_vertex_end; ++distance_2_vertex)
                    {
                        // If the two vertexes are not the same
                        if (*vertex != *distance_2_vertex && *distance_1_vertex != *distance_2_vertex)
                        {
                            // Check for intersection with all obstacles for the edge between vertex and distance 2 vertex
                            bool intersect = false;
                            for (int i = 0; i < obstacles.size(); i++)
                            {
                                for (int j = 0; j < obstacles[i].size(); j++)
                                {
                                    if (isIntersecting(graph[*vertex].point, graph[*distance_2_vertex].point, obstacles[i][j], obstacles[i][(j + 1) % obstacles[i].size()]))
                                    {
                                        intersect = true;
                                        goto label;
                                    }
                                }
                            }

                            // If the edge does not intersect with an obstacle add it to the graph
                            if (!intersect)
                            {
                                std::pair<GraphType::edge_descriptor, bool> e = boost::add_edge(*vertex, *distance_2_vertex, {distance_btw_points(*vertex, *distance_2_vertex)}, graph);
                            }

                        label:
                            (void)0;
                        }
                    }
                }
            }
        }
    }

    /**
     * @brief Check if two lines intersect
     *
     * @param p1 First point of the first line
     * @param p2 Second point of the first line
     * @param q1 First point of the second line
     * @param q2 Second point of the second line
     * @return true if the lines intersect
     * @return false if the lines do not intersect
     */
    bool isIntersecting(Point &p1, Point &p2, Point &q1, Point &q2)
    {
        return (((q1.x - p1.x) * (p2.y - p1.y) - (q1.y - p1.y) * (p2.x - p1.x)) * ((q2.x - p1.x) * (p2.y - p1.y) - (q2.y - p1.y) * (p2.x - p1.x)) < 0) &&
               (((p1.x - q1.x) * (q2.y - q1.y) - (p1.y - q1.y) * (q2.x - q1.x)) * ((p2.x - q1.x) * (q2.y - q1.y) - (p2.y - q1.y) * (q2.x - q1.x)) < 0);
    }

    /**
     * @brief Plot the graph onto an image
     *
     * @param img Image to show the graph on
     */
    void show_graph(cv::Mat &img)
    {
        // Cycle over the edges of the graph
        boost::graph_traits<GraphType>::edge_iterator e, e_end;
        for (boost::tie(e, e_end) = boost::edges(graph); e != e_end; ++e)
        {
            int x1 = int(graph[e->m_source].point.x * 500) + 50;
            int y1 = img.size().height - int(graph[e->m_source].point.y * 500) - 50;
            int x2 = int(graph[e->m_target].point.x * 500) + 50;
            int y2 = img.size().height - int(graph[e->m_target].point.y * 500) - 50;

            // Draw a line between the two vertexes connected by the edge (light red color)
            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(100, 100, 255), 2);
        }

        // Cycle over the vertexes of the graph
        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            int x = int(graph[*v].point.x * 500) + 50;
            int y = img.size().height - int(graph[*v].point.y * 500) - 50;

            cv::Scalar color = cv::Scalar(255, 255, 255);

            if (graph[*v].type == GATE)
            {
                // The gates are green colored
                color = cv::Scalar(4, 255, 4);
            }
            else if (graph[*v].type == WAYPOINT)
            {
                // The waypoints are red colored
                color = cv::Scalar(0, 0, 255);
            }
            else if (graph[*v].type == PURSUER)
            {
                // The pursuer is blue colored
                color = cv::Scalar(255, 83, 27);
            }
            else if (graph[*v].type == EVADER)
            {
                // The evader is yellow colored
                color = cv::Scalar(0, 150, 255);
            }

            // Put a text representing the vertex index near each vertex
            // cv::putText(img, std::to_string(*v), cv::Point(x - 10, y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 2);

            // Draw a circle representing the vertex
            cv::circle(img, cv::Point(x, y), 5, color, cv::FILLED);
        }
    }

    /**
     * @brief Get the indexes of the vertexes of type gate
     *
     * @return A vector containing the indexes of the vertexes of type gate
     */
    std::vector<int> get_gates_indexes()
    {
        std::vector<int> indexes;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == GATE)
            {
                indexes.push_back(i);
            }
        }

        return indexes;
    }

    /**
     * @brief Get indexes of the vertexes of type robot
     *
     * @return A vector containing the indexes of the vertexes of type robot
     */
    std::vector<int> get_robots_indexes()
    {
        std::vector<int> indexes;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == PURSUER || graph[i].type == EVADER)
            {
                indexes.push_back(i);
            }
        }

        return indexes;
    }

    /**
     * @brief Get the index of the vertex of type pursuer
     *
     * @return The index of the vertex of type pursuer
     */
    int get_pursuer_index()
    {
        int index;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == PURSUER)
            {
                index = i;
            }
        }

        return index;
    }

    /**
     * @brief Get the index of the vertex of type evader
     *
     * @return The index of the vertex of type evader
     */
    int get_evader_index()
    {
        int index;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == EVADER)
            {
                index = i;
            }
        }

        return index;
    }

    /**
     * @brief Get indexes of the vertexes of type waypoint
     *
     * @return A vector containing the indexes of the vertexes of type waypoint
     */
    std::vector<int> get_waypoint_indexes()
    {
        std::vector<int> indexes;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            if (graph[i].type == WAYPOINT)
            {
                indexes.push_back(i);
            }
        }

        return indexes;
    }

    /**
     * @brief Get indexes of all the vertexe
     *
     * @return A vector containing the indexes of all the vertexes
     */
    std::vector<int> get_locations_indexes()
    {
        std::vector<int> indexes;

        for (int i = 0; i < graph.m_vertices.size(); i++)
        {
            indexes.push_back(i);
        }

        return indexes;
    }

    /**
     * @brief Get the locations of the graph were the evader will not move into
     *
     * @param evader_index_path The path of the evader
     * @return A vector containing the index of the locations of the graph were the evader will not move into
     */
    std::vector<int> get_missing_evader_locations_indexes(std::vector<int> evader_index_path)
    {
        std::vector<int> indexes;

        // For each vertex of the graph
        boost::graph_traits<GraphMap::GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            bool inserted = false;

            // For each vertex of the evader path
            for (int i = 0; i < evader_index_path.size(); i++)
            {
                if (*v == evader_index_path[i])
                {
                    inserted = true;
                }
            }

            // If the vertex is not in the evader path add it to the return vector
            if (inserted != true)
            {
                indexes.push_back(*v);
            }
        }

        return indexes;
    }

    /**
     * @brief Get the coordinates of a vertex
     *
     * @param i Index of the vertex
     * @return Point object containing the coordinates of the vertex
     */
    Point point_from_index(int i)
    {
        return graph[i].point;
    }

    /**
     * @brief Get the distance between two vertexes
     *
     * @param v1 Index of the first vertex
     * @param v2 Index of the second vertex
     * @return Distance between the two vertexes
     */
    float distance_btw_points(GraphType::vertex_descriptor v1, GraphType::vertex_descriptor v2)
    {
        return sqrt(pow(graph[v2].point.x - graph[v1].point.x, 2) + pow(graph[v2].point.y - graph[v1].point.y, 2));
    }

    /**
     * @brief Get the robot distances between it and all the gates
     *
     * @param robot The type of the robot
     * @return A map with key the index of the gate and value the distance between the robot and the gate
     */
    std::map<int, float> get_robot_gate_distances(PointType robot)
    {
        std::vector<GraphType::vertex_descriptor> gate_indexes;
        std::map<int, float> distances;
        GraphType::vertex_descriptor robot_index;

        // For every vertex in the graph
        boost::graph_traits<GraphType>::vertex_iterator v, v_end;
        for (boost::tie(v, v_end) = boost::vertices(graph); v != v_end; ++v)
        {
            if (graph[*v].type == robot)
            {
                // If it is a robot store it in the robot index
                robot_index = *v;
            }
            else if (graph[*v].type == GATE)
            {
                // If it is a gate add it to the gate indexes
                gate_indexes.push_back(*v);
            }
        }

        // For every gate
        for (int i = 0; i < gate_indexes.size(); i++)
        {
            // Compute the distance between the robot and the gate and add it to the map
            distances[gate_indexes[i]] = (int)(distance_btw_points(robot_index, gate_indexes[i]) * 1000);
        }

        return distances;
    }

    /**
     * @brief Get the distances between all the connected vertexes
     *
     * @return A map with key the indexes of the connected vertexes and value the distance between them
     */
    std::map<std::pair<int, int>, int> get_locations_distances()
    {
        std::map<std::pair<int, int>, int> data;

        // For every vertex in the graph
        boost::graph_traits<GraphMap::GraphType>::edge_iterator e, e_end;
        for (boost::tie(e, e_end) = boost::edges(graph); e != e_end; ++e)
        {
            std::pair<int, int> locations;
            locations.first = e->m_source;
            locations.second = e->m_target;

            // Compute the distance
            int distance = (int)(GraphMap::EdgeWeightMap[*e] * 1000);

            // Add the informmation to the map
            data[locations] = distance;
        }

        return data;
    }

private:
    /**
     * @brief Get the center of a line
     *
     * @param p1 First point of the line
     * @param p2 Second point of the line
     * @return Point representing the center of the line
     */
    Point get_line_center(Point &p1, Point &p2)
    {
        Point p;
        p.x = (p1.x + p2.x) / 2;
        p.y = (p1.y + p2.y) / 2;
        return p;
    }

    /**
     * @brief Get the center of a triangle
     *
     * @param p1 First point of the triangle
     * @param p2 Second point of the triangle
     * @param p3 Third point of the triangle
     * @return Point representing the center of the triangle
     */
    Point get_triangle_center(Point &p1, Point &p2, Point &p3)
    {
        Point p;
        p.x = (p1.x + p2.x + p3.x) / 3;
        p.y = (p1.y + p2.y + p3.y) / 3;
        return p;
    }

    /**
     * @brief Get the center of a rectangle
     *
     * @param p1 First point of the rectangle
     * @param p2 Second point of the rectangle
     * @param p3 Third point of the rectangle
     * @param p4 Fourth point of the rectangle
     * @return Point representing the center of the rectangle
     */
    Point get_rectangle_center(Point &p1, Point &p2, Point &p3, Point &p4)
    {
        Point p;
        p.x = (p1.x + p2.x + p3.x + p4.x) / 4;
        p.y = (p1.y + p2.y + p3.y + p4.y) / 4;
        return p;
    }

    /**
     * @brief Find the shared edge between two triangles and add the center of it to the graph
     *
     * @param t1 First triangle
     * @param t2 Second triangle
     * @param triangles Triangles obtained from the CDT triangulation
     * @param points Coordinates of the vertexes of the triangles
     * @return The index of the added vertex
     */
    GraphType::vertex_descriptor add_line_between_triangles(size_t t1, size_t t2, std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        std::vector<size_t> common_points;

        // For every vertex of the first triangle
        for (int k = 0; k < 3; k++)
        {
            // For every vertex of the second triangle
            for (int l = 0; l < 3; l++)
            {
                // If the vertex is in common
                if (triangles[t1].vertices[k] == triangles[t2].vertices[l])
                {
                    // Add the vertex to the common points vector
                    common_points.push_back(triangles[t1].vertices[k]);
                }
            }
        }

        // Create a set of the indexes of the two triangles
        std::set<size_t> triangle_neighbor_set = {t1, t2};

        // Create a new edgein the center of the edge shared by the two triangles
        GraphType::vertex_descriptor triangles_middlepoint = add_line_center(triangle_neighbor_set, points[common_points[0]], points[common_points[1]]);

        // Return the index of the vertex
        return triangles_middlepoint;
    }

    /**
     * @brief Add the shared edge between two triangles to the graph
     *
     * @param index Set of the two triangles indexes
     * @param p1 First point of the shared edge
     * @param p2 Second point of the shared edge
     * @return The index of the added vertex
     */
    GraphType::vertex_descriptor add_line_center(std::set<size_t> index, Point &p1, Point &p2)
    {
        GraphType::vertex_descriptor line_baricenter;

        // If the vertex is not already in the graph
        if (line_vertex_map.find(index) == line_vertex_map.end())
        {
            // Get the point representing the center of the line
            Point neighbor_line_center = get_line_center(p1, p2);

            // Set the type of the vertex to waypoint
            VertexProperty property(neighbor_line_center, WAYPOINT);

            // Add the vertex to the graph
            line_baricenter = boost::add_vertex(property, graph);

            // Store the center of the shared edge in the map of the indexes of the two triangles
            line_vertex_map[index] = line_baricenter;
        }
        else
        {
            // Return the already present center of the shared edge
            line_baricenter = line_vertex_map[index];
        }

        // Return the index of the vertex
        return line_baricenter;
    }

    /**
     * @brief Find the baricenter of a triangle and add it to the graph
     *
     * @param index Triangle index
     * @param triangles Triangles obtained from the CDT triangulation
     * @param points Coordinates of the vertexes of the triangles
     * @return The index of the added vertex
     */
    GraphType::vertex_descriptor add_triangle_center(size_t index, std::vector<CDT::Triangle> &triangles, std::vector<Point> &points)
    {
        Point p1 = points[triangles[index].vertices[0]];
        Point p2 = points[triangles[index].vertices[1]];
        Point p3 = points[triangles[index].vertices[2]];

        GraphType::vertex_descriptor triangle_baricenter;

        // If the vertex is not already in the graph
        if (triangle_vertex_map.find(index) == triangle_vertex_map.end())
        {
            // Get the point representing the baricenter of the triangle
            Point triangle_center = get_triangle_center(p1, p2, p3);

            // Set the type of the vertex to waypoint
            VertexProperty property(triangle_center, WAYPOINT);

            // Add the vertex to the graph
            triangle_baricenter = boost::add_vertex(property, graph);

            // Store the baricenter of the triangle in the map of the indexes of the triangles
            triangle_vertex_map[index] = triangle_baricenter;
        }
        else
        {
            // Return the already present baricenter of the triangle
            triangle_baricenter = triangle_vertex_map[index];
        }

        // Return the index of the vertex
        return triangle_baricenter;
    }

    /**
     * @brief Find the baricenter of a rectangle and add it to the graph
     *
     * @param points The coordinates of the vertexes of the rectangle
     * @return The index of the added vertex
     */
    GraphType::vertex_descriptor add_rectangle_center(const std::vector<Point> &points)
    {
        Point p1 = points[0];
        Point p2 = points[1];
        Point p3 = points[2];
        Point p4 = points[3];

        GraphType::vertex_descriptor rectangle_baricenter;

        // Get the point representing the baricenter of the rectangle
        Point rectangle_center = get_rectangle_center(p1, p2, p3, p4);

        // Set the type of the vertex to gate
        VertexProperty property(rectangle_center, GATE);

        // Add the vertex to the graph
        rectangle_baricenter = boost::add_vertex(property, graph);

        // Return the index of the vertex
        return rectangle_baricenter;
    }
};
