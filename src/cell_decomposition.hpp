#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "CDT/CDT.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>

#include <iostream>
#include <stdexcept>
#include <sstream>

/**
 * @brief Edge between two vertexes
 *
 */
class Edge
{
public:
    std::size_t v1; // first vertex
    std::size_t v2; // second vertex

    /**
     * @brief Construct a new Edge object
     *
     * @param _v1 First vertex
     * @param _v2 Second vertex
     */
    Edge(std::size_t _v1, std::size_t _v2)
    {
        v1 = _v1;
        v2 = _v2;
    }
};

/**
 * @brief Object for performing triangular cell decomposition
 *
 */
class CellDecomposition
{
private:
    CDT::Triangulation<float> cdt; // cdt library object
    std::size_t point_count = 0;   //

public:
    std::vector<Point> points;            // vertexes of the polygons
    std::vector<Edge> edges;              // edges between the polygons vertexes
    std::vector<CDT::Triangle> triangles; // triangles found by applying triangular decomposition

    /**
     * @brief Insert a polygon
     *
     * @param polygon A polygon
     */
    void add_polygon(const Polygon &polygon)
    {
        // for each point in the polygon
        for (int i = 0; i < polygon.size(); i++)
        {
            // add the vertex point
            points.push_back(polygon[i]);
            point_count++;

            // add the edges
            if (i > 0)
            {
                edges.push_back(Edge(point_count - 1, point_count - 2));
            }
        }

        // add the last closing edge
        edges.push_back(Edge(point_count - 1, point_count - polygon.size()));
    }

    /**
     * @brief Insert polygons
     *
     * @param polygons A vector of Polygon
     */
    void add_polygons(const std::vector<Polygon> &polygons)
    {
        for (int i = 0; i < polygons.size(); i++)
        {
            add_polygon(polygons[i]);
        }
    }

    /**
     * @brief Create a cdt library object
     *
     */
    void create_cdt()
    {
        // insert vertex in the cdt library object
        cdt.insertVertices(
            points.begin(),
            points.end(),
            [](const Point &p)
            { return p.x; },
            [](const Point &p)
            { return p.y; });

        // insert edges in the cdt library object
        cdt.insertEdges(
            edges.begin(),
            edges.end(),
            [](const Edge &e)
            { return e.v1; },
            [](const Edge &e)
            { return e.v2; });

        // remove the big triangle around all the other
        // cdt.eraseSuperTriangle();

        // remove the big triangle and remove the edges inside the obstacles
        cdt.eraseOuterTrianglesAndHoles();

        triangles = cdt.triangles;
    }

    /**
     * @brief Print the triangles in console
     *
     */
    void print_triangles()
    {
        for (CDT::TriangleVec::iterator t = triangles.begin(); t != triangles.end(); ++t)
        {
            std::cout << "Triangle: " << t->vertices[0] << " " << t->vertices[1] << " " << t->vertices[2] << std::endl;
        }
    }

    /**
     * @brief Print the triangles on the image
     *
     * @param img The image to add the triangles to
     */
    void show_triangles(cv::Mat &img)
    {
        for (CDT::TriangleVec::iterator t = triangles.begin(); t != triangles.end(); ++t)
        {
            for (int i = 0; i < 3; i++)
            {
                Point p1 = points[t->vertices[i]];
                Point p2 = points[t->vertices[(i + 1) % 3]];
                int x1 = int(p1.x * 500);
                int y1 = img.size().height - int(p1.y * 500);
                int x2 = int(p2.x * 500) ;
                int y2 = img.size().height - int(p2.y * 500);

                cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1);
            }
        }
    }
};
