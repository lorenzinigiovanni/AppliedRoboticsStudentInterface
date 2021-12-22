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

class Edge
{
public:
    std::size_t v1;
    std::size_t v2;

    Edge(std::size_t _v1, std::size_t _v2)
    {
        v1 = _v1;
        v2 = _v2;
    }
};

class CellDecomposition
{
private:
    CDT::Triangulation<float> cdt;
    std::size_t point_count = 0;

public:
    std::vector<Point> points;
    std::vector<Edge> edges;
    std::vector<CDT::Triangle> triangles;

    void add_polygon(const Polygon &polygon)
    {
        for (int i = 0; i < polygon.size(); i++)
        {
            points.push_back(polygon[i]);
            point_count++;
            if (i > 0)
            {
                edges.push_back(Edge(point_count - 1, point_count - 2));
            }
        }
        edges.push_back(Edge(point_count - 1, point_count - polygon.size()));
    }

    void add_polygons(const std::vector<Polygon> &polygons)
    {
        for (int i = 0; i < polygons.size(); i++)
        {
            for (int j = 0; j < polygons[i].size(); j++)
            {
                points.push_back(polygons[i][j]);
                point_count++;
                if (j > 0)
                {
                    edges.push_back(Edge(point_count - 1, point_count - 2));
                }
            }
            edges.push_back(Edge(point_count - 1, point_count - polygons[i].size()));
        }
    }

    void create_cdt()
    {
        cdt.insertVertices(
            points.begin(),
            points.end(),
            [](const Point &p)
            { return p.x; },
            [](const Point &p)
            { return p.y; });

        cdt.insertEdges(
            edges.begin(),
            edges.end(),
            [](const Edge &e)
            { return e.v1; },
            [](const Edge &e)
            { return e.v2; });

        // cdt.eraseSuperTriangle();
        cdt.eraseOuterTrianglesAndHoles();

        triangles = cdt.triangles;
    }

    void print_triangles()
    {
        for (CDT::TriangleVec::iterator t = triangles.begin(); t != triangles.end(); ++t)
        {
            std::cout << "Triangle: " << t->vertices[0] << " " << t->vertices[1] << " " << t->vertices[2] << std::endl;
        }
    }

    void show_triangles()
    {
        unsigned int size_x = 1000;
        unsigned int size_y = 800;

        cv::Mat img = cv::Mat(size_y, size_x, CV_8UC3);

        for (CDT::TriangleVec::iterator t = cdt.triangles.begin(); t != cdt.triangles.end(); ++t)
        {
            for (int i = 0; i < 3; i++)
            {
                Point p1 = points[t->vertices[i]];
                Point p2 = points[t->vertices[(i + 1) % 3]];
                int x1 = int(p1.x * 500) + 50;
                int y1 = size_y - int(p1.y * 500) - 50;
                int x2 = int(p2.x * 500) + 50;
                int y2 = size_y - int(p2.y * 500) - 50;

                cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1, 8);
            }
        }
        
        cv::imshow("Cell decomposition", img);
        cv::waitKey(0);
    }
};
