#include "utils.hpp"
#include "CDT/CDT.h"
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

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
    CDT::Triangulation<float> cdt;

    std::vector<Point> points;
    std::size_t point_count = 0;
    std::vector<Edge> edges;

public:
    void add_borders(const Polygon &borders)
    {
        for (int i = 0; i < borders.size(); i++)
        {
            points.push_back(Point(borders[i].x, borders[i].y));
            point_count++;
            if (i > 0)
            {
                edges.push_back(Edge(point_count - 1, point_count - 2));
            }
        }
        edges.push_back(Edge(point_count - 1, point_count - borders.size()));
    }

    void add_obstacles(const std::vector<Polygon> &obstacles)
    {
        for (int i = 0; i < obstacles.size(); i++)
        {
            for (int j = 0; j < obstacles[i].size(); j++)
            {
                points.push_back(Point(obstacles[i][j].x, obstacles[i][j].y));
                point_count++;
                if (j > 0)
                {
                    edges.push_back(Edge(point_count - 1, point_count - 2));
                }
            }
            edges.push_back(Edge(point_count - 1, point_count - obstacles[i].size()));
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
    }

    void print_triangles()
    {
        for (CDT::TriangleVec::iterator t = cdt.triangles.begin(); t != cdt.triangles.end(); ++t)
        {
            std::cout << "Triangle: " << t->vertices[0] << " " << t->vertices[1] << " " << t->vertices[2] << std::endl;
        }
    }

    void show_triangles()
    {
        cv::Mat img = cv::Mat(1000, 1000, CV_8UC3);

        for (CDT::TriangleVec::iterator t = cdt.triangles.begin(); t != cdt.triangles.end(); ++t)
        {
            for (int i = 0; i < 3; i++)
            {
                Point p1 = points[t->vertices[i]];
                Point p2 = points[t->vertices[(i + 1) % 3]];
                int x1 = int(p1.x * 500) + 10;
                int y1 = int(p1.y * 500) + 10;
                int x2 = int(p2.x * 500) + 10;
                int y2 = int(p2.y * 500) + 10;

                cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1, 8);
            }
        }

        cv::imshow("Nome", img);
        cv::waitKey(0);
    }
};
