#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <map>
#include <vector>
#include <algorithm>

class LineOffsetter
{
public:
    static std::vector<Polygon> offset_polygons(const std::vector<Polygon> &obstacles, int offset)
    {
        std::vector<ClipperLib::Path> paths = convert_polygons_to_paths(obstacles);

        std::vector<ClipperLib::Path> offsetted_paths;
        for (const auto &path : paths)
        {
            offsetted_paths.push_back(offset_path(path, offset));
        }

        return convert_paths_to_polygons(offsetted_paths);
    }

    static std::vector<Polygon> merge_polygons(const std::vector<Polygon> &polygon)
    {
        std::vector<ClipperLib::Path> paths = convert_polygons_to_paths(polygon);

        ClipperLib::Clipper c;
        ClipperLib::Paths solution;

        c.AddPaths(paths, ClipperLib::ptSubject, true);
        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        c.Clear();

        return convert_paths_to_polygons(solution);
    }

    static std::vector<Polygon> intersect_polygons(const std::vector<Polygon> &polygon1, const std::vector<Polygon> &polygon2)
    {
        std::vector<ClipperLib::Path> paths1 = convert_polygons_to_paths(polygon1);
        std::vector<ClipperLib::Path> paths2 = convert_polygons_to_paths(polygon2);

        ClipperLib::Clipper c;
        ClipperLib::Paths solution;

        c.AddPaths(paths1, ClipperLib::ptSubject, true);
        c.AddPaths(paths2, ClipperLib::ptClip, true);
        c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
        c.Clear();

        return convert_paths_to_polygons(solution);
    }

private:
    static ClipperLib::Path offset_path(ClipperLib::Path path, int offset)
    {
        ClipperLib::Paths solution;

        ClipperLib::ClipperOffset co;
        co.AddPath(path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);

        co.Execute(solution, offset);

        return solution[0];
    }

    static ClipperLib::IntPoint convert_point_to_int_point(Point _p)
    {
        ClipperLib::IntPoint p;
        p.X = _p.x * 1000;
        p.Y = _p.y * 1000;
        return p;
    }

    static Point convert_int_point_to_point(ClipperLib::IntPoint _ip)
    {
        Point p;
        p.x = float(_ip.X) / 1000;
        p.y = float(_ip.Y) / 1000;
        return p;
    }

    static std::vector<ClipperLib::Path> convert_polygons_to_paths(const std::vector<Polygon> &polygons)
    {
        std::vector<ClipperLib::Path> paths;
        for (const auto &polygon : polygons)
        {
            ClipperLib::Path path;
            for (const auto &point : polygon)
            {
                path.push_back(convert_point_to_int_point(point));
            }
            paths.push_back(path);
        }
        return paths;
    }

    static std::vector<Polygon> convert_paths_to_polygons(const std::vector<ClipperLib::Path> &paths)
    {
        std::vector<Polygon> polygons;
        for (const auto &path : paths)
        {
            Polygon polygon;
            for (const auto &point : path)
            {
                polygon.push_back(convert_int_point_to_point(point));
            }
            polygons.push_back(polygon);
        }
        return polygons;
    }
};
