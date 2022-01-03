#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"

#include <iostream>
#include <stdexcept>
#include <sstream>
#include <map>
#include <vector>
#include <algorithm>

/**
 * @brief Add clearance for obstacles or borders
 *
 */
class LineOffsetter
{
public:
    /**
     * @brief Offset a list of polygons
     *
     * @param polygons List of polygons to be offsetted
     * @param offset Offset value in integer multiplied by 1000
     * @return A vector of offsetted polygons
     */
    static std::vector<Polygon> offset_polygons(const std::vector<Polygon> &polygons, int offset)
    {
        // convert polygons to paths
        std::vector<ClipperLib::Path> paths = convert_polygons_to_paths(polygons);

        // offset each path
        std::vector<ClipperLib::Path> offsetted_paths;
        for (const auto &path : paths)
        {
            offsetted_paths.push_back(offset_path(path, offset));
        }

        // convert offsetted paths to polygons and return them
        return convert_paths_to_polygons(offsetted_paths);
    }

    /**
     * @brief Merge overlapping polygons in one
     *
     * @param polygon List of polygons to be merged, may as well not be overlapping
     * @return A vector of merged polygons, one polygon for each overlapping block
     */
    static std::vector<Polygon> merge_polygons(const std::vector<Polygon> &polygon)
    {
        // convert polygons to paths
        std::vector<ClipperLib::Path> paths = convert_polygons_to_paths(polygon);

        ClipperLib::Clipper c;
        ClipperLib::Paths solution;

        // add the paths to the clipper object
        c.AddPaths(paths, ClipperLib::ptSubject, true);

        // find the union of the paths, using the non zero filling rule
        c.Execute(ClipperLib::ctUnion, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        // convert merged paths to polygons and return them
        return convert_paths_to_polygons(solution);
    }

    /**
     * @brief Intersect polygons to obtain the area of intersection
     *
     * @param polygon1 List of polygons to be intersected
     * @param polygon2 List of polygons to be intersected
     * @return A vector of intersected polygons
     */
    static std::vector<Polygon> intersect_polygons(const std::vector<Polygon> &polygon1, const std::vector<Polygon> &polygon2)
    {
        // convert polygons to paths
        std::vector<ClipperLib::Path> paths1 = convert_polygons_to_paths(polygon1);
        std::vector<ClipperLib::Path> paths2 = convert_polygons_to_paths(polygon2);

        ClipperLib::Clipper c;
        ClipperLib::Paths solution;

        // add the paths to the clipper object
        c.AddPaths(paths1, ClipperLib::ptSubject, true);
        c.AddPaths(paths2, ClipperLib::ptClip, true);

        // find the intersection of the paths, using the non zero filling rule
        c.Execute(ClipperLib::ctIntersection, solution, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

        // convert intersected paths to polygons and return them
        return convert_paths_to_polygons(solution);
    }

private:
    /**
     * @brief Offset a path
     *
     * @param path Object on which to apply offset
     * @param offset Specify offset amount to be multiplied by 1000
     * @return An offsetted path
     */
    static ClipperLib::Path offset_path(ClipperLib::Path path, int offset)
    {
        ClipperLib::Paths solution;
        ClipperLib::ClipperOffset co;

        // jtMiter specifies a maximum distance that vertices will be offset
        co.AddPath(path, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);

        co.Execute(solution, offset);

        return solution[0];
    }

    /**
     * @brief Convert a point to an integer point to be used by clipper
     *
     * @param _p A float point
     * @return An integer point
     */
    static ClipperLib::IntPoint convert_point_to_int_point(Point _p)
    {
        ClipperLib::IntPoint ip;
        ip.X = _p.x * 1000;
        ip.Y = _p.y * 1000;
        return ip;
    }

    /**
     * @brief Convert and integer point to a float point
     *
     * @param _ip And integer point
     * @return A float point
     */
    static Point convert_int_point_to_point(ClipperLib::IntPoint _ip)
    {
        Point p;
        p.x = float(_ip.X) / 1000;
        p.y = float(_ip.Y) / 1000;
        return p;
    }

    /**
     * @brief Convert polygons to paths
     *
     * @param polygons Vector of polygons
     * @return A vector of paths
     */
    static std::vector<ClipperLib::Path> convert_polygons_to_paths(const std::vector<Polygon> &polygons)
    {
        // for each polygon in polygons
        std::vector<ClipperLib::Path> paths;
        for (const auto &polygon : polygons)
        {
            // for each point in polygon
            ClipperLib::Path path;
            for (const auto &point : polygon)
            {
                // convert the point to integer point and add it to path
                path.push_back(convert_point_to_int_point(point));
            }
            // add the path to paths
            paths.push_back(path);
        }
        return paths;
    }

    /**
     * @brief Convert paths to polygons
     *
     * @param paths Vector of paths
     * @return A vector of polygons
     */
    static std::vector<Polygon> convert_paths_to_polygons(const std::vector<ClipperLib::Path> &paths)
    {
        // for each path in paths
        std::vector<Polygon> polygons;
        for (const auto &path : paths)
        {
            // for each point in the path
            Polygon polygon;
            for (const auto &point : path)
            {
                // convert the integer point to point and add it to polygon
                polygon.push_back(convert_int_point_to_point(point));
            }
            // add the polygon to the vector of polygons
            polygons.push_back(polygon);
        }
        return polygons;
    }
};
