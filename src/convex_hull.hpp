#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "boost/geometry.hpp"
#include "boost/geometry/geometries/polygon.hpp"
#include "boost/geometry/geometries/adapted/boost_tuple.hpp"
#include <boost/geometry/algorithms/convex_hull.hpp>

BOOST_GEOMETRY_REGISTER_BOOST_TUPLE_CS(cs::cartesian)

/**
 * @brief Create convex hull for ensemble of Polygons
 *
 */
class ConvexHull
{
    typedef boost::tuple<double, double> point;                   // Boost point type
    typedef boost::geometry::model::polygon<point> boost_polygon; // Boost polygon type

public:
    /**
     * @brief Apply the convex hull algorithm to the ensemble of polygons
     *
     * @param polygons List of polygons
     * @return A list of polygons
     */
    static std::vector<Polygon> create_convex_hull(const std::vector<Polygon> polygons)
    {
        std::vector<Polygon> convex_hulls;

        // For every polygon
        for (int i = 0; i < polygons.size(); i++)
        {
            // Convert the polygon to a string used by boost
            std::string wkt_string = convert_polygon_to_boost_polygon(polygons[i]);

            boost_polygon poly;
            boost_polygon hull;

            // Convert the string to a boost polygon
            boost::geometry::read_wkt(wkt_string, poly);

            // Apply the convex hull algorithm
            boost::geometry::convex_hull(poly, hull);

            // Convert back the boost polygon to a polygon
            Polygon polygon = convert_boost_polygon_to_polygon(hull);

            convex_hulls.push_back(polygon);
        }

        return convex_hulls;
    }

    /**
     * @brief Convert polygon to boost polygon
     *
     * @param polygon Single standard polygon
     * @return A string representing a polygon used by boost
     */
    static std::string convert_polygon_to_boost_polygon(const Polygon &polygon)
    {
        std::string wkt_string = "polygon((";

        // For each polygon's vertex
        for (int i = 0; i < polygon.size(); i++)
        {
            // Add the vertex segment pair to the string
            wkt_string += std::to_string(polygon[i].x) + " " + std::to_string(polygon[i].y) + ",";
        }

        // Add the closing vertex segment
        wkt_string += std::to_string(polygon[0].x) + " " + std::to_string(polygon[0].y);

        wkt_string += "))";

        return wkt_string;
    }

    /**
     * @brief Convert boost polygon to polygon
     *
     * @param boost_polygon Single boost polygon
     * @return Standard polygon
     */
    static Polygon convert_boost_polygon_to_polygon(const boost_polygon boost_polygon)
    {
        Polygon polygon;

        // For each boost polygon's vertex
        for (int i = 0; i < boost_polygon.outer().size() - 1; i++)
        {
            Point point;

            // Get boost polygon's vertex coordinates
            point.x = boost_polygon.outer()[i].get<0>();
            point.y = boost_polygon.outer()[i].get<1>();

            // Save standard polygon
            polygon.push_back(point);
        }

        return polygon;
    }
};
