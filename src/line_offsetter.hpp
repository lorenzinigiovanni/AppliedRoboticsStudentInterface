#include "utils.hpp"
#include "clipper/clipper.hpp"

#include <stdexcept>
#include <sstream>

class LineOffsetter
{
public:
    static std::vector<Polygon> offset_obstacles(const std::vector<Polygon> &obstacles)
    {
        std::vector<Polygon> result;
        for (const auto &obstacle : obstacles)
        {
            result.push_back(offset_obstacle(obstacle));
        }
        return result;
    }

    static Polygon offset_obstacle(Polygon polygon)
    {
        ClipperLib::Path subj;
        ClipperLib::Paths solution;

        // convert Poligons into path
        for (int i = 0; i < polygon.size(); i++)
        {
            subj.push_back(convert_point_to_int_point(polygon[i]));
        }

        ClipperLib::ClipperOffset co;
        co.AddPath(subj, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
        co.Execute(solution, 100); // Quanto sta da offsettare

        Polygon result;

        for (int i = 0; i < solution[0].size(); i++)
        {
            result.push_back(convert_int_point_to_point(solution[0][i]));
        }

        return result;
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
};
