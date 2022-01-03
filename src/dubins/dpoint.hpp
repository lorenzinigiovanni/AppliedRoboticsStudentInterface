#pragma once

#include <iostream>

/**
 * @brief Point in the 2D space with robot orientation for the Dubins path
 *
 */
class DubinsPoint
{
public:
    double x; // x coordinate
    double y; // y coordinate
    double t; // angle of the robot

    /**
     * @brief Construct a new Dubins Point object
     *
     * @param _x The x coordinate
     * @param _y The y coordinate
     * @param _t The angle of the robot
     */
    DubinsPoint(double _x, double _y, double _t) : x(_x), y(_y), t(_t){};

    /**
     * @brief Construct a new Dubins Point object
     *
     * @param p A point in the 2D space
     * @param _t The angle of the robot
     */
    DubinsPoint(const Point p, double _t) : x(p.x), y(p.y), t(_t){};

    friend std::ostream &operator<<(std::ostream &os, const DubinsPoint &_p);
};

/**
 * @brief Print Dubins Point in console
 * 
 * @param os 
 * @param a 
 * @return std::ostream& 
 */
std::ostream &operator<<(std::ostream &os, const DubinsPoint &_p)
{
    return os << "(" << _p.x << " " << _p.y << " " << _p.t << ")";
}
