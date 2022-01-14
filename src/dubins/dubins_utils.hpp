#pragma once

#include <cmath>
#include "../../../simulator/src/9_project_interface/include/utils.hpp"

using namespace std;

/**
 * @brief Utility functions used in Dubins
 *
 */
class DubinsUtils
{
public:
    /**
     * @brief Compute the sinc function
     *
     * @param t The argument of the sinc function
     * @return The sinc of t
     */
    double static sinc(double t)
    {
        double s;
        if (abs(t) < 0.002)
        {
            s = 1 - (pow(t, 2) / 6) + (pow(t, 4) / 120);
        }
        else
        {
            s = sin(t) / t;
        }
        return s;
    }

    /**
     * @brief Keep the angle between 0 and 2 pi
     *
     * @param ang The angle to keep between 0 and 2 pi
     * @return The angle between 0 and 2 pi
     */
    double static mod2pi(double ang)
    {
        double out = ang;
        while (out < 0)
        {
            out = out + 2 * M_PI;
        }
        while (out >= 2 * M_PI)
        {
            out = out - 2 * M_PI;
        }
        return out;
    }

    /**
     * @brief Keep the angle between -pi and pi
     *
     * @param ang The angle to keep between -pi and pi
     * @return The angle between -pi and pi
     */
    double static rangeSymm(double ang)
    {
        double out = ang;
        while (out <= -1 * M_PI)
        {
            out = out + 2 * M_PI;
        }
        while (out > M_PI)
        {
            out = out - 2 * M_PI;
        }

        return out;
    }

    /**
     * @brief Check the validity of a Dubins curve
     *
     * @param s1 Lenght of the first arc
     * @param k0 Curvature of the first arc
     * @param s2 Lenght of the second arc
     * @param k1 Curvature of the second arc
     * @param s3 Lenght of the third arc
     * @param k2 Curvature of the third arc
     * @param th0 Starting angle
     * @param thf Final angle
     * @return true if the curve is valid
     * @return false if the curve is not valid
     */
    bool static check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf)
    {
        double x0 = -1;
        double y0 = 0;
        double xf = 1;
        double yf = 0;

        double eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
        double eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
        double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

        bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
    }
    /**
     * @brief Compute cross product between two points
     *
     * @param p1
     * @param p2
     * @return double
     */
    double static cprod(Point p1, Point p2)
    {
        return p1.x * p2.y - p1.y * p2.x;
    }
    
    /**
     * @brief Compute dot product between two points
     *
     * @param p1
     * @param p2
     * @return double
     */
    double static dprod(Point p1, Point p2)
    {
        return p1.x * p2.x + p1.y * p2.y;
    }
};
