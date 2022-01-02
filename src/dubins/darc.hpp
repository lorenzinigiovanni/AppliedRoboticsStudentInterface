#pragma once

#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "dpoint.hpp"
#include "dubins_utils.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Arc of a Dubins curve
 *
 */
class DubinsArc
{
public:
    DubinsPoint *i; // starting point
    DubinsPoint *f; // ending point
    double k;  // curvature
    double L;  // lenght

    /**
     * @brief Construct a new Dubins Arc object
     *
     * @param _i The starting point
     * @param _k The curvature
     * @param _L The lenght
     */
    DubinsArc(DubinsPoint *_i, double _k, double _L) : i(_i), k(_k), L(_L)
    {
        // compute ending point given starting point, curvature and lenght
        f = circLine(_L, _i, _k);
    }

    /**
     * @brief Compute the point on the circle given the starting point, curvature and lenght
     *
     * @param s The lenght of the arc
     * @param p The starting point
     * @param k The curvature
     * @return The ending point of the arc
     */
    DubinsPoint *circLine(double s, DubinsPoint *p, double k)
    {
        double x = p->x + s * DubinsUtils::sinc(k * s / 2.0) * cos(p->t + k * s / 2);
        double y = p->y + s * DubinsUtils::sinc(k * s / 2.0) * sin(p->t + k * s / 2);
        double theta = DubinsUtils::mod2pi(p->t + k * s);
        return new DubinsPoint(x, y, theta);
    }

    /**
     * @brief Print the Dubins Arc on the image
     *
     * @param img The image to add the Dubins Arc to
     * @param color_index The index of the robot or the index of the Dubins Arc in the Dubins Curve
     */
    void show_darc(cv::Mat &img, int color_index)
    {
        // samples the arc with n_points
        int n_points = 1000;
        std::vector<cv::Point> contours;

        for (int index = 0; index < n_points; index++)
        {
            double s = L / n_points * index;
            DubinsPoint *d = circLine(s, i, k);
            contours.push_back(cv::Point(int(d->x * 500 + 50), img.size().height - int(d->y * 500 + 50)));
        }

        cv::Scalar color;

        // switch (color_index)
        // {
        // case 0: // start green
        //     color = cv::Scalar(0, 255, 0);
        //     break;
        // case 1: // middle blue
        //     color = cv::Scalar(255, 0, 0);
        //     break;
        // case 2: // end red
        //     color = cv::Scalar(0, 0, 255);
        //     break;
        // }

        switch (color_index)
        {
        case 0: // pursuer
            color = cv::Scalar(255, 83, 27);
            break;
        case 1: // escaper
            color = cv::Scalar(0, 200, 255);
            break;
        }

        // add every point to the image connecting it to the neighbour points
        for (int i = 1; i < n_points; i++)
        {
            cv::line(img, contours[i], contours[i - 1], color, 2);
        }
    }

    /**
     * @brief Convert the Dubins Arc to a vector of Pose
     *
     * @return A vector of Pose
     */
    std::vector<Pose> to_pose_vect()
    {
        std::vector<Pose> pose_vect;

        // samples the arc with n points
        double delta = L / 100.0;

        pose_vect.push_back(Pose(delta, i->x, i->y, i->t, k));

        DubinsPoint *pi = i;
        for (double l = 0.0; l < L; l += delta)
        {
            pi = circLine(delta, pi, k);
            pose_vect.push_back(Pose(delta, pi->x, pi->y, pi->t, k)); // Pose(s, x, y, t, k) s = pathlength, k = curvature
        }

        return pose_vect;
    }

    friend std::ostream &operator<<(std::ostream &os, const DubinsArc &a);
};

std::ostream &operator<<(std::ostream &os, const DubinsArc &a)
{
    return os << setprecision(2) << "[" << *(a.i) << " " << *(a.f) << " k: " << a.k << " L: " << a.L << "]";
}
