#pragma once

#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "dpoint.hpp"
#include "dubins_utils.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

class dArc
{
public:
    dPoint *i;
    dPoint *f;
    double k, L;

    dArc(dPoint *_i, double _k, double _L) : i(_i), k(_k), L(_L)
    {
        f = circLine(_L, _i, _k);
    }

    dPoint *circLine(double s, dPoint *p, double k)
    {
        double x = p->x + s * DubinsUtils::sinc(k * s / 2.0) * cos(p->t + k * s / 2);
        double y = p->y + s * DubinsUtils::sinc(k * s / 2.0) * sin(p->t + k * s / 2);
        double theta = DubinsUtils::mod2pi(p->t + k * s);
        return new dPoint(x, y, theta);
    }

    void show_darc(cv::Mat &img, int color_index)
    {
        int n_points = 1000;
        std::vector<cv::Point> contours;

        for (int index = 0; index < n_points; index++)
        {
            double s = L / n_points * index;
            dPoint *d3 = circLine(s, i, k);
            contours.push_back(cv::Point(int(d3->x * 500 + 50), img.size().height - int(d3->y * 500 + 50)));
        }

        cv::Scalar color;

        // switch (color_index)
        // {
        // case 0:
        //     color = cv::Scalar(0, 255, 0);
        //     break;
        // case 1:
        //     color = cv::Scalar(255, 0, 0);
        //     break;
        // case 2:
        //     color = cv::Scalar(0, 0, 255);
        //     break;
        // }

        if (color_index == 1) // Escaper
        {
            color = cv::Scalar(164, 184, 44);
        }
        else // Pursuer
        {
            color = cv::Scalar(17, 166, 238);
        }

        for (int i = 1; i < n_points; i++)
        {
            cv::line(img, contours[i], contours[i - 1], color, 4);
        }
    }

    friend std::ostream &operator<<(std::ostream &os, const dArc &a);

    std::vector<Pose> to_pose_vect()
    {
        std::vector<Pose> pose_vect;
        double delta = L / 10.0;

        pose_vect.push_back(Pose(delta, i->x, i->y, i->t, k));
        
        dPoint *pi = i;
        for (double l = 0.0; l < L; l += delta)
        {
            pi = circLine(delta, pi, k);
            pose_vect.push_back(Pose(delta, pi->x, pi->y, pi->t, k)); // Pose(s, x, y, t, k) s = pathlength, k = curvature
        }

        return pose_vect;
    }
};

std::ostream &operator<<(std::ostream &os, const dArc &a)
{
    return os << setprecision(2) << "[" << *(a.i) << " " << *(a.f) << " k: " << a.k << " L: " << a.L << "]";
}
