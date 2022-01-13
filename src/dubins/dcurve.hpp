#pragma once

#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "darc.hpp"
#include "dpoint.hpp"

#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Dubins Curve
 *
 */
class DubinsCurve
{
public:
    DubinsArc a1, a2, a3; // three arc of a curve
    double L;             // lenght of the curve

    /**
     * @brief Construct a new Dubins Curve object
     *
     * @param d1 Starting point
     * @param s1 Lenght of the first arc
     * @param s2 Lenght of the second arc
     * @param s3 Lenght of the third arc
     * @param k1 Curvature of the first arc
     * @param k2 Curvature of the second arc
     * @param k3 Curvature of the third arc
     */
    DubinsCurve(DubinsPoint d1, double s1, double s2, double s3, double k1, double k2, double k3)
    {
        // a Dubins Curve is composed of three Dubins Arc
        a1 = DubinsArc(d1, k1, s1);
        a2 = DubinsArc(a1.f, k2, s2);
        a3 = DubinsArc(a2.f, k3, s3);

        // the lenght of the curve is the sum of the lenght of the three arcs
        L = a1.L + a2.L + a3.L;
    }

    /**
     * @brief Print the Dubins Curve on the image
     *
     * @param img The image to add the Dubins Arc to
     * @param color_index The index of the robot to color the curve
     */
    void show_dcurve(cv::Mat &img, int color_index)
    {
        // color start green, middle blue, end red
        // a1->show_darc(img, 0);
        // a2->show_darc(img, 1);
        // a3->show_darc(img, 2);

        // color depends on robot
        a1.show_darc(img, color_index);
        a2.show_darc(img, color_index);
        a3.show_darc(img, color_index);
    }

    /**
     * @brief Convert the Dubins Curve to a vector of Pose
     *
     * @return A vector of Pose
     */
    std::vector<Pose> to_pose_vect(double initial_l = 0.0)
    {
        std::vector<Pose> path; // path composed of the three arcs

        // convert arc1 to pose vector and add to path
        std::vector<Pose> arc1 = a1.to_pose_vect(initial_l);
        path.insert(path.end(), std::make_move_iterator(arc1.begin()), std::make_move_iterator(arc1.end()));

        initial_l += a1.L;

        // convert arc2 to pose vector and add to path
        std::vector<Pose> arc2 = a2.to_pose_vect(initial_l);
        path.insert(path.end(), std::make_move_iterator(arc2.begin()), std::make_move_iterator(arc2.end()));

        initial_l += a2.L;

        // convert arc3 to pose vector and add to path
        std::vector<Pose> arc3 = a3.to_pose_vect(initial_l);
        path.insert(path.end(), std::make_move_iterator(arc3.begin()), std::make_move_iterator(arc3.end()));

        return path;
    }
};
