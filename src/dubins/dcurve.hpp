#pragma once

#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "darc.hpp"
#include "dpoint.hpp"

#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

class dCurve
{
public:
    dArc *a1, *a2, *a3;
    float L;

    dCurve(dPoint *d1, float s1, float s2, float s3, float k1, float k2, float k3)
    {
        a1 = new dArc(d1, k1, s1);
        a2 = new dArc(a1->f, k2, s2);
        a3 = new dArc(a2->f, k3, s3);
        L = a1->L + a2->L + a3->L;
    }

    void show_dcurve(cv::Mat &img, int _index)
    {
        // a1->show_darc(img, 0);
        // a2->show_darc(img, 1);
        // a3->show_darc(img, 2);

        a1->show_darc(img, _index);
        a2->show_darc(img, _index);
        a3->show_darc(img, _index);
    }

    std::vector<Pose> to_pose_vect()
    {
        std::vector<Pose> path;
        std::vector<Pose> arc;

        arc = a1->to_pose_vect();
        path.insert(path.end(), std::make_move_iterator(arc.begin()), std::make_move_iterator(arc.end()));

        arc = a2->to_pose_vect();
        path.insert(path.end(), std::make_move_iterator(arc.begin()), std::make_move_iterator(arc.end()));

        arc = a3->to_pose_vect();
        path.insert(path.end(), std::make_move_iterator(arc.begin()), std::make_move_iterator(arc.end()));

        return path;
    }
};
