#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"

#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include <vector>

class Router
{
private:
    std::vector<Dubins::Solution> solutions;
    std::vector<dPoint> dpoints;

public:
    // Point to dPoint
    void add_path(std::vector<Point> &path, float theta_start)
    {
        for (int i = 0; i < path.size(); i++)
        {
            float theta0 = 0.0;
            float theta1 = 0.0;

            if (i == 0)
            {
                dpoints.push_back(dPoint(path[i], theta_start)); // suppose theta escaper = theta[1]
            }
            else if (i == path.size() - 1)
            {
                float dx = path[i].x - path[i - 1].x;
                float dy = path[i].y - path[i - 1].y;
                theta0 = std::atan2(dy, dx);
                dpoints.push_back(dPoint(path[i], theta0));
            }
            else
            {
                float dx1 = path[i].x - path[i - 1].x;
                float dy1 = path[i].y - path[i - 1].y;
                theta0 = std::atan2(dy1, dx1);

                float dx2 = path[i + 1].x - path[i].x;
                float dy2 = path[i + 1].y - path[i].y;
                theta1 = std::atan2(dy2, dx2);

                float theta_tot = (theta0 + theta1) / 2;

                dpoints.push_back(dPoint(path[i], theta_tot));
            }
        }
    }

    void elaborate_solution()
    {
        std::vector<Pose> tmp_path;
        for (int i = 0; i < dpoints.size() - 1; i++)
        {
            Dubins::Solution solution = Dubins::solve(dpoints[i], dpoints[i + 1], 25); // dPoint, dPoint, max curvature

            if (solution.pidx >= 0)
            {
                solutions.push_back(solution);
            }
            else
            {
                cout << "Failed to find a solution\n";
            }
        }
    }

    std::vector<Pose> get_path()
    {
        std::vector<Pose> path;

        for (int i = 0; i < solutions.size(); i++)
        {
            std::vector<Pose> paths_from_dCurve = solutions[i].c.to_pose_vect();
            path.insert(path.end(), std::make_move_iterator(paths_from_dCurve.begin()), std::make_move_iterator(paths_from_dCurve.end()));
        }

        return path;
    }

    void show_path(cv::Mat &img, unsigned int index)
    {
        for (int i = 0; i < solutions.size() - 1; i++)
        {
            solutions[i].c.show_dcurve(img, index);
        }
    }
};
