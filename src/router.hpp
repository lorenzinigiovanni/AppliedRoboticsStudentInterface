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

    double k_max = 25;
    double n_angle_steps = 36.0;

public:
    void add_path(std::vector<Point> &path, float theta_start)
    {
        /*
        Compute multi point dubins path using Dynamic Programming
        */

        int path_lenght = path.size();

        double angle = 2 * M_PI;
        double angle_step = angle / n_angle_steps;

        std::vector<std::vector<double>> lenghts_m(path_lenght, std::vector<double>(n_angle_steps, 10000.0)); // matrix path_lenght x n_angle_steps
        std::vector<std::vector<int>> angles_m(path_lenght, std::vector<int>(n_angle_steps, 0));              // matrix path_lenght x n_angle_steps

        for (int n = path_lenght - 2; n >= 0; n--)
        {
            for (int i = 0; i < n_angle_steps; i++)
            {
                for (int j = 0; j < n_angle_steps; j++)
                {
                    dPoint point_1(path[n], angle_step * i);
                    dPoint point_2(path[n + 1], angle_step * j);

                    Dubins::Solution solution = Dubins::solve(point_1, point_2, k_max);
                    double l1 = solution.c.L;

                    double l2 = 0.0;
                    if (n != path_lenght - 2)
                    {
                        l2 = lenghts_m[n + 1][j];
                    }

                    if (lenghts_m[n][i] > l1 + l2)
                    {
                        lenghts_m[n][i] = l1 + l2;
                        angles_m[n + 1][i] = j;
                    }
                }
            }
        }

        int prev_angle = (int)(theta_start / angle_step);

        for (int i = 0; i < path.size(); i++)
        {
            if (i == 0)
            {
                dpoints.push_back(dPoint(path[i], prev_angle * angle_step));
            }
            else
            {
                int curr_angle = angles_m[i][prev_angle];
                dpoints.push_back(dPoint(path[i], curr_angle * angle_step));
                prev_angle = curr_angle;
            }
        }
    }

    void elaborate_solution()
    {
        double total_length = 0.0;
        std::vector<Pose> tmp_path;
        for (int i = 0; i < dpoints.size() - 1; i++)
        {
            Dubins::Solution solution = Dubins::solve(dpoints[i], dpoints[i + 1], k_max);

            if (solution.pidx >= 0)
            {
                solutions.push_back(solution);
                total_length += solution.c.L;
            }
            else
            {
                std::cout << "Failed to find a solution" << std::endl;
            }
        }
        std::cout << "Lenght: " << total_length << std::endl;
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
        for (int i = 0; i < solutions.size(); i++)
        {
            solutions[i].c.show_dcurve(img, index);
        }
    }
};
