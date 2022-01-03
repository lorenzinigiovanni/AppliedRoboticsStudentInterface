#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"

#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include <vector>
#include <cmath>

class Router
{
private:
    std::vector<Dubins::Solution> solutions;
    std::vector<DubinsPoint> dpoints;

    double k_max = 22;
    int n_angle_steps = 32;
    int iterations = 2;

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

        std::vector<std::vector<int>> angles_m(path_lenght, std::vector<int>(n_angle_steps, 0)); // matrix path_lenght x n_angle_steps
        std::vector<std::vector<int>> angles_i(iterations, std::vector<int>(path_lenght, 0));    // matrix iterations x path_lenght

        for (int iteration = 0; iteration < iterations; iteration++)
        {
            for (int n = path_lenght - 2; n >= 0; n--)
            {
                for (int i = 0; i < n_angle_steps; i++)
                {
                    for (int j = 0; j < n_angle_steps; j++)
                    {
                        double angle_1 = 0.0;
                        double angle_2 = 0.0;

                        for (int z = 0; z < iteration; z++)
                        {
                            angle_1 += angle_step * angles_i[z][n] * std::pow(3.0 / n_angle_steps, z);
                            angle_2 += angle_step * angles_i[z][n + 1] * std::pow(3.0 / n_angle_steps, z);

                            if (z > 0)
                            {
                                angle_1 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, z - 1);
                                angle_2 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, z - 1);
                            }
                        }

                        angle_1 += angle_step * i * std::pow(3.0 / n_angle_steps, iteration);
                        angle_2 += angle_step * j * std::pow(3.0 / n_angle_steps, iteration);

                        if (iteration > 0)
                        {
                            angle_1 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                            angle_2 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                        }

                        DubinsPoint point_1(path[n], angle_1);
                        DubinsPoint point_2(path[n + 1], angle_2);

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

            std::vector<int> prev_angle(iterations);

            prev_angle[0] = round(theta_start / angle_step);

            for (int i = 1; i < iteration; i++)
            {
                int angle = (int)round(theta_start / angle_step);
                for (int j = 0; j < i; j++)
                {
                    angle += -prev_angle[j] * pow(3 / angle_step, j) + (3.0 / 2.0) * pow(3 / angle_step, j);
                }

                if (angle >= n_angle_steps)
                {
                    angle = 0;
                }
                else if (angle < 0)
                {
                    angle = 0;
                }

                prev_angle[i] = angle;
            }

            for (int i = 0; i < path_lenght; i++)
            {
                int curr_angle = angles_m[i][prev_angle[iteration]];
                angles_i[iteration][i] = curr_angle;
                prev_angle[iteration] = curr_angle;
            }
        }

        for (int i = 0; i < path.size(); i++)
        {
            if (i == 0)
            {
                dpoints.push_back(DubinsPoint(path[i], theta_start));
            }
            else
            {
                double angle = 0.0;

                for (int iteration = 0; iteration < iterations; iteration++)
                {
                    angle += angle_step * angles_i[iteration][i] * std::pow(3.0 / n_angle_steps, iteration);

                    if (iteration > 0)
                    {
                        angle -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                    }
                }

                dpoints.push_back(DubinsPoint(path[i], angle));
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

    std::vector<Pose> get_path(int steps=-1)
    {
        std::vector<Pose> path;

        if (solutions.size() < steps)
        {
            steps = solutions.size();
        }

        for (int i = 0; i < steps; i++)
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
