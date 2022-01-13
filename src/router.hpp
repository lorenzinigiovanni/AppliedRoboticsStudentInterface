#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"

#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include "settings.hpp"
#include <vector>
#include <cmath>

/**
 * @brief Compute a Dubins multi-point path using dynamic programming starting from a vector of points
 *
 */
class Router
{
private:
    std::vector<Dubins::Solution> solutions; // solutions of the Dubins paths
    std::vector<DubinsPoint> dubins_points;  // Dubins points

    double k_max = Settings::k_max; // maximum curvature, critical maximum is 22
    int n_angle_steps = 32;         // number of angles for the discretization of the angle
    int iterations = 2;             // iterations of the Dubins algorithm

public:
    /**
     * @brief Add the path to the router
     *
     * @param path Path to add
     * @param theta_start Initial angle of the robot
     */
    void add_path(std::vector<Point> &_path, float theta_start)
    {
        std::vector<Point> path = optimize_path(_path);
        int path_lenght = path.size();

        double angle = 2 * M_PI;
        double angle_step = angle / n_angle_steps;

        // store the length of the path from a point to the end for every point and angle
        std::vector<std::vector<double>> lenghts_m(path_lenght, std::vector<double>(n_angle_steps, 10000.0));

        // store the angle for every point and angle of the previous point
        std::vector<std::vector<int>> angles_m(path_lenght, std::vector<int>(n_angle_steps, 0));

        // store the best angle for each iteration and point
        std::vector<std::vector<int>> angles_i(iterations, std::vector<int>(path_lenght, 0));

        // iteration over the number of iterations
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            // iteration over the number of points
            for (int n = path_lenght - 2; n >= 0; n--)
            {
                // iteration for the angle of the FROM point over the number of angles
                for (int angle_1_index = 0; angle_1_index < n_angle_steps; angle_1_index++)
                {
                    // iteration for the angle of the TO point over the number of angles
                    for (int angle_2_index = 0; angle_2_index < n_angle_steps; angle_2_index++)
                    {
                        double angle_1 = 0.0;
                        double angle_2 = 0.0;

                        // compute the angle in radiants using the discrete angles of the previous iteration
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

                        // add the current iteration angle properly scaled
                        angle_1 += angle_step * angle_1_index * std::pow(3.0 / n_angle_steps, iteration);
                        angle_2 += angle_step * angle_2_index * std::pow(3.0 / n_angle_steps, iteration);

                        if (iteration > 0)
                        {
                            angle_1 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                            angle_2 -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                        }

                        // instantiate the dubins points with the FROM and TO points and the angles
                        DubinsPoint point_1(path[n], angle_1);
                        DubinsPoint point_2(path[n + 1], angle_2);

                        // compute the length of the path from the FROM point to the TO point
                        Dubins::Solution solution = Dubins::solve(point_1, point_2, k_max);
                        double l1 = solution.c.L;

                        // lenght of the path from the TO point to the END
                        double l2 = 0.0;
                        if (n != path_lenght - 2)
                        {
                            l2 = lenghts_m[n + 1][angle_2_index];
                        }

                        // the length of the path from the FROM point to the END is shorter than the previous one
                        if (lenghts_m[n][angle_1_index] > l1 + l2)
                        {
                            // the leght of the path from the FROM point
                            lenghts_m[n][angle_1_index] = l1 + l2;

                            // the angle of the TO point wrt the angle of the FROM point
                            angles_m[n + 1][angle_1_index] = angle_2_index;
                        }
                    }
                }
            }

            // store the index of the angle in the previous point for each iteration
            std::vector<int> previous_angle_index(iterations);

            // find the indexes needed to compute back the angle
            for (int i = 0; i < iteration; i++)
            {
                int angle_index = (int)round(theta_start / angle_step);
                for (int j = 0; j < i; j++)
                {
                    angle_index += -previous_angle_index[j] * pow(3 / angle_step, j) + (3.0 / 2.0) * pow(3 / angle_step, j);
                }

                // keep the angle index between 0 and the number of angles - 1
                if (angle_index >= n_angle_steps)
                {
                    angle_index = 0;
                }
                else if (angle_index < 0)
                {
                    angle_index = 0;
                }

                previous_angle_index[i] = angle_index;
            }

            // compute the best angle for the current iteration for each point
            for (int i = 0; i < path_lenght; i++)
            {
                // the current angle index is given looking at angles_m in the current point given the previous angles
                int curr_angle_index = angles_m[i][previous_angle_index[iteration]];
                angles_i[iteration][i] = curr_angle_index;
                previous_angle_index[iteration] = curr_angle_index;
            }
        }

        // add each Dubins point to a Dubins Path
        for (int i = 0; i < path.size(); i++)
        {
            if (i == 0)
            {
                // the first Dubins Point has the robot angle
                dubins_points.push_back(DubinsPoint(path[i], theta_start));
            }
            else
            {
                // for the other points compute the angle summing the properly scaled angles by they index
                double angle = 0.0;

                for (int iteration = 0; iteration < iterations; iteration++)
                {
                    angle += angle_step * angles_i[iteration][i] * std::pow(3.0 / n_angle_steps, iteration);

                    if (iteration > 0)
                    {
                        angle -= angle_step * (3.0 / 2.0) * std::pow(3.0 / n_angle_steps, iteration - 1);
                    }
                }

                dubins_points.push_back(DubinsPoint(path[i], angle));
            }
        }
    }

    /**
     * @brief Compute the optimal Dubins Path for the given path points
     *
     */
    void elaborate_solution()
    {
        double total_length = 0.0;

        // for every section of the path compute a Dubins Solution and add it to the Dubins Path
        for (int i = 0; i < dubins_points.size() - 1; i++)
        {
            Dubins::Solution solution = Dubins::solve(dubins_points[i], dubins_points[i + 1], k_max);

            if (solution.pidx >= 0)
            {
                solutions.push_back(solution);
                total_length += solution.c.L;
            }
            else
            {
                std::cout << "Failed to find a solution" << std::endl;
                break;
            }
        }

        std::cout << "Lenght: " << total_length << std::endl;
    }

    /**
     * @brief Get the computed Dubins Path as a vector of Pose
     *
     * @param lenght Max lenght to be moved
     * @return A vector of Pose that represent the computed Dubins Path
     */
    std::vector<Pose> get_path(double lenght)
    {
        std::vector<Pose> path;

        // for every Dubins Solution compute the current path and add it to the path
        double total_lenght = 0.0;
        for (int i = 0; i < solutions.size(); i++)
        {
            // get the current path and add it to the path
            std::vector<Pose> current_path = solutions[i].c.to_pose_vect(total_lenght);
            total_lenght += solutions[i].c.L;

            // add the current path to the path
            path.insert(path.end(), std::make_move_iterator(current_path.begin()), std::make_move_iterator(current_path.end()));

            if (total_lenght >= lenght)
            {
                break;
            }
        }

        return path;
    }

    /**
     * @brief Print the path on the image
     *
     * @param img The image to add the path to
     * @param index The index of the robot to color the curve
     */
    void show_path(cv::Mat &img, unsigned int index)
    {
        // for every Dubins Solution print it on the image
        for (int i = 0; i < solutions.size(); i++)
        {
            solutions[i].c.show_dcurve(img, index);
        }
    }

private:
    std::vector<Point> optimize_path(std::vector<Point> &path)
    {
        std::vector<Point> optimized_path;

        // push the first point of the path
        optimized_path.push_back(path[0]);

        float distance = 0.0;

        // for each element in the path to optimize
        for (int i = 1; i < path.size(); i++)
        {
            // iteratively increase the distance between the current and the previous point
            distance += distance_btw_points(path[i - 1], path[i]);

            // if the distance is above a threshold
            if (distance >= 0.1 || i == path.size() - 1)
            {
                // add the current point to the path, so discard the other point that would be close to it
                optimized_path.push_back(path[i]);
                distance = 0.0;
            }
        }

        return optimized_path;
    }

    float distance_btw_points(Point p1, Point p2)
    {
        // euclidean distance between two points
        return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    }
};
