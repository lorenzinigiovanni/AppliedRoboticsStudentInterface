#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "cell_decomposition.hpp"
#include "line_offsetter.hpp"
#include "graph_map.hpp"
#include "planner/planner_evader_estimate.hpp"
#include "planner/planner_evader.hpp"
#include "planner/planner_pursuer.hpp"
#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include "router.hpp"
#include "convex_hull.hpp"
#include "settings.hpp"
#include "image_handle.hpp"

#include <iostream>
#include <experimental/filesystem>

#include <algorithm>
#include <stdexcept>
#include <sstream>
#include <chrono>

namespace student
{

    void loadImage(cv::Mat &img_out, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - LOAD IMAGE - NOT IMPLEMENTED");
    }

    void genericImageListener(const cv::Mat &img_in, std::string topic, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - IMAGE LISTENER - NOT CORRECTLY IMPLEMENTED");
    }

    bool extrinsicCalib(const cv::Mat &img_in, std::vector<cv::Point3f> object_points,
                        const cv::Mat &camera_matrix, cv::Mat &rvec, cv::Mat &tvec, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - EXTRINSIC CALIB - NOT IMPLEMENTED");
    }

    void imageUndistort(const cv::Mat &img_in, cv::Mat &img_out,
                        const cv::Mat &cam_matrix, const cv::Mat &dist_coeffs, const std::string &config_folder)
    {

        throw std::logic_error("STUDENT FUNCTION - IMAGE UNDISTORT - NOT IMPLEMENTED");
    }

    void findPlaneTransform(const cv::Mat &cam_matrix, const cv::Mat &rvec,
                            const cv::Mat &tvec, const std::vector<cv::Point3f> &object_points_plane,
                            const std::vector<cv::Point2f> &dest_image_points_plane,
                            cv::Mat &plane_transf, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - FIND PLANE TRANSFORM - NOT IMPLEMENTED");
    }

    void unwarp(const cv::Mat &img_in, cv::Mat &img_out, const cv::Mat &transf,
                const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - UNWRAP - NOT IMPLEMENTED");
    }

    bool processMap(const cv::Mat &img_in, const double scale, std::vector<Polygon> &obstacles,
                    std::vector<std::pair<int, Polygon>> &victim_list, Polygon &gate, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - PROCESS MAP - NOT IMPLEMENTED");
    }

    bool findRobot(const cv::Mat &img_in, const double scale, Polygon &triangle,
                   double &x, double &y, double &theta, const std::string &config_folder)
    {
        throw std::logic_error("STUDENT FUNCTION - FIND ROBOT - NOT IMPLEMENTED");
    }

    /**
     * @brief Plan a path toward the gate for the evader robot and a path to intercept the evader robot for the pursuer robot
     *
     * @param border A polygon representing the border of the map
     * @param obstacles A vector of polygons representing the obstacles in the map
     * @param gates A vector of polygons representing the gates in the map
     * @param x A vector of x coordinates of the robots (0 pursuer, 1 evader)
     * @param y A vector of y coordinates of the robots (0 pursuer, 1 evader)
     * @param theta A vector of angles of the robots (0 pursuer, 1 evader)
     * @param paths A list of paths to be filled by the function (0 pursuer, 1 evader)
     * @param config_folder
     * @return true if the function succeeded in planning the paths
     * @return false if the function failed to plan the paths
     */
    bool planPath(const Polygon &border, const std::vector<Polygon> &obstacles, const std::vector<Polygon> &gates,
                  const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
                  std::vector<Path> &paths, const std::string &config_folder)
    {
        // Take the current time at the start of the planning function
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // // Graph Generation // //

        // Offset the borders by the max dimension of the robot
        std::vector<Polygon> borders;
        borders.push_back(border);
        std::vector<Polygon> offsetted_borders = LineOffsetter::offset_polygons(borders, -Settings::offset);

        // Offset the obstacles by the max dimension of the robot
        std::vector<Polygon> offsetted_obstacles = LineOffsetter::offset_polygons(obstacles, Settings::offset);

        // Merge the overlapping inflated obstacles in a single obstacle
        std::vector<Polygon> merged_obstacles = LineOffsetter::merge_polygons(offsetted_obstacles);
        // Intersect the obstacles with the borders of the map to prevent them to being outside the borders
        std::vector<Polygon> intersected_obstacles_borders = LineOffsetter::intersect_polygons(offsetted_borders, merged_obstacles);
        // Apply the convex hull algorithm to the obstacles
        std::vector<Polygon> obstacles_convex_hull = ConvexHull::create_convex_hull(intersected_obstacles_borders);

        // Merge obstacles and ofsetted borders vectors
        std::vector<Polygon> obstacles_and_borders;
        for (int i = 0; i < obstacles_convex_hull.size(); i++)
        {
            obstacles_and_borders.push_back(obstacles_convex_hull[i]);
        }
        for (int i = 0; i < offsetted_borders.size(); i++)
        {
            obstacles_and_borders.push_back(offsetted_borders[i]);
        }

        // Create an object for doing the cell decomposition operation
        CellDecomposition cell_decomposition;
        // Add the borders and the obstacles to the cell decomposition object
        cell_decomposition.add_polygons(offsetted_borders);
        cell_decomposition.add_polygons(obstacles_convex_hull);
        // Do the triangular cell decomposition
        cell_decomposition.create_cdt();

        // Create a graph map object
        GraphMap graph_map;
        // Initialize the graph map object with the triangles of the cell decomposition
        graph_map.create_graph(cell_decomposition.triangles, cell_decomposition.points);
        // Add the gates to the graph
        graph_map.add_gates(gates, obstacles_and_borders, borders);
        // Add the robots to the graph
        graph_map.add_robots(x, y);
        // Apply some optimizations to the graph
        graph_map.optimize(obstacles_and_borders);

        // // Path Planning // //

        // Planners
        PlannerEvader *evader_planner = NULL;
        PlannerEvaderEstimate *evader_estimated_planner = NULL;
        PlannerPursuer *pursuer_planner = NULL;

        // Planner for planning the evader path
        evader_planner = new PlannerEvader(graph_map, Settings::behavioural_complexity);
        // Create a PDDL file containing the problem to be solved
        evader_planner->write_problem();
        // Call the planner to generate the plan
        bool evader_plan_found = evader_planner->generate_plan();
        // Get the path from the planner
        std::vector<Point> evader_path = evader_planner->extract_path_from_plan();

        // Planner for estimating evader path, used by the pursuer
        evader_estimated_planner = new PlannerEvaderEstimate(graph_map, Settings::behavioural_complexity);
        // Create a PDDL file containing the problem to be solved
        evader_estimated_planner->write_problem();
        // Call the planner to generate the plan
        bool evader_estimated_plan_found = evader_estimated_planner->generate_plan();
        // Get the path from the planner
        std::vector<int> evader_estimated_path = evader_estimated_planner->extract_path_indexes_from_plan();

        // Results of the pursuer planner
        bool pursuer_plan_found = false;
        std::vector<Point> pursuer_path;

        // The pursuer planner can only run if a estimated plan for the evader is found
        if (evader_estimated_plan_found)
        {
            // Planner for planning the pursuer path
            pursuer_planner = new PlannerPursuer(graph_map, Settings::behavioural_complexity, evader_estimated_path);
            // Create a PDDL file containing the problem to be solved
            pursuer_planner->write_problem();
            // Call the planner to generate the plan
            pursuer_plan_found = pursuer_planner->generate_plan();
            // Get the path from the planner
            pursuer_path = pursuer_planner->extract_path_from_plan();
        }

        // // Path Computation // //

        // Routers
        Router *evader_router = NULL;
        Router *pursuer_router = NULL;

        // Run the Dubins algorthm only if a evader plan has been found
        if (evader_plan_found)
        {
            // Router for evader
            evader_router = new Router();
            // Set the found plan in the evader router along with its starting angle
            evader_router->add_path(evader_path, theta[1], obstacles_and_borders);
            // Compute the evader path
            evader_router->elaborate_solution();
            // Extract the evader path and put it in the second position of the paths vector
            std::vector<Pose> evader_poses = evader_router->get_path(Settings::path_length);
            for (int i = 0; i < evader_poses.size(); i++)
            {
                paths[1].points.emplace_back(evader_poses[i].s, evader_poses[i].x, evader_poses[i].y, evader_poses[i].theta, evader_poses[i].kappa);
            }
        }

        // Run the Dubins algorthm only if a pursuer plan has been found
        if (pursuer_plan_found)
        {
            // Router for pursuer
            pursuer_router = new Router();
            // Set the found plan in the puruser router along with its starting angle
            pursuer_router->add_path(pursuer_path, theta[0], obstacles_and_borders);
            // Compute the pursuer path
            pursuer_router->elaborate_solution();
            // Extract the pursuer path and put it in the first position of the paths vector
            std::vector<Pose> pursuer_poses = pursuer_router->get_path(Settings::path_length);
            for (int i = 0; i < pursuer_poses.size(); i++)
            {
                paths[0].points.emplace_back(pursuer_poses[i].s, pursuer_poses[i].x, pursuer_poses[i].y, pursuer_poses[i].theta, pursuer_poses[i].kappa);
            }
        }

        // Take the current time at the end of the planning function
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //  Print the time taken to execute the function to the console
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000 << "[ms]" << std::endl;

        // // Debug Images // //

        if (Settings::debug_img)
        {
            unsigned int size_x = 780;
            unsigned int size_y = 530;

            std::string path = Settings::workspace_path + "images/";

            cv::Mat img = cv::Mat::zeros(size_y, size_x, CV_8UC3);

            if (Settings::deep_debug_img)
            {
                show_polygon_list(img, obstacles);
                show_polygon_list(img, borders);
                show_polygon_list(img, gates);
                write_img(img, path);

                img = cv::Mat::zeros(size_y, size_x, CV_8UC3);

                show_polygon_list(img, offsetted_borders);
                show_polygon_list(img, intersected_obstacles_borders);
                write_img(img, path);

                img = cv::Mat::zeros(size_y, size_x, CV_8UC3);
            }

            show_polygon_list(img, offsetted_borders);
            show_polygon_list(img, obstacles_convex_hull);
            if (Settings::deep_debug_img)
            {
                write_img(img, path);
            }

            cell_decomposition.show_triangles(img);
            if (Settings::deep_debug_img)
            {
                write_img(img, path);
            }

            graph_map.show_graph(img);
            if (Settings::deep_debug_img)
            {
                write_img(img, path);
            }

            if (pursuer_planner)
            {
                pursuer_planner->show_plan(img);
            }
            if (evader_planner)
            {
                evader_planner->show_plan(img);
            }
            if (Settings::deep_debug_img)
            {
                write_img(img, path);
            }

            if (pursuer_router)
            {
                pursuer_router->show_path(img, 0);
                pursuer_router->show_collision_points(img, obstacles_and_borders);
            }
            if (evader_router)
            {
                evader_router->show_path(img, 1);
                evader_router->show_collision_points(img, obstacles_and_borders);

            }

            write_img(img, path);
        }

        // Deletes
        delete pursuer_router;
        delete evader_router;
        delete evader_planner;
        delete evader_estimated_planner;
        delete pursuer_planner;

        return true;
    }
}
