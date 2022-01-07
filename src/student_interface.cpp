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

    bool planPath(const Polygon &border, const std::vector<Polygon> &obstacles, const std::vector<Polygon> &gates,
                  const std::vector<float> x, const std::vector<float> y, const std::vector<float> theta,
                  std::vector<Path> &paths, const std::string &config_folder)
    {
        int behavioural_complexity = 3;

        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        // Offsetting
        std::vector<Polygon> borders;
        borders.push_back(border);
        std::vector<Polygon> offsetted_borders = LineOffsetter::offset_polygons(borders, -95);
        std::vector<Polygon> offsetted_obstacles = LineOffsetter::offset_polygons(obstacles, 95); // 95

        // Merge and intersection
        std::vector<Polygon> merged_obstacles = LineOffsetter::merge_polygons(offsetted_obstacles);
        std::vector<Polygon> intersected_obstacles_borders = LineOffsetter::intersect_polygons(offsetted_borders, merged_obstacles);

        // Convex hull
        std::vector<Polygon> convex_hull = ConvexHull::create_convex_hull(intersected_obstacles_borders);

        // Cell decomposition
        CellDecomposition cell_decomposition;
        cell_decomposition.add_polygons(offsetted_borders);
        cell_decomposition.add_polygons(convex_hull);
        cell_decomposition.create_cdt();

        // Graph map
        GraphMap graph_map;
        graph_map.create_graph(cell_decomposition.triangles, cell_decomposition.points);
        graph_map.add_gates(gates);
        graph_map.add_robots(x, y);
        graph_map.optimize(convex_hull);

        // Routers
        Router *pursuer_router;
        Router *evader_router;

        // Planners
        PlannerEvader *evader_planner;
        PlannerEvaderEstimate *evader_estimated_planner;
        PlannerPursuer *pursuer_planner;

        // Planner for evader
        evader_planner = new PlannerEvader(graph_map, behavioural_complexity);
        evader_planner->write_problem();
        bool evader_plan_found = evader_planner->generate_plan();
        std::vector<Point> evader_path = evader_planner->extract_path_from_plan();

        // Planner for estimating evader path
        evader_estimated_planner = new PlannerEvaderEstimate(graph_map, behavioural_complexity);
        evader_estimated_planner->write_problem();
        bool evader_estimated_plan_found = evader_estimated_planner->generate_plan();
        std::vector<int> evader_estimated_path = evader_estimated_planner->extract_path_indexes_from_plan();

        if (evader_estimated_plan_found)
        {
            // Planner for pursuer
            pursuer_planner = new PlannerPursuer(graph_map, behavioural_complexity, evader_estimated_path);
            pursuer_planner->write_problem();
            bool pursuer_plan_found = pursuer_planner->generate_plan();
            std::vector<Point> pursuer_path = pursuer_planner->extract_path_from_plan();

            if (pursuer_plan_found)
            {
                // Router for pursuer
                pursuer_router = new Router();
                pursuer_router->add_path(pursuer_path, theta[0]);
                pursuer_router->elaborate_solution();
                std::vector<Pose> pursuer_solution = pursuer_router->get_path(1);
                paths[0].points = pursuer_solution;
            }
        }

        if (evader_plan_found)
        {
            // Route for evader
            evader_router = new Router();
            evader_router->add_path(evader_path, theta[1]);
            evader_router->elaborate_solution();
            std::vector<Pose> evader_solution = evader_router->get_path(1);
            paths[1].points = evader_solution;
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() / 1000 << "[ms]" << std::endl;

        bool debug_img = true;
        if (debug_img)
        {
            unsigned int size_x = 1000;
            unsigned int size_y = 800;

            cv::Mat img = cv::Mat(size_y, size_x, CV_8UC3);

            cell_decomposition.show_triangles(img);

            graph_map.show_graph(img);

            if (pursuer_planner != nullptr)
            {
                pursuer_planner->show_plan(img);
            }
            if (evader_planner != nullptr)
            {
                evader_planner->show_plan(img);
            }

            if (pursuer_router != nullptr)
            {
                pursuer_router->show_path(img, 0);
            }
            if (evader_router != nullptr)
            {
                evader_router->show_path(img, 1);
            }

            // cv::imshow("Image", img);
            // cv::waitKey(1);

            std::experimental::filesystem::path photo_path("/home/ubuntu/workspace/images");

            int last_image = 0;
            for (const auto &file : std::experimental::filesystem::directory_iterator(photo_path))
            {
                if (!std::experimental::filesystem::is_empty(file))
                {
                    last_image++;
                }
            }

            cv::imwrite("/home/ubuntu/workspace/images/image-" + std::to_string(last_image + 1) + ".png", img);
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
