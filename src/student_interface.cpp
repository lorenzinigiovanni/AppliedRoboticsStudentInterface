#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "cell_decomposition.hpp"
#include "line_offsetter.hpp"
#include "graph_map.hpp"
#include "planner.hpp"
#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"

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
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        unsigned int size_x = 1000;
        unsigned int size_y = 800;
        cv::Mat img = cv::Mat(size_y, size_x, CV_8UC3);

        // offsetting
        std::vector<Polygon> borders;
        borders.push_back(border);
        std::vector<Polygon> offsetted_borders = LineOffsetter::offset_polygons(borders, -50);
        std::vector<Polygon> offsetted_obstacles = LineOffsetter::offset_polygons(obstacles, 50);

        // merge and intersection
        std::vector<Polygon> merged_obstacles = LineOffsetter::merge_polygons(offsetted_obstacles);
        std::vector<Polygon> intersected_obstacles_borders = LineOffsetter::intersect_polygons(offsetted_borders, merged_obstacles);

        // cell decomposition
        CellDecomposition cell_decomposition;
        cell_decomposition.add_polygons(offsetted_borders);
        cell_decomposition.add_polygons(intersected_obstacles_borders);
        cell_decomposition.create_cdt();
        // cell_decomposition.print_triangles();
        // cell_decomposition.show_triangles(img);

        GraphMap graph_map;
        graph_map.create_graph(cell_decomposition.triangles, cell_decomposition.points);
        graph_map.add_gates(gates);
        graph_map.add_robots(x, y);
        // graph_map.show_graph(img);

        Planner escaper_planner("escaper", graph_map);
        escaper_planner.write_problem();
        escaper_planner.generate_plan();
        std::vector<Point> escaper_path = escaper_planner.extract_path_from_plan();
        // escaper_planner.show_plan(img);

        Planner pursuer_planner("pursuer", graph_map);
        pursuer_planner.write_problem();
        pursuer_planner.generate_plan();
        std::vector<Point> pursuer_path = pursuer_planner.extract_path_from_plan();
        // pursuer_planner.show_plan(img);

        // ESCAPER

        {
            std::vector<dPoint> escaper_dpoints;

            for (int i = 0; i < escaper_path.size(); i++)
            {
                float theta0 = 0.0;
                float theta1 = 0.0;

                if (i == 0)
                {
                    escaper_dpoints.push_back(dPoint(escaper_path[i], theta[1])); // suppose theta escaper = theta[1]
                }
                else if (i == escaper_path.size() - 1)
                {
                    float dx = escaper_path[i].x - escaper_path[i - 1].x;
                    float dy = escaper_path[i].y - escaper_path[i - 1].y;
                    theta0 = std::atan2(dy, dx);
                    escaper_dpoints.push_back(dPoint(escaper_path[i], theta0));
                }
                else
                {
                    float dx1 = escaper_path[i].x - escaper_path[i - 1].x;
                    float dy1 = escaper_path[i].y - escaper_path[i - 1].y;
                    theta0 = std::atan2(dy1, dx1);

                    float dx2 = escaper_path[i + 1].x - escaper_path[i].x;
                    float dy2 = escaper_path[i + 1].y - escaper_path[i].y;
                    theta1 = std::atan2(dy2, dx2);

                    float theta_tot = (theta0 + theta1) / 2;

                    escaper_dpoints.push_back(dPoint(escaper_path[i], theta_tot));
                }
            }

            std::vector<Dubins::Solution> escaper_solutions;
            std::vector<Pose> tmp_path;

            for (int i = 0; i < escaper_dpoints.size() - 1; i++)
            {
                escaper_solutions.push_back(Dubins::solve(escaper_dpoints[i], escaper_dpoints[i + 1], 40)); // dPoint, dPoint, max curvature

                if (escaper_solutions[i].pidx >= 0)
                {
                    std::vector<Pose> escaper_paths_from_dCurve;

                    // escaper_solutions[i].c.show_dcurve(img, 1);
                    escaper_paths_from_dCurve = escaper_solutions[i].c.to_pose_vect();

                    tmp_path.insert(tmp_path.end(), std::make_move_iterator(escaper_paths_from_dCurve.begin()), std::make_move_iterator(escaper_paths_from_dCurve.end()));
                }
                else
                {
                    cout << "Failed to find a solution\n";
                }
            }

            paths[1].points.clear();

            for (int i = 0; i < tmp_path.size(); i++)
            {
                paths[1].points.emplace_back(tmp_path[i]); // paths[1] = escaper path
            }
        }

        // PURSUER

        {
            std::vector<dPoint> pursuer_dpoints;

            for (int i = 0; i < pursuer_path.size(); i++)
            {
                float theta0 = 0.0;
                float theta1 = 0.0;

                if (i == 0)
                {
                    pursuer_dpoints.push_back(dPoint(pursuer_path[i], theta[0])); // supponiamo theta pursuer theta[0]
                }
                else if (i == pursuer_path.size() - 1)
                {
                    float dx = pursuer_path[i].x - pursuer_path[i - 1].x;
                    float dy = pursuer_path[i].y - pursuer_path[i - 1].y;
                    theta0 = std::atan2(dy, dx);
                    pursuer_dpoints.push_back(dPoint(pursuer_path[i], theta0));
                }
                else
                {
                    float dx1 = pursuer_path[i].x - pursuer_path[i - 1].x;
                    float dy1 = pursuer_path[i].y - pursuer_path[i - 1].y;
                    theta0 = std::atan2(dy1, dx1);

                    float dx2 = pursuer_path[i + 1].x - pursuer_path[i].x;
                    float dy2 = pursuer_path[i + 1].y - pursuer_path[i].y;
                    theta1 = std::atan2(dy2, dx2);

                    float theta_tot = (theta0 + theta1) / 2;

                    pursuer_dpoints.push_back(dPoint(pursuer_path[i], theta_tot));
                }
            }

            std::vector<Dubins::Solution> pursuer_solutions;
            std::vector<Pose> tmp_path;

            for (int i = 0; i < pursuer_dpoints.size() - 1; i++)
            {
                pursuer_solutions.push_back(Dubins::solve(pursuer_dpoints[i], pursuer_dpoints[i + 1], 40)); // dPoint, dPoint, max curvature
                if (pursuer_solutions[i].pidx >= 0)
                {
                    std::vector<Pose> escaper_paths_from_dCurve;

                    // pursuer_solutions[i].c.show_dcurve(img, 0);
                    escaper_paths_from_dCurve = pursuer_solutions[i].c.to_pose_vect();

                    tmp_path.insert(tmp_path.end(), std::make_move_iterator(escaper_paths_from_dCurve.begin()), std::make_move_iterator(escaper_paths_from_dCurve.end()));
                }
                else
                {
                    cout << "Failed to find a solution\n";
                }
            }

            paths[0].points.clear();

            for (int i = 0; i < tmp_path.size(); i++)
            {
                paths[0].points.emplace_back(tmp_path[i]);
            }
        }

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

        // cv::imshow("Image", img);
        // cv::waitKey(0);

        return true;
    }
}
