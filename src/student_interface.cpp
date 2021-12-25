#include "student_image_elab_interface.hpp"
#include "student_planning_interface.hpp"
#include "cell_decomposition.hpp"
#include "line_offsetter.hpp"
#include "graph_map.hpp"
#include "planner.hpp"

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
        cell_decomposition.show_triangles(img);

        GraphMap graph_map;
        graph_map.create_graph(cell_decomposition.triangles, cell_decomposition.points);
        graph_map.add_gates(gates);
        graph_map.add_robots(x, y);
        graph_map.show_graph(img);

        Planner escaper_planner("escaper", graph_map);
        escaper_planner.write_problem();
        escaper_planner.generate_plan();
        std::vector<Point> escaper_path = escaper_planner.extract_path_from_plan();
        escaper_planner.show_plan(img);

        Planner pursuer_planner("pursuer", graph_map);
        pursuer_planner.write_problem();
        pursuer_planner.generate_plan();
        std::vector<Point> pursuer_path = pursuer_planner.extract_path_from_plan();
        pursuer_planner.show_plan(img);

        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

        std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

        cv::imshow("Image", img);
        cv::waitKey(0);
    }
}
