#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"
#include "line_offsetter.hpp"
#include "cell_decomposition.hpp"
#include "graph_map.hpp"
#include "planner.hpp"
#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include <iostream>
#include <iterator>

int main()
{
    unsigned int size_x = 1000;
    unsigned int size_y = 800;
    cv::Mat img = cv::Mat(size_y, size_x, CV_8UC3);

    // Robot locations
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> theta;

    x.push_back(0.15);
    x.push_back(0.45);
    x.push_back(0.0);

    y.push_back(0.15);
    y.push_back(0.65);
    y.push_back(0.0);

    theta.push_back(M_PI_2);
    theta.push_back(M_PI_4);
    theta.push_back(0.0);

    // Borders arena
    Polygon border = Polygon({Point(0, 0), Point(0, 1), Point(0.8, 1), Point(0.8, 0)});

    // Gate positions
    std::vector<Polygon> gates;
    gates.push_back(Polygon({Point(0.2, 0), Point(0.4, 0), Point(0.4, 0.2), Point(0.2, 0.2)}));
    gates.push_back(Polygon({Point(0.6, 0.9), Point(0.8, 0.9), Point(0.8, 1), Point(0.6, 1)}));

    // Obstacle position
    std::vector<Polygon> obstacles;
    obstacles.push_back(Polygon({Point(0.2, 0.2), Point(0.2, 0.4), Point(0.4, 0.4), Point(0.4, 0.2)}));
    obstacles.push_back(Polygon({Point(0.35, 0.35), Point(0.35, 0.65), Point(0.65, 0.65), Point(0.65, 0.35)}));
    obstacles.push_back(Polygon({Point(0.6, 0.6), Point(0.6, 0.75), Point(0.75, 0.75), Point(0.75, 0.6)}));
    // obstacles.push_back(Polygon({Point(0.2, 0.4), Point(0.2, 0.3), Point(0.4, 0.2), Point(0.1, 0.4)}));

    std::vector<Polygon> borders;
    borders.push_back(border);

    std::vector<Polygon> offsetted_borders = LineOffsetter::offset_polygons(borders, -50);
    std::vector<Polygon> offsetted_obstacles = LineOffsetter::offset_polygons(obstacles, 50);

    std::vector<Polygon> merged_paths = LineOffsetter::merge_polygons(offsetted_obstacles);
    std::vector<Polygon> intersected_paths_borders = LineOffsetter::intersect_polygons(offsetted_borders, merged_paths);

    // cell decomposition
    CellDecomposition cell_decomposition;
    cell_decomposition.add_polygons(offsetted_borders);
    cell_decomposition.add_polygons(intersected_paths_borders);
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

    std::vector<dPoint> dpoints;

    for (int i = 0; i < pursuer_path.size(); i++)
    {
        float theta0 = 0.0;
        float theta1 = 0.0;

        if (i == 0)
        {
            dpoints.push_back(dPoint(pursuer_path[i], theta[0]));
        }
        else if (i == pursuer_path.size() - 1)
        {
            float dx = pursuer_path[i].x - pursuer_path[i - 1].x;
            float dy = pursuer_path[i].y - pursuer_path[i - 1].y;
            theta0 = std::atan2(dy, dx);
            dpoints.push_back(dPoint(pursuer_path[i], theta0));
        }
        else
        {
            float dx1 = pursuer_path[i].x - pursuer_path[i - 1].x;
            float dy1 = pursuer_path[i].y - pursuer_path[i - 1].y;
            theta0 = std::atan2(dy1, dx1);

            float dx2 = pursuer_path[i + 1].x - pursuer_path[i].x;
            float dy2 = pursuer_path[i + 1].y - pursuer_path[i].y;
            theta1 = std::atan2(dy2, dx2);

            float theta_tot_2 = (theta0 + theta1) / 2;

            dpoints.push_back(dPoint(pursuer_path[i], theta_tot_2));
        }
    }

    std::vector<Dubins::Solution> solutions;
    std::vector<Path> path;
    std::vector<Pose> total_path;

    path.resize(3);

    for (int i = 0; i < dpoints.size() - 1; i++)
    {
        solutions.push_back(Dubins::solve(dpoints[i], dpoints[i + 1], 40)); // dPoint, dPoint, max curvature
        if (solutions[i].pidx >= 0)
        {
            std::vector<Pose> escaper_paths_from_dCurve;
            cout << "Success to find a solution\n";

            cout << "a1: " << *(solutions[i].c.a1) << endl;
            cout << "a2: " << *(solutions[i].c.a2) << endl;
            cout << "a3: " << *(solutions[i].c.a3) << endl;
            cout << "L: " << solutions[i].c.L << endl;

            solutions[i].c.show_dcurve(img, 0);
            escaper_paths_from_dCurve = solutions[i].c.to_pose_vect();
            total_path.insert(total_path.end(), std::make_move_iterator(escaper_paths_from_dCurve.begin()), std::make_move_iterator(escaper_paths_from_dCurve.end()));
        }
        else
        {
            cout << "Failed to find a solution\n";
        }
    }

    for (int i = 0; i < total_path.size(); i++)
    {
        path[0].points.emplace_back(total_path[i]);
    }

    cv::imshow("Image", img);
    cv::waitKey(0);

    return 0;
}
