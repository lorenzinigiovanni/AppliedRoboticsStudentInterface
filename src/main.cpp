#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"
#include "line_offsetter.hpp"
#include "cell_decomposition.hpp"
#include "graph_map.hpp"
#include "planner.hpp"
#include <iostream>

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

    theta.push_back(0.0);
    theta.push_back(0.0);
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

    cv::imshow("Image", img);
    cv::waitKey(0);

    return 0;
}
