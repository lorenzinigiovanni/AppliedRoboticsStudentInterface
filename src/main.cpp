#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"
#include "line_offsetter.hpp"
#include "convex_hull.hpp"
#include "cell_decomposition.hpp"
#include "graph_map.hpp"
#include "planner/planner_evader_estimate.hpp"
#include "planner/planner_evader.hpp"
#include "planner/planner_pursuer.hpp"
#include "dubins/dpoint.hpp"
#include "dubins/dubins.hpp"
#include "router.hpp"
#include <iostream>
#include <iterator>

/**
 * @brief File used to test our code before using it in the simulator
 *
 * @return int
 */
int main()
{
    int behavioural_complexity = 3;

    // Robot locations
    std::vector<float> x;
    std::vector<float> y;
    std::vector<float> theta;

    // Robot 1 location
    x.push_back(0.2);
    y.push_back(0.15);
    theta.push_back(M_PI_2);

    // Robot 1 location
    x.push_back(0.15);
    y.push_back(0.65);
    theta.push_back(M_PI_4);

    // Robot 2 location
    x.push_back(0.0);
    y.push_back(0.0);
    theta.push_back(0.0);

    // Borders arena
    Polygon border = Polygon({Point(0, 0), Point(0, 1), Point(0.8, 1), Point(0.8, 0)});

    // Gate positions
    std::vector<Polygon> gates;
    // gates.push_back(Polygon({Point(0.2, 0), Point(0.4, 0), Point(0.4, 0.2), Point(0.2, 0.2)}));
    gates.push_back(Polygon({Point(0.6, 0.5), Point(0.8, 0.5), Point(0.8, 0.4), Point(0.6, 0.4)}));

    // Obstacle position
    std::vector<Polygon> obstacles;
    obstacles.push_back(Polygon({Point(0.2, 0.2), Point(0.2, 0.4), Point(0.4, 0.4), Point(0.4, 0.2)}));
    // obstacles.push_back(Polygon({Point(0.35, 0.35), Point(0.35, 0.65), Point(0.65, 0.65), Point(0.65, 0.35)}));
    obstacles.push_back(Polygon({Point(0.6, 0.6), Point(0.6, 0.75), Point(0.75, 0.75), Point(0.75, 0.6)}));
    // obstacles.push_back(Polygon({Point(0.2, 0.4), Point(0.2, 0.3), Point(0.4, 0.2), Point(0.1, 0.4)}));

    // Offsetting
    std::vector<Polygon> borders;
    borders.push_back(border);
    std::vector<Polygon> offsetted_borders = LineOffsetter::offset_polygons(borders, -50);
    std::vector<Polygon> offsetted_obstacles = LineOffsetter::offset_polygons(obstacles, 50);

    // Merge and intersection
    std::vector<Polygon> merged_obstacles = LineOffsetter::merge_polygons(offsetted_obstacles);
    std::vector<Polygon> intersected_obstacles_borders = LineOffsetter::intersect_polygons(offsetted_borders, merged_obstacles);

    // Convex hull
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

    // Cell decomposition
    CellDecomposition cell_decomposition;
    cell_decomposition.add_polygons(offsetted_borders);
    cell_decomposition.add_polygons(obstacles_convex_hull);
    cell_decomposition.create_cdt();

    // Graph map
    GraphMap graph_map;
    graph_map.create_graph(cell_decomposition.triangles, cell_decomposition.points);
    graph_map.add_gates(gates, obstacles_and_borders, borders);
    graph_map.add_robots(x, y);
    graph_map.optimize(obstacles_convex_hull);

    // Planners
    PlannerEvader *evader_planner = NULL;
    PlannerEvaderEstimate *evader_estimated_planner = NULL;
    PlannerPursuer *pursuer_planner = NULL;

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

    bool pursuer_plan_found = false;
    std::vector<Point> pursuer_path;

    if (evader_estimated_plan_found)
    {
        // Planner for pursuer
        pursuer_planner = new PlannerPursuer(graph_map, behavioural_complexity, evader_estimated_path);
        pursuer_planner->write_problem();
        pursuer_plan_found = pursuer_planner->generate_plan();
        pursuer_path = pursuer_planner->extract_path_from_plan();
        }

    // // Path Computation // //

    // Routers
    Router *pursuer_router = NULL;
    Router *evader_router = NULL;

    if (evader_plan_found)
    {
        // Route for evader
        evader_router = new Router();
        evader_router->add_path(evader_path, theta[1], obstacles_and_borders);
        evader_router->elaborate_solution();
        std::vector<Pose> evader_solution = evader_router->get_path(1);
    }

    if (pursuer_plan_found)
    {
        // Router for pursuer
        pursuer_router = new Router();
        pursuer_router->add_path(pursuer_path, theta[0], obstacles_and_borders);
        pursuer_router->elaborate_solution();
        std::vector<Pose> pursuer_solution = pursuer_router->get_path(1);
    }

    bool debug_img = true;
    if (debug_img)
    {
        unsigned int size_x = 900;
        unsigned int size_y = 650;

        cv::Mat img = cv::Mat::zeros(size_y, size_x, CV_8UC3);

        cell_decomposition.show_triangles(img);

        graph_map.show_graph(img);

        if (pursuer_planner)
        {
            pursuer_planner->show_plan(img);
        }
        if (evader_planner)
        {
            evader_planner->show_plan(img);
        }

        if (pursuer_router)
        {
            pursuer_router->show_path(img, 0);
        }
        if (evader_router)
        {
            evader_router->show_path(img, 1);
        }

        cv::imshow("Image", img);
        cv::waitKey(0);
    }

    // Deletes
    delete pursuer_router;
    delete evader_router;
    delete evader_planner;
    delete evader_estimated_planner;
    delete pursuer_planner;

    return 0;
}
