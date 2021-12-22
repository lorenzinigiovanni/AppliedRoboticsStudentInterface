#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include "clipper/clipper.hpp"
#include "line_offsetter.hpp"
#include "cell_decomposition.hpp"
#include <iostream>

int main()
{
    Polygon border = Polygon({Point(0, 0), Point(0, 1), Point(0.8, 1), Point(0.8, 0)});

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
    cell_decomposition.print_triangles();
    cell_decomposition.show_triangles();

    return 0;
}
