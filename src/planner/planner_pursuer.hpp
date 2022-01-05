#pragma once

#include "planner.hpp"

class PlannerPursuer : public virtual Planner
{
public:
    PlannerPursuer(
        GraphMap &_graph_map,
        int _complexity,
        std::vector<int> _evader_estimated_path)
        : Planner("pursuer", _graph_map, _complexity),
          evader_estimated_path(_evader_estimated_path)
    {
    }

    std::vector<int> evader_estimated_path;

protected:
    void initialize_objects(std::ofstream &problem)
    {
        problem << "(pursuing)" << std::endl;
        problem << std::endl;

        int evader_distance = 0;
        for (int i = 0; i < evader_estimated_path.size(); i++)
        {
            if (i == 0)
            {
                problem << "(= (evader-cost l" << evader_estimated_path[i] << ") 100000)" << std::endl;
            }
            else
            {
                evader_distance += (int)(1000 * graph_map.distance_btw_points(evader_estimated_path[i], evader_estimated_path[i - 1]));
                problem << "(= (evader-cost l" << evader_estimated_path[i] << ") " << evader_distance << ")" << std::endl;
            }
        }

        problem << get_missing_evader_cost_locations(evader_estimated_path);
    }

    void define_goal(std::ofstream &problem)
    {
        problem << "(caught r1)" << std::endl;
    }

    std::string get_missing_evader_cost_locations(std::vector<int> evader_index_path)
    {
        std::string data;
        std::vector<int> missing_indexes = graph_map.get_missing_evader_locations_indexes(evader_index_path);
        for (int i = 0; i < missing_indexes.size(); i++)
        {
            data += "(= (evader-cost l" + std::to_string(missing_indexes[i]) + ") -1)\n";
        }

        return data;
    }

    int virtual get_robot_location()
    {
        return graph_map.get_pursuer_index();
    }
};
