#pragma once

#include "planner.hpp"

class PlannerDistance : public virtual Planner
{
    int gate_location;
    int robot_index;

public:
    PlannerDistance(
        GraphMap &_graph_map,
        int _gate_location,
        int _robot_index)
        : Planner("distance", _graph_map, 0),
          gate_location(_gate_location),
          robot_index(_robot_index)
    {
    }

    int get_cost()
    {
        return cost;
    }

protected:
    void initialize_objects(std::ofstream &problem)
    {
        if (robot_index == 1)
        {
            problem << "(pursuing)" << std::endl;
        }
        else if (robot_index == 2)
        {
            problem << "(evading)" << std::endl;
        }
    }

    void define_goal(std::ofstream &problem)
    {
        problem << "(at r" << std::to_string(robot_index) << " l" << std::to_string(gate_location) << ")" << std::endl;
    }

    int virtual get_robot_location()
    {
        if (robot_index == 1)
        {
            return graph_map.get_pursuer_index();
        }
        else if (robot_index == 2)
        {
            return graph_map.get_evader_index();
        }
    }
};
