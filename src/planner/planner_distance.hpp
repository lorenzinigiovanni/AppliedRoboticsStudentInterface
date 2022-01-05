#pragma once

#include "planner.hpp"

class PlannerDistance : public virtual Planner
{
public:
    std::string gate_location;
    int robot_index;

    PlannerDistance(
        GraphMap &_graph_map,
        int _complexity,
        std::string _gate_location,
        int _robot_index)
        : Planner("distance", _graph_map, _complexity),
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
        problem << "(at r" << std::to_string(robot_index) << " " << gate_location << ")" << std::endl;
    }
};
