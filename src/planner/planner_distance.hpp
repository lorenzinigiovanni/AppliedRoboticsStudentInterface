#pragma once

#include "planner.hpp"

/**
 * @brief Class to be used to compute the distance between a point and a gate
 *
 */
class PlannerDistance : public virtual Planner
{
    int gate_location; // The gate location index
    int robot_index;   // The robot location index

public:
    /**
     * @brief Construct a new Planner Distance object
     *
     * @param _graph_map The graph map with the map informations to be used by the planner
     * @param _gate_location The gate location index
     * @param _robot_index The robot location index
     */
    PlannerDistance(
        GraphMap &_graph_map,
        int _gate_location,
        int _robot_index)
        : Planner("distance", _graph_map, 0),
          gate_location(_gate_location),
          robot_index(_robot_index)
    {
    }

protected:
    /**
     * @brief Initialize some objects of the PDDL problem
     *
     * @param problem The PDDL problem to which the definitions will be added
     */
    void initialize_objects(std::ofstream &problem)
    {
        // The pursuer robot will be pursuing
        if (robot_index == 1)
        {
            problem << "(pursuing)" << std::endl;
        }

        // The evading robot will be evading
        else if (robot_index == 2)
        {
            problem << "(evading)" << std::endl;
        }
    }

    /**
     * @brief Define the goal of the PDDL problem
     *
     * @param problem The PDDL problem to which the goal will be added
     */
    void define_goal(std::ofstream &problem)
    {
        // The goal is for the robot to reach the gate defined in the costructor
        problem << "(at r" << std::to_string(robot_index) << " l" << std::to_string(gate_location) << ")" << std::endl;
    }

    /**
     * @brief Get the vertex index of the position of the robot
     *
     * @return Index of the vertex of the position of the robot
     */
    int get_robot_location()
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
