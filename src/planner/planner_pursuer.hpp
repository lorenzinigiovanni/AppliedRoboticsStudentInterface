#pragma once

#include "planner.hpp"

/**
 * @brief Class used to compute the pursuer path
 *
 */
class PlannerPursuer : public virtual Planner
{
    std::vector<int> evader_estimated_path;

public:
    /**
     * @brief Construct a new Planner Pursuer object
     *
     * @param _graph_map The graph map with the map informations to be used by the planner
     * @param _complexity The behavioural complexity of the planner
     * @param _evader_estimated_path The estimated path of the evader
     */
    PlannerPursuer(
        GraphMap &_graph_map,
        int _complexity,
        std::vector<int> _evader_estimated_path)
        : Planner("pursuer", _graph_map, _complexity),
          evader_estimated_path(_evader_estimated_path)
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
        // The pursuing robot will be pursuing
        problem << "(pursuing)" << std::endl;
        problem << std::endl;

        int evader_distance = 0;

        // For each step in the estimated path of the evader
        for (int i = 0; i < evader_estimated_path.size(); i++)
        {
            if (i == evader_estimated_path.size() - 1)
            {
                // Pursuer goes to the destination of the evader if it cannot reach it before
                problem << "(= (evader-cost l" << evader_estimated_path[i] << ") 100000)" << std::endl;
            }
            else
            {
                // The pursuer should intercet the evader at a future estimated position of the evader before the evader itself
                evader_distance += (int)(1000 * graph_map.distance_btw_points(evader_estimated_path[i], evader_estimated_path[i - 1]));
                problem << "(= (evader-cost l" << evader_estimated_path[i] << ") " << evader_distance << ")" << std::endl;
            }
        }

        // Set a cost for the positions not part of the evader path to -1
        problem << get_missing_evader_cost_locations(evader_estimated_path);
    }

    /**
     * @brief Define the goal of the PDDL problem
     *
     * @param problem The PDDL problem to which the goal will be added
     */
    void define_goal(std::ofstream &problem)
    {
        // The pursuer should caught the evader
        problem << "(caught r1)" << std::endl;
    }

    /**
     * @brief Get the locations evader cost not part of the evader path
     *
     * @param _evader_estimated_path The estimated path of the evader
     * @return String containing the missing evader cost locations
     */
    std::string get_missing_evader_cost_locations(std::vector<int> _evader_estimated_path)
    {
        std::string data;

        // Get the locations not part of the evader path
        std::vector<int> missing_indexes = graph_map.get_missing_evader_locations_indexes(_evader_estimated_path);

        // For every location not part of the evader path
        for (int i = 0; i < missing_indexes.size(); i++)
        {
            // Set the cost to -1
            data += "(= (evader-cost l" + std::to_string(missing_indexes[i]) + ") -1)\n";
        }

        return data;
    }

    /**
     * @brief Get the vertex index of the position of the robot
     *
     * @return Index of the vertex of the position of the robot
     */
    int get_robot_location()
    {
        return graph_map.get_pursuer_index();
    }
};
