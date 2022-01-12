#pragma once

#include "planner.hpp"
#include "planner_distance.hpp"

/**
 * @brief Class used to compute the evader path
 *
 */
class PlannerEvader : public virtual Planner
{
public:
    /**
     * @brief Construct a new Planner Evader object
     *
     * @param _graph_map The graph map with the map informations to be used by the planner
     * @param _complexity The behavioural complexity of the planner
     */
    PlannerEvader(
        GraphMap &_graph_map,
        int _complexity)
        : Planner("evader", _graph_map, _complexity)
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
        // The evading robot will be evading
        problem << "(evading)" << std::endl;
    }

    /**
     * @brief Define the goal of the PDDL problem
     *
     * @param problem The PDDL problem to which the goal will be added
     */
    void define_goal(std::ofstream &problem)
    {
        // The goal depends on the behaviour complexity of the robot
        switch (complexity)
        {
            // For behavioural complexity 1 the goal is to reach the gate in the fastest way
        case 1:
        {
            // The evader should evade
            problem << "(evaded r2)" << std::endl;
            break;
        }

            // For behavioural complexity 2 the goal is to reach a random gate in the fastest way
        case 2:
        {
            problem << "(and" << std::endl;

            // The evader should evade
            problem << "(evaded r2)" << std::endl;

            std::string choosen_gate_location = "";

            std::fstream test_file(Settings::workspace_path + "state/evader_info.txt");

            // If the file with the choosen gate does not exist, create it and choose a random gate
            if (!test_file.good())
            {
                // Decide gate toward which the evader will head
                choosen_gate_location = get_random_gate();

                std::ofstream file;
                file.open(Settings::workspace_path + "state/evader_info.txt", std::ofstream::out);
                file << choosen_gate_location << std::endl;
                file.close();
            }

            // If the file with the choosen gate already exists, read it, and keep the choosen gate as destination
            else
            {
                std::ifstream file;
                file.open(Settings::workspace_path + "state/evader_info.txt", std::ifstream::in);

                std::string line;
                std::getline(file, line);

                choosen_gate_location = line;
                file.close();
            }

            // The robot evader robot should go to the gate choosed above
            problem << "(at r2 " << choosen_gate_location << ")" << std::endl;
            problem << ")" << std::endl;
            break;
        }

            // For behavioural complexity 3 the goal is to reach the gate that is the closest to the evader robot and the furthest from the pursuer robot
        case 3:
        {
            problem << "(and" << std::endl;

            // The evader should evade
            problem << "(evaded r2)" << std::endl;

            // Get all gates possible indexes
            std::vector<int> gates = graph_map.get_gates_indexes();

            std::vector<std::vector<int>> distances(2);
            std::vector<std::string> gates_str;

            // For each gate in the graph map
            for (int i = 0; i < gates.size(); i++)
            {
                // Use the PDDL planner to get the distance between the evader and the gate
                PlannerDistance distance_evader_planner(graph_map, gates[i], 2);
                distance_evader_planner.write_problem();
                bool found_evader_plan = distance_evader_planner.generate_plan();

                // If the evader can find a plan towards this gate
                if (found_evader_plan)
                {
                    // Store the distance between the evader and the gate
                    distance_evader_planner.extract_path_indexes_from_plan();
                    distances[0].push_back(distance_evader_planner.get_cost());

                    // Save the gate as a possibile destination
                    gates_str.push_back(std::to_string(gates[i]));

                    // Use the PDDL planner to get the distance between the pursuer and the gate
                    PlannerDistance distance_pursuer_planner(graph_map, gates[i], 1);
                    distance_pursuer_planner.write_problem();
                    bool found_pursuer_plan = distance_pursuer_planner.generate_plan();

                    // If the pursuer can find a plan towards this gate
                    if (found_pursuer_plan)
                    {
                        // Store the distance between the pursuer and the gate
                        distance_pursuer_planner.extract_path_indexes_from_plan();
                        distances[1].push_back(distance_pursuer_planner.get_cost());
                    }
                    else
                    {
                        // Otherwise set the distance to a very high value
                        distances[1].push_back(100000);
                    }
                }
            }

            int L = 100000;
            int index;

            // For each possibile gate
            for (int i = 0; i < distances[0].size(); i++)
            {
                // Distance between the evader and the gate[i] minus the distance between the pursuer and the gate[i]
                int tmp = (distances[0][i] - distances[1][i]) / (distances[0][i] + distances[1][i]);

                // If it is the smaller length keep the index of the gate
                if (tmp < L)
                {
                    index = i;
                    L = tmp;
                }
            }

            // The choosen gate is l and the found index
            std::string choosen_gate_location = "l" + gates_str[index];

            // The evader robot should go to the gate calculated above
            problem << "(at r2 " << choosen_gate_location << ")" << std::endl;
            problem << ")" << std::endl;
            break;
        }

        default:
        {
            problem << "(evaded r2)" << std::endl;
            break;
        }
        }
    }

    /**
     * @brief Get the vertex index of the position of the robot
     *
     * @return Index of the vertex of the position of the robot
     */
    int get_robot_location()
    {
        return graph_map.get_evader_index();
    }
};
