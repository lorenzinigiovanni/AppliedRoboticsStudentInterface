#pragma once

#include "planner.hpp"
#include "planner_distance.hpp"

/**
 * @brief Class to be used by the pursuer for estimating the path that the evader will take
 *
 */
class PlannerEvaderEstimate : public virtual Planner
{
public:
    /**
     * @brief Construct a new Planner Evader Estimate object
     *
     * @param _graph_map The graph map with the map informations to be used by the planner
     * @param _complexity The behavioural complexity of the planner
     */
    PlannerEvaderEstimate(
        GraphMap &_graph_map,
        int _complexity)
        : Planner("evader_estimated", _graph_map, _complexity)
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

            // For behavioural complexity 2 and 3 the goal is to go to the gate to which the evader robot is evading
        case 2:
        case 3:
        {
            problem << "(and" << std::endl;

            // The evader should evade
            problem << "(evaded r2)" << std::endl;

            // Get all gates possible indexes
            std::vector<int> gates = graph_map.get_gates_indexes();

            // Map to store the current distance between the evader robot and a gate
            std::map<int, int> evader_distances;

            // For each gate in the graph map
            for (int i = 0; i < gates.size(); i++)
            {
                // Use the PDDL planner to get the distance between the evader and the gate
                PlannerDistance distance_planner(graph_map, gates[i], 2);
                distance_planner.write_problem();
                distance_planner.generate_plan();
                distance_planner.extract_path_indexes_from_plan();

                // Store the distance between the evader and the gate
                evader_distances[gates[i]] = distance_planner.get_cost();
            }

            // Check if the file storing the distances exists
            std::fstream test_file("/home/ubuntu/workspace/state/evader_estimated_info.txt");
            if (!test_file.good())
            {
                // If it doesn't exist create it
                std::ofstream file;
                file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::ofstream::out);

                // Create an header with the gates names
                for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
                {
                    file << "l" << it->first << "\t";
                }
                file << std::endl;

                file.close();
            }

            // Open the distances file to store the distances
            std::ofstream out_file;
            out_file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::fstream::out | std::fstream::app);

            // Store the distances between each gate and the robot in the file
            for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
            {
                out_file << it->second << "\t";
            }
            out_file << std::endl;

            out_file.close();

            // Map to store for each gate the distances to the evader in the current and previous steps
            std::map<std::string, std::vector<int>> gates_map;

            // Vector to store the gates names
            std::vector<std::string> gates_str;

            int count = 0;
            std::string line;

            // Open the distances files to read the distances
            std::ifstream in_file;
            in_file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::fstream::in);

            // For every line in the file
            while (std::getline(in_file, line))
            {
                // For the first line
                if (count == 0)
                {
                    // Read the header containing the gates names and save them in the vector and map
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates_map[str[i]] = std::vector<int>();
                        gates_str.push_back(str[i]);
                    }
                }
                else
                {
                    // Read the distances between evader robot and gates and store them in the map
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates_map[gates_str[i]].push_back(std::atoi(str[i].c_str()));
                    }
                }

                count++;
            }

            in_file.close();

            // If we do not have enought data use behavioural complexity 1
            // The evader robot should evade towards the gate with the shortest distance
            if (count <= 2)
            {
                problem << ")" << std::endl;
                break;
            }

            int lenght_difference = -10000;
            int index = 0;

            // Search the gate to whom the escaper robot have the major decrease in distance and so it's heading to 
            for (int i = 0; i < gates_str.size(); i++)
            {
                // The number of measured distances (steps)
                int size = gates_map[gates_str[i]].size();

                // Distance difference
                int current_lenght_difference = gates_map[gates_str[i]][size - 2] - gates_map[gates_str[i]][size - 1];

                if (current_lenght_difference > lenght_difference)
                {
                    lenght_difference = current_lenght_difference;
                    index = i;
                }
            }

            // The evader robot should go to the gate calculated above
            problem << "(at r2 " << gates_str[index] << ")" << std::endl;
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
