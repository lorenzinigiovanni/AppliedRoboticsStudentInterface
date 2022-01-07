#pragma once

#include "planner.hpp"
#include "planner_distance.hpp"

class PlannerEvaderEstimate : public virtual Planner
{
public:
    PlannerEvaderEstimate(
        GraphMap &_graph_map,
        int _complexity)
        : Planner("evader_estimated", _graph_map, _complexity)
    {
    }

protected:
    void initialize_objects(std::ofstream &problem)
    {
        problem << "(evading)" << std::endl;
    }

    void define_goal(std::ofstream &problem)
    {
        switch (complexity)
        {
        case 1:
        {
            problem << "(evaded r2)" << std::endl;
            break;
        }

        case 2:
        case 3:
        {
            problem << "(and" << std::endl;
            problem << "(evaded r2)" << std::endl;

            std::vector<int> gates = graph_map.get_gates_indexes();

            std::map<int, int> evader_distances;
            for (int i = 0; i < gates.size(); i++)
            {
                PlannerDistance distance_planner(graph_map, gates[i], 2);
                distance_planner.write_problem();
                distance_planner.generate_plan();
                distance_planner.extract_path_indexes_from_plan();
                evader_distances[gates[i]] = distance_planner.get_cost();
            }

            std::fstream test_file("/home/ubuntu/workspace/state/evader_estimated_info.txt");
            if (!test_file.good())
            {
                std::ofstream file;
                file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::ofstream::out);

                for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
                {
                    file << "l" << it->first << "\t";
                }
                file << std::endl;

                file.close();
            }

            std::ofstream out_file;
            out_file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::fstream::out | std::fstream::app);

            for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
            {
                out_file << it->second << "\t";
            }
            out_file << std::endl;

            out_file.close();

            std::map<std::string, std::vector<int>> gates_map;
            std::vector<std::string> gates_str;

            int count = 0;
            std::string line;

            std::ifstream in_file;
            in_file.open("/home/ubuntu/workspace/state/evader_estimated_info.txt", std::fstream::in);

            while (std::getline(in_file, line))
            {
                if (count == 0)
                {
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates_map[str[i]] = std::vector<int>();
                        gates_str.push_back(str[i]);
                    }
                }
                else
                {
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates_map[gates_str[i]].push_back(std::atoi(str[i].c_str()));
                    }
                }

                count++;
            }

            in_file.close();

            // if we do not have enought data use behaviour complexity 1
            if (count <= 2)
            {
                problem << ")" << std::endl;
                break;
            }

            int lenght_difference = -10000;
            int index = 0;
            for (int i = 0; i < gates_str.size(); i++)
            {
                int size = gates_map[gates_str[i]].size();
                int current_lenght_difference = gates_map[gates_str[i]][size - 2] - gates_map[gates_str[i]][size - 1];

                if (current_lenght_difference > lenght_difference)
                {
                    lenght_difference = current_lenght_difference;
                    index = i;
                }
            }

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

    int virtual get_robot_location()
    {
        return graph_map.get_evader_index();
    }
};
