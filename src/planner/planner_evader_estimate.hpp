#pragma once

#include "planner.hpp"

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

            std::map<int, float> distances = graph_map.get_robot_gate_distances(GraphMap::PointType::ESCAPER);

            std::fstream test_file("/home/ubuntu/workspace/project/state/evader_estimated_info.txt");
            if (!test_file.good())
            {
                std::ofstream file;
                file.open("/home/ubuntu/workspace/project/state/evader_estimated_info.txt", std::ofstream::out);

                for (std::map<int, float>::const_iterator it = distances.begin(); it != distances.end(); it++)
                {
                    file << "l" << it->first << "\t";
                }
                file << std::endl;

                file.close();
            }

            std::fstream file;
            file.open("/home/ubuntu/workspace/project/state/evader_estimated_info.txt", std::fstream::in | std::fstream::out | std::fstream::app);

            for (std::map<int, float>::const_iterator it = distances.begin(); it != distances.end(); it++)
            {
                file << it->second << "\t";
            }
            file << std::endl;

            std::map<std::string, std::vector<int>> gates;
            std::vector<std::string> gates_str;

            int count = 0;
            std::string line;

            while (std::getline(file, line))
            {
                if (count == 0)
                {
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates[str[i]] = std::vector<int>();
                        gates_str.push_back(str[i]);
                    }
                }
                else
                {
                    std::vector<std::string> str = split_string(line, "\t");
                    for (int i = 0; i < str.size(); i++)
                    {
                        gates[gates_str[i]].push_back(std::stoi(str[i]));
                    }
                }

                count++;
            }

            file.close();

            // if we do not have enought data use behaviour complexity 1
            if (count <= 2)
            {
                problem << ")" << std::endl;
                break;
            }

            int lenght_difference = 0;
            int index = 0;
            for (int i = 0; i < gates_str.size(); i++)
            {
                int size = gates[gates_str[i]].size();
                int current_lenght_difference = gates[gates_str[i]][size - 2] - gates[gates_str[i]][size - 1];

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
};
