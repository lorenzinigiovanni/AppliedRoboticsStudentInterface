#pragma once

#include "planner.hpp"
#include "planner_distance.hpp"

class PlannerEvader : public virtual Planner
{
public:
    PlannerEvader(
        GraphMap &_graph_map,
        int _complexity)
        : Planner("evader", _graph_map, _complexity)
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
        // case 0:
        case 1:
        {
            problem << "(evaded r2)" << std::endl;
            break;
        }

        case 2:
        {
            problem << "(and" << std::endl;
            problem << "(evaded r2)" << std::endl;

            std::string choosen_gate_location = "";

            std::fstream test_file("/home/ubuntu/workspace/project/state/evader_info.txt");
            if (!test_file.good())
            {
                // decide gate toward which the evader will head
                choosen_gate_location = get_random_gate();

                std::ofstream file;
                file.open("/home/ubuntu/workspace/project/state/evader_info.txt", std::ofstream::out);
                file << choosen_gate_location << std::endl;
                file.close();
            }
            else
            {
                // read from file the decided gate toward which the evader is headed
                std::ifstream file;
                file.open("/home/ubuntu/workspace/project/state/evader_info.txt", std::ifstream::in);

                std::string line;
                std::getline(file, line);

                choosen_gate_location = line;
                file.close();
            }

            problem << "(at r2 " << choosen_gate_location << ")" << std::endl;
            problem << ")" << std::endl;
            break;
        }

        case 3:
        {
            problem << "(and" << std::endl;
            problem << "(evaded r2)" << std::endl;

            std::string choosen_gate_location = "";

            std::vector<int> gates = graph_map.get_gates_indexes();

            std::map<int, int> pursuer_distances;
            for (int i = 0; i < gates.size(); i++)
            {
                PlannerDistance distance_planner(graph_map, gates[i], 1);
                distance_planner.write_problem();
                distance_planner.generate_plan();
                pursuer_distances[gates[i]] = distance_planner.get_cost();
            }

            std::map<int, int> evader_distances;
            for (int i = 0; i < gates.size(); i++)
            {
                PlannerDistance distance_planner(graph_map, gates[i], 2);
                distance_planner.write_problem();
                distance_planner.generate_plan();
                evader_distances[gates[i]] = distance_planner.get_cost();
            }

            std::vector<std::vector<int>> distances(2);

            std::vector<std::string> gates_str;

            for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
            {
                gates_str.push_back(std::to_string(it->first));
            }

            for (std::map<int, int>::const_iterator it = evader_distances.begin(); it != evader_distances.end(); it++)
            {
                distances[0].push_back(it->second);
            }

            for (std::map<int, int>::const_iterator it = pursuer_distances.begin(); it != pursuer_distances.end(); it++)
            {
                distances[1].push_back(it->second);
            }

            int L = 100000;
            int index;
            for (int i = 0; i < distances[0].size(); i++)
            {
                int tmp = distances[0][i] - distances[1][i];
                if (tmp < L)
                {
                    index = i;
                    L = tmp;
                }
            }

            choosen_gate_location = "l" + gates_str[index];

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
};
