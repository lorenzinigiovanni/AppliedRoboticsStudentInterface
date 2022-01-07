#pragma once

#include "../graph_map.hpp"
#include "../boost/graph/adjacency_list.hpp"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <memory>
#include <string>
#include <array>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>

class Planner
{
protected:
    std::string problem_name;
    std::vector<std::vector<std::string>> data;
    std::vector<Point> path;
    GraphMap graph_map;
    int complexity;
    int cost;

    Planner(std::string _problem_name,
            GraphMap &_graph_map,
            int _complexity)
        : problem_name(_problem_name),
          graph_map(_graph_map),
          complexity(_complexity)
    {
    }

public:
    void write_problem()
    {
        std::ofstream problem;
        problem.open("/home/ubuntu/workspace/project/src/pddl/" + problem_name + ".problem.pddl", std::ofstream::out | std::ofstream::trunc);

        problem << "(define (problem " + problem_name + "-prob)" << std::endl;
        problem << "(:domain pursuer-evader)" << std::endl;

        problem << "(:objects" << std::endl;
        problem << define_waypoints() << std::endl;
        problem << define_gates() << std::endl;
        problem << ")" << std::endl;

        problem << "(:init" << std::endl;
        problem << "(= (total-cost) 0)" << std::endl;
        problem << initialize_robots_locations() << std::endl;
        problem << initialize_locations_relations_distances() << std::endl;
        initialize_objects(problem);
        problem << ")" << std::endl;

        problem << "(:goal" << std::endl;
        define_goal(problem);
        problem << ")" << std::endl;

        // Minimize cost
        problem << "(:metric minimize(total-cost))" << std::endl;

        problem << ")" << std::endl;
        problem.close();
    }

    bool generate_plan()
    {
        std::string command = "cd /home/ubuntu/workspace/project/src/pddl/ \n ./ff -o domain.pddl -f " + problem_name + ".problem.pddl -s 3 -w 1";
        std::string solution = exec(command.c_str());

        if (solution.find("weighted A* search space empty! problem proven unsolvable.") != std::string::npos)
        {
            std::cout << "FAIL TO FIND A PLAN" << std::endl;
            return false;
        }

        std::cout << solution << std::endl;

        std::vector<std::string> lines = split_string(solution, "\n");

        for (auto &line : lines)
        {
            std::vector<std::string> words = split_string(line, " ");
            std::vector<std::string> nice_words;
            for (auto &word : words)
            {
                if (word != "")
                {
                    nice_words.push_back(word);
                }
            }
            data.push_back(nice_words);
        }

        return true;
    }

    std::vector<Point> extract_path_from_plan()
    {
        std::vector<int> path_indexes = extract_path_indexes_from_plan();

        for (int i = 0; i < path_indexes.size(); i++)
        {
            path.push_back(graph_map.point_from_index(path_indexes[i]));
        }

        return path;
    }

    std::vector<int> extract_path_indexes_from_plan()
    {
        // plan with some actions
        /*
            ff: found legal plan as follows
            step    0: MOVE R2 L3 L2
                    1: MOVE R2 L2 L5
                    2: MOVE R2 L5 L4
                    3: MOVE R2 L4 L0
            plan cost: 28.000000
        */

        // empty plan
        /*
            ff: found legal plan as follows
            step
            plan cost: 0.000000
        */

        // fail to find a plan
        /*
            weighted A* search space empty! problem proven unsolvable.
        */

        std::cout << "*************" << problem_name << "*************" << std::endl;
        std::vector<int> path_indexes;

        cost = 0;

        for (int i = 0; i < data.size(); i++)
        {
            if (data[i].size() > 1)
            {
                // step    0: MOVE R2 L3 L2
                //         1: MOVE R2 L2 L5
                if (data[i][0] == "step")
                {
                    data[i].erase(data[i].begin());
                }

                if (data[i][1] == "MOVE")
                {
                    size_t robot = std::stoi(data[i][2].substr(1));
                    size_t start = std::stoi(data[i][3].substr(1));
                    size_t finish = std::stoi(data[i][4].substr(1));

                    if (path_indexes.size() == 0)
                    {
                        path_indexes.push_back(start);
                    }
                    path_indexes.push_back(finish);

                    std::cout << "MOVE " << robot << " " << start << " " << finish << std::endl;
                }

                // plan cost: 22.000000
                else if (data[i][1] == "cost:")
                {
                    cost = std::stoi(data[i][2]);

                    std::cout << "COST " << cost << std::endl;
                }
            }
        }

        if (cost == 0)
        {
            std::cout << "FOUND AN EMPTY PLAN" << std::endl;
            path_indexes.push_back(get_robot_location());
        }

        return path_indexes;
    }

    void show_plan(cv::Mat &img)
    {
        cv::Scalar color;
        if (problem_name == "evader")
        {
            color = cv::Scalar(124, 226, 255);
        }
        else if (problem_name == "pursuer")
        {
            color = cv::Scalar(255, 138, 100);
        }

        for (int i = 1; i < path.size(); i++)
        {
            int x1 = int(path[i - 1].x * 500) + 50;
            int y1 = img.size().height - int(path[i - 1].y * 500) - 50;
            int x2 = int(path[i].x * 500) + 50;
            int y2 = img.size().height - int(path[i].y * 500) - 50;

            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        }
    }

    int get_cost()
    {
        return cost;
    }

protected:
    // object definitions

    std::string define_waypoints()
    {
        std::string data;

        std::vector<int> locations = graph_map.get_waypoint_indexes();
        std::vector<int> gates_indexes = graph_map.get_gates_indexes();

        int pursuer = graph_map.get_pursuer_index();
        int evader = graph_map.get_evader_index();

        for (int i = 0; i < locations.size(); i++)
        {
            data += "l" + std::to_string(locations[i]) + " ";
        }
        data += "l" + std::to_string(pursuer) + " l" + std::to_string(evader) + " - waypoint";

        return data;
    }

    std::string define_gates()
    {
        std::string data;

        std::vector<int> gates_indexes = graph_map.get_gates_indexes();

        for (int i = 0; i < gates_indexes.size(); i++)
        {
            data += "l" + std::to_string(gates_indexes[i]) + " ";
        }

        data += "- gate";

        return data;
    }

    // object initializations

    virtual void initialize_objects(std::ofstream &problem) = 0;

    std::string initialize_robots_locations()
    {
        std::string data;

        int pursuer = graph_map.get_pursuer_index();
        int evader = graph_map.get_evader_index();

        data += "(at r1 l" + std::to_string(pursuer) + ")\n";
        data += "(at r2 l" + std::to_string(evader) + ")\n";

        return data;
    }

    std::string initialize_locations_relations_distances()
    {
        std::string data;
        std::map<std::pair<int, int>, int> relations = graph_map.get_locations_distances();

        for (auto const &relation : relations)
        {
            int l1 = relation.first.first;
            int l2 = relation.first.second;
            int distance = relation.second;
            data += "(near l" + std::to_string(l1) + " l" + std::to_string(l2) + ")\n";
            data += "(= (distance l" + std::to_string(l1) + " l" + std::to_string(l2) + ") " + std::to_string(distance) + ")\n";
            data += "(= (distance l" + std::to_string(l2) + " l" + std::to_string(l1) + ") " + std::to_string(distance) + ")\n";
        }

        return data;
    }

    // goal definitions

    virtual void define_goal(std::ofstream &problem) = 0;

    int virtual get_robot_location() = 0;

    // utilities

    std::string get_random_gate()
    {
        std::string data;
        std::vector<int> gate_indexes = graph_map.get_gates_indexes();

        srand(time(NULL));
        int random = rand() % gate_indexes.size();

        data += "l" + std::to_string(gate_indexes[random]);

        return data;
    }

    std::string exec(const char *cmd)
    {
        std::array<char, 128> buffer;
        std::string result;
        std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
        if (!pipe)
        {
            throw std::runtime_error("popen() failed!");
        }
        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr)
        {
            result += buffer.data();
        }
        return result;
    }

    std::vector<std::string> split_string(const std::string &str,
                                          const std::string &delimiter)
    {
        std::vector<std::string> strings;

        std::string::size_type pos = 0;
        std::string::size_type prev = 0;

        while ((pos = str.find(delimiter, prev)) != std::string::npos)
        {
            strings.push_back(str.substr(prev, pos - prev));
            prev = pos + 1;
        }

        strings.push_back(str.substr(prev));

        return strings;
    }
};
