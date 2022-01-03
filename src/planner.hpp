#include "graph_map.hpp"

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
    std::string problem_name;
    std::vector<std::vector<std::string>> data;
    std::vector<Point> path;
    std::vector<int> escaper_estimated_path;
    GraphMap graph_map;
    int complexity;

public:
    Planner(std::string _problem_name,
            GraphMap &_graph_map,
            int _complexity,
            std::vector<int> _escaper_estimated_path = std::vector<int>())
        : problem_name(_problem_name),
          graph_map(_graph_map),
          complexity(_complexity),
          escaper_estimated_path(_escaper_estimated_path)
    {
    }

    void write_problem()
    {
        std::ofstream problem;
        problem.open("/home/ubuntu/workspace/project/src/pddl/" + problem_name + ".problem.pddl", std::ofstream::out | std::ofstream::trunc);

        problem << "(define (problem " + problem_name + "-prob)" << std::endl;
        problem << "(:domain pursuer-escaper)" << std::endl;

        define_objects(problem);
        initialize_objects(problem);
        define_goal(problem);

        // Minimize cost
        problem << "(:metric minimize(total-cost))" << std::endl;

        problem << ")" << std::endl;
        problem.close();
    }

    bool generate_plan()
    {
        std::string command = "cd /home/ubuntu/workspace/project/src/pddl/ \n ./ff -o domain.pddl -f " + problem_name + ".problem.pddl -s 3 -w 1";
        std::string solution = exec(command.c_str());

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
        /*
        ff: found legal plan as follows
        step    0: MOVE R2 L3 L2
                1: MOVE R2 L2 L5
                2: MOVE R2 L5 L4
                3: MOVE R2 L4 L0
        */

        std::vector<int> path_indexes;

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
                    size_t finish = std::stoi(data[i][4].substr(1).substr(0, data[i][4].find(")") - 1));

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
                    size_t cost = std::stoi(data[i][2]);

                    std::cout << "COST " << cost << std::endl;
                }
            }
        }

        return path_indexes;
    }

    void show_plan(cv::Mat &img)
    {
        cv::Scalar color;
        if (problem_name == "escaper")
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

private:
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

    void define_objects(std::ofstream &problem)
    {
        problem << "(:objects" << std::endl;
        problem << graph_map.get_locations() << std::endl;
        problem << graph_map.get_gate_locations() << std::endl;
        problem << ")" << std::endl;
    }

    void initialize_objects(std::ofstream &problem)
    {
        problem << "(:init" << std::endl;
        problem << "(= (total-cost) 0)" << std::endl;
        problem << graph_map.get_robots_locations() << std::endl;
        problem << graph_map.get_locations_relations() << std::endl;

        if (problem_name == "escaper" || problem_name == "escaper_estimated")
        {
            initialize_objects_escaper(problem);
        }
        else if (problem_name == "pursuer")
        {
            initialize_objects_pursuer(problem);
        }

        problem << ")" << std::endl;
    }

    void initialize_objects_escaper(std::ofstream &problem)
    {
        problem << "(escaping)" << std::endl;
    }

    void initialize_objects_pursuer(std::ofstream &problem)
    {
        problem << "(pursuing)" << std::endl;
        problem << std::endl;

        int escaper_distance = 0;
        for (int i = 0; i < escaper_estimated_path.size(); i++)
        {
            if (i == 0)
            {
                problem << "(= (escaper-cost l" << escaper_estimated_path[i] << ") 100000)" << std::endl;
            }
            else
            {
                escaper_distance += (int)(1000 * graph_map.distance_btw_points(escaper_estimated_path[i], escaper_estimated_path[i - 1]));
                problem << "(= (escaper-cost l" << escaper_estimated_path[i] << ") " << escaper_distance << ")" << std::endl;
            }
        }

        problem << graph_map.get_missing_escaper_cost_locations(escaper_estimated_path);
    }

    void define_goal(std::ofstream &problem)
    {
        problem << "(:goal" << std::endl;

        if (problem_name == "escaper")
        {
            define_goal_escaper(problem);
        }
        else if (problem_name == "escaper_estimated")
        {
            define_goal_escaper_estimated(problem);
        }
        else if (problem_name == "pursuer")
        {
            define_goal_pursuer(problem);
        }
        problem << ")" << std::endl;
    }

    void define_goal_escaper(std::ofstream &problem)
    {
        switch (complexity)
        {
        case 1:
        {
            problem << "(escaped r2)" << std::endl;
            break;
        }

        case 2:
        {
            problem << "(and" << std::endl;
            problem << "(escaped r2)" << std::endl;

            std::string choosen_gate_location = "";

            std::fstream test_file("/home/ubuntu/workspace/project/state/escaper_info.txt");
            if (!test_file.good())
            {
                // decide gate toward which the escaper will head
                choosen_gate_location = graph_map.get_random_gate();

                std::ofstream file;
                file.open("/home/ubuntu/workspace/project/state/escaper_info.txt", std::ofstream::out);
                file << choosen_gate_location << std::endl;
                file.close();
            }
            else
            {
                // read from file the decided gate toward which the escaper is headed
                std::ifstream file;
                file.open("/home/ubuntu/workspace/project/state/escaper_info.txt", std::ifstream::in);

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
            break;
        }

        default:
        {
            problem << "(escaped r2)" << std::endl;
            break;
        }
        }
    }

    void define_goal_escaper_estimated(std::ofstream &problem)
    {
        switch (complexity)
        {
        case 1:
        {
            problem << "(escaped r2)" << std::endl;
            break;
        }

        case 2:
        {
            problem << "(and" << std::endl;
            problem << "(escaped r2)" << std::endl;

            std::map<int, float> distances = graph_map.get_robot_gate_distances();

            std::fstream test_file("/home/ubuntu/workspace/project/state/escaper_estimated_info.txt");
            if (!test_file.good())
            {
                std::ofstream file;
                file.open("/home/ubuntu/workspace/project/state/escaper_estimated_info.txt", std::ofstream::out);

                for (std::map<int, float>::const_iterator it = distances.begin(); it != distances.end(); it++)
                {
                    file << "l" << it->first << "\t";
                }
                file << std::endl;

                file.close();
            }

            std::fstream file;
            file.open("/home/ubuntu/workspace/project/state/escaper_estimated_info.txt", std::fstream::in | std::fstream::out | std::fstream::app);

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

        case 3:
        {
            break;
        }

        default:
        {
            problem << "(escaped r2)" << std::endl;
            break;
        }
        }
    }

    void define_goal_pursuer(std::ofstream &problem)
    {
        problem << "(caught r1)" << std::endl;
    }
};
