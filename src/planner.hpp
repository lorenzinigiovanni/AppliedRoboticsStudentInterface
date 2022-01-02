#include "graph_map.hpp"

#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdio>
#include <memory>
#include <string>
#include <array>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv/cv.hpp>

class Planner
{
    std::string problem_name;
    std::vector<std::vector<std::string>> data;
    std::vector<Point> path;
    GraphMap graph_map;

public:
    Planner(std::string _problem_name, GraphMap &_graph_map) : problem_name(_problem_name), graph_map(_graph_map) {}

    void write_problem(std::vector<int> escaper_index_path = std::vector<int>())
    {
        std::ofstream problem;
        problem.open("/home/ubuntu/workspace/project/src/pddl/" + problem_name + ".problem.pddl", std::ofstream::out | std::ofstream::trunc);

        problem << "(define (problem " + problem_name + "-prob)" << std::endl;
        problem << "(:domain pursuer-escaper)" << std::endl;

        // Define objects
        problem << "(:objects" << std::endl;
        problem << graph_map.get_locations() << std::endl;
        problem << graph_map.get_gate_locations() << std::endl;
        problem << ")" << std::endl;

        // Init robot position, relations between locations and distances
        problem << "(:init" << std::endl;
        problem << graph_map.get_robots_locations() << std::endl;
        problem << graph_map.get_locations_relations() << std::endl;
        problem << "(= (total-cost) 0)" << std::endl;

        if (problem_name == "escaper")
        {
            problem << "(escaping)" << std::endl;
        }
        else if (problem_name == "pursuer")
        {
            problem << "(pursuing)" << std::endl;
            problem << std::endl;

            int escaper_distance = 0;
            for (int i = 0; i < escaper_index_path.size(); i++)
            {
                if (i == 0)
                {
                    problem << "(= (escaper-cost l" << escaper_index_path[i] << ") 100000)" << std::endl;
                }
                else
                {
                    escaper_distance += (int)(1000 * graph_map.distance_btw_points(escaper_index_path[i], escaper_index_path[i - 1]));
                    problem << "(= (escaper-cost l" << escaper_index_path[i] << ") " << escaper_distance << ")" << std::endl;
                }
            }

            problem << graph_map.get_missing_escaper_cost_locations(escaper_index_path);
        }

        problem << ")" << std::endl;

        // Declare goal
        problem << "(:goal" << std::endl;
        if (problem_name == "escaper")
        {
            problem << "(escaped r2)" << std::endl;
        }
        else if (problem_name == "pursuer")
        {
            problem << "(caught r1)" << std::endl;
        }
        problem << ")" << std::endl;

        // Minimize cost
        problem << "(:metric minimize(total-cost))" << std::endl;

        problem << ")" << std::endl;
        problem.close();
    }

    bool generate_plan()
    {
        std::string command = "cd /home/ubuntu/workspace/project/src/pddl/ \n ./ff -o domain.pddl -f " + problem_name + ".problem.pddl -s 5 -w 1";
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
            color = cv::Scalar(0, 90, 170);
        }
        else if (problem_name == "pursuer")
        {
            color = cv::Scalar(255, 0, 230);
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
};
