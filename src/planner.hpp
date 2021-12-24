#include "graph_map.hpp"

#include <iostream>
#include <fstream>

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

    void write_problem()
    {
        std::ofstream problem;
        problem.open("/home/ubuntu/workspace/project/src/pddl/" + problem_name + ".pddl", std::ofstream::out | std::ofstream::trunc);

        problem << "(define (problem pursuer-prob)" << std::endl;
        problem << "(:domain pursuer)" << std::endl;

        // Define objects
        problem << "(:objects" << std::endl;
        problem << graph_map.get_locations() << std::endl;
        problem << "r1 r2 - robot" << std::endl;
        problem << ")" << std::endl;

        // Init robot position, relations between locations
        problem << "(:init" << std::endl;
        problem << graph_map.get_robots_locations() << std::endl;
        problem << graph_map.get_locations_relations() << std::endl;
        problem << ")" << std::endl;

        // Declare goal
        problem << "(:goal" << std::endl;
        problem << "(caught r1 r2)" << std::endl;
        problem << ")" << std::endl;

        problem << ")" << std::endl;
        problem.close();
    }

    void generate_plan()
    {
        std::string cmd = "cd /home/ubuntu/workspace/project/src/pddl/ \n planutils run lama domain.pddl " + problem_name + ".pddl";
        system(cmd.c_str());

        std::ifstream solution("/home/ubuntu/workspace/project/src/pddl/sas_plan.1");

        for (std::string line; std::getline(solution, line);)
        {
            data.push_back(split_string(line, " "));
        }

        solution.close();
    }

    std::vector<Point> extract_path_from_plan()
    {
        for (int i = 0; i < data.size(); i++)
        {
            // (move r2 l4 l5)
            if (data[i][0] == "(move")
            {
                size_t robot = std::stoi(data[i][1].substr(1));
                size_t start = std::stoi(data[i][2].substr(1));
                size_t finish = std::stoi(data[i][3].substr(1).substr(0, data[i][3].find(")") - 1));

                if (path.size() == 0)
                {
                    path.push_back(graph_map.point_from_index(start));
                }
                path.push_back(graph_map.point_from_index(finish));

                std::cout << "move (" << robot << " " << start << " " << finish << ")" << std::endl;
            }
            // (reach-gatess r2 l5)
            else if (data[i][0] == "(reach-gatess")
            {
                size_t robot = std::stoi(data[i][1].substr(1));
                size_t location = std::stoi(data[i][2].substr(1).substr(0, data[i][2].find(")") - 1));

                std::cout << "reach-gatess (" << robot << " " << location << ")" << std::endl;
            }
            // (reach-evader r1 r2 l0)
            else if (data[i][0] == "(reach-evader")
            {
                size_t pursuer = std::stoi(data[i][1].substr(1));
                size_t evader = std::stoi(data[i][2].substr(1));
                size_t location = std::stoi(data[i][3].substr(1).substr(0, data[i][3].find(")") - 1));

                std::cout << "reach-evader (" << pursuer << " " << evader << " " << location << ")" << std::endl;
            }
            // ; cost = 2 (unit cost)
            else if (data[i][1] == "cost")
            {
                size_t cost = std::stoi(data[i][3]);

                std::cout << "cost (" << cost << ")" << std::endl;
            }
        }

        return path;
    }

    void show_plan(cv::Mat &img)
    {
        for (int i = 1; i < path.size(); i++)
        {
            int x1 = int(path[i - 1].x * 500) + 50;
            int y1 = img.size().height - int(path[i - 1].y * 500) - 50;
            int x2 = int(path[i].x * 500) + 50;
            int y2 = img.size().height - int(path[i].y * 500) - 50;

            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(187, 208, 40), 2);
        }
    }

private:
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
