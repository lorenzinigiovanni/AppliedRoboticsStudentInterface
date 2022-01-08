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

/**
 * @brief Virtual class to be used to deal with an PDDL planner
 *
 */
class Planner
{
protected:
    std::string problem_name;                   // The name of the PDDL problem
    std::vector<std::vector<std::string>> data; // The data returned by the PDDL planner
    std::vector<Point> path;                    // The path returned by the PDDL planner converted in a vector of points
    GraphMap graph_map;                         // The graph map with the map informations to be used by the planner
    int complexity;                             // The behavioural complexity of the planner
    int cost;                                   // The total cost of the found plan

    /**
     * @brief Construct a new Planner object
     *
     * @param _problem_name The name of the PDDL problem
     * @param _graph_map The graph map with the map informations to be used by the planner
     * @param _complexity The behavioural complexity of the planner
     */
    Planner(std::string _problem_name,
            GraphMap &_graph_map,
            int _complexity)
        : problem_name(_problem_name),
          graph_map(_graph_map),
          complexity(_complexity)
    {
    }

public:
    /**
     * @brief Create the file *.problem.pddl to be used by the planner
     *
     */
    void write_problem()
    {
        std::ofstream problem;

        // Write the problem file as problemname.problem.pddl
        problem.open("/home/ubuntu/workspace/project/src/pddl/" + problem_name + ".problem.pddl", std::ofstream::out | std::ofstream::trunc);

        // Initialize the problem file by defining the problem name and the domain name
        problem << "(define (problem " + problem_name + "-prob)" << std::endl;
        problem << "(:domain pursuer-evader)" << std::endl;

        // Write in the problem file the objects to be used by the planner
        problem << "(:objects" << std::endl;
        problem << define_waypoints() << std::endl;
        problem << define_gates() << std::endl;
        problem << ")" << std::endl;

        // Write in the problem file the initial state of the problem
        problem << "(:init" << std::endl;
        problem << "(= (total-cost) 0)" << std::endl;
        problem << initialize_robots_locations() << std::endl;
        problem << initialize_locations_relations_distances() << std::endl;
        initialize_objects(problem);
        problem << ")" << std::endl;

        // Write in the problem file the goal
        problem << "(:goal" << std::endl;
        define_goal(problem);
        problem << ")" << std::endl;

        // Write in the problem file the metric to minimize cost
        problem << "(:metric minimize(total-cost))" << std::endl;

        problem << ")" << std::endl;
        problem.close();
    }

    /**
     * @brief Call the planner to obtain the plan
     *
     * @return true if the planner succeded to found a plan
     * @return false if the planner failed to found a plan
     */
    bool generate_plan()
    {
        // Command used to call the Metric-FF planner
        // -o domain file
        // -f problem file
        // -s the type of search
        // -w the weight
        std::string command = "cd /home/ubuntu/workspace/project/src/pddl/ \n ./ff -o domain.pddl -f " + problem_name + ".problem.pddl -s 3 -w 1";

        // Executo the command and read the console output of it's execution
        std::string solution = exec(command.c_str());

        // If the planner failed to found a plan, return false
        if (solution.find("weighted A* search space empty! problem proven unsolvable.") != std::string::npos)
        {
            std::cout << "FAIL TO FIND A PLAN" << std::endl;
            return false;
        }

        // Split the solution in lines
        std::vector<std::string> lines = split_string(solution, "\n");

        // For each line in the solution
        for (auto &line : lines)
        {
            // Split the line in words
            std::vector<std::string> words = split_string(line, " ");

            // If the world is an empty word, discard it, otherwise add to the data vector
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

    /**
     * @brief Convert the plan returned by the planner into a vector of points
     *
     * @return A vector of points representing the plan returned by the planner
     */
    std::vector<Point> extract_path_from_plan()
    {
        // Get the indexes of the vertexes in the plan
        std::vector<int> path_indexes = extract_path_indexes_from_plan();

        // For each vertex in the plan
        for (int i = 0; i < path_indexes.size(); i++)
        {
            // Convert the vertex index into a Point and add it to the path
            path.push_back(graph_map.point_from_index(path_indexes[i]));
        }

        return path;
    }

    /**
     * @brief Get the indexes of the vertexes in the plan returned by the planner
     *
     * @return A vector of indexes representing the vertexes of the plan returned by the planner
     */
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

        // For each line in the plan
        for (int i = 0; i < data.size(); i++)
        {
            // Check that is the line is composed of at least two words
            if (data[i].size() > 1)
            {
                // step    0: MOVE R2 L3 L2
                //         1: MOVE R2 L2 L5

                // Remove the word "step" from the line as it only occurs in the first line of the plan
                if (data[i][0] == "step")
                {
                    data[i].erase(data[i].begin());
                }

                // If it founds the world "MOVE" read the robot number and the start and finish positions
                if (data[i][1] == "MOVE")
                {
                    size_t robot = std::stoi(data[i][2].substr(1));
                    size_t start = std::stoi(data[i][3].substr(1));
                    size_t finish = std::stoi(data[i][4].substr(1));

                    // Only for the first line of the plan add to the path indexes also the starting position
                    if (path_indexes.size() == 0)
                    {
                        path_indexes.push_back(start);
                    }

                    // Always add to the path indexes the destination position
                    path_indexes.push_back(finish);

                    std::cout << "MOVE " << robot << " " << start << " " << finish << std::endl;
                }

                // plan cost: 22.000000

                // If the line contains the word "cost"
                else if (data[i][1] == "cost:")
                {
                    // Store the cost of the plan
                    cost = std::stoi(data[i][2]);

                    std::cout << "COST " << cost << std::endl;
                }
            }
        }

        // If the plan has cost 0, it means that the planner found a plan in wich the robot should not move
        if (cost == 0)
        {
            std::cout << "FOUND AN EMPTY PLAN" << std::endl;

            // Add the starting position to the path indexes
            path_indexes.push_back(get_robot_location());
        }

        return path_indexes;
    }

    /**
     * @brief Print the plan on the image
     *
     * @param img Image on which the plan will be printed
     */
    void show_plan(cv::Mat &img)
    {
        cv::Scalar color;
        if (problem_name == "evader")
        {
            // The evader plan is light yellow colored
            color = cv::Scalar(120, 200, 255);
        }
        else if (problem_name == "pursuer")
        {
            // The pursuer plan is light blue colored
            color = cv::Scalar(255, 138, 100);
        }

        // For every step in the plan
        for (int i = 1; i < path.size(); i++)
        {
            int x1 = int(path[i - 1].x * 500) + 50;
            int y1 = img.size().height - int(path[i - 1].y * 500) - 50;
            int x2 = int(path[i].x * 500) + 50;
            int y2 = img.size().height - int(path[i].y * 500) - 50;

            // Draw a line between the two points of the step in the image
            cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color, 2);
        }
    }

    /**
     * @brief Get the cost of the plan
     *
     * @return Cost of the plan
     */
    int get_cost()
    {
        return cost;
    }

protected:
    /**
     * @brief Define the waypoints objects in the PDDL problem
     *
     * @return String with the waypoints definitions
     */
    std::string define_waypoints()
    {
        std::string data;

        // Get the indexes of the waypoints
        std::vector<int> locations = graph_map.get_waypoint_indexes();

        // Get the indexes of the locations of the robots
        int pursuer = graph_map.get_pursuer_index();
        int evader = graph_map.get_evader_index();

        // Add the locations definition to the PDDL problem
        for (int i = 0; i < locations.size(); i++)
        {
            data += "l" + std::to_string(locations[i]) + " ";
        }
        data += "l" + std::to_string(pursuer) + " l" + std::to_string(evader);

        // Set the type to waypoint
        data += " - waypoint";

        return data;
    }

    /**
     * @brief Define the gates objects in the PDDL problem
     *
     * @return String with the gates definitions
     */
    std::string define_gates()
    {
        std::string data;

        // Get the indexes of the gates
        std::vector<int> gates_indexes = graph_map.get_gates_indexes();

        // Add the gates definition to the PDDL problem
        for (int i = 0; i < gates_indexes.size(); i++)
        {
            data += "l" + std::to_string(gates_indexes[i]) + " ";
        }

        // Set the type to gate
        data += "- gate";

        return data;
    }

    /**
     * @brief Initialize some objects of the PDDL problem
     *
     * @param problem The PDDL problem to which the definitions will be added
     */
    virtual void initialize_objects(std::ofstream &problem) = 0;

    /**
     * @brief Define the initial positions of the robots in the PDDL problem
     *
     * @return String with the initialization of the positions of the robots
     */
    std::string initialize_robots_locations()
    {
        std::string data;

        // Get the indexes of the locations of the robots
        int pursuer = graph_map.get_pursuer_index();
        int evader = graph_map.get_evader_index();

        // Add the initial locations to the PDDL problem
        data += "(at r1 l" + std::to_string(pursuer) + ")\n";
        data += "(at r2 l" + std::to_string(evader) + ")\n";

        return data;
    }

    /**
     * @brief Define the proximity and the distance between the locations in the map
     *
     * @return String with the proximity and the distance between the locations
     */
    std::string initialize_locations_relations_distances()
    {
        std::string data;

        // Get the relations between locations in the graph map
        std::map<std::pair<int, int>, int> relations = graph_map.get_locations_distances();

        // For each relation
        for (auto const &relation : relations)
        {
            int l1 = relation.first.first;
            int l2 = relation.first.second;

            // Tell the planner that the robot can move between the two locations
            data += "(near l" + std::to_string(l1) + " l" + std::to_string(l2) + ")\n";

            int distance = relation.second;

            // Tell the plannner the distance between the two locations
            data += "(= (distance l" + std::to_string(l1) + " l" + std::to_string(l2) + ") " + std::to_string(distance) + ")\n";
            data += "(= (distance l" + std::to_string(l2) + " l" + std::to_string(l1) + ") " + std::to_string(distance) + ")\n";
        }

        return data;
    }

    /**
     * @brief Define the goal of the PDDL problem
     *
     * @param problem The PDDL problem to which the goal will be added
     */
    virtual void define_goal(std::ofstream &problem) = 0;

    /**
     * @brief Get the vertex index of the position of the robot
     *
     * @return Index of the vertex of the position of the robot
     */
    int virtual get_robot_location() = 0;

    /**
     * @brief Get a random gate to which the robot can move
     *
     * @return The gate index preceded by an l
     */
    std::string get_random_gate()
    {
        std::string data;

        // Get all gates possible indexes
        std::vector<int> gate_indexes = graph_map.get_gates_indexes();

        // Initialize the random seed
        srand(time(NULL));

        // Get a random number between 0 and the number of gates - 1
        int random = rand() % gate_indexes.size();

        // Generate the string with l and random gate index
        data += "l" + std::to_string(gate_indexes[random]);

        return data;
    }

    /**
     * @brief Execute a command in the terminal with a seprate process
     *
     * @param cmd The command to execute
     * @return The output of the command in the terminal
     */
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

    /**
     * @brief Split a string by a delimiter
     *
     * @param str String to be splitted
     * @param delimiter The delimiter to split the string
     * @return Splitted string
     */
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
