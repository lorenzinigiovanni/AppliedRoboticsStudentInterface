#pragma once

#include <iostream>

class Settings
{
public:
    // Flag for printing image
    static const bool debug_img = true;
    static const bool deep_debug_img = false;

    // Lenght of the path that the robot will do in a step (-1 for infinite)
    static const int path_length = 1;

    // Set the behavioural complexity of the robots (1, 2, 3)
    static const int behavioural_complexity = 3;

    // Offset that is applied to polygon (footprint_length/2 + axel_dx = 190/2 + 50) --> 1_lego_robot/urdf/my_robot_2.xacro
    static const int offset = 95;

    // Set neighbouring distance to connect close points in the graph
    static constexpr float distance = 0.1;

    // Maximum curvature, critical maximum is 22
    static constexpr double k_max = 20;

    // Absolute path to the workspace folder
    static std::string workspace_path;
};

std::string Settings::workspace_path = "/home/ubuntu/workspace/";
