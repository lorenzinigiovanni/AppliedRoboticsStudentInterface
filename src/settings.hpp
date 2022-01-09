#pragma once

#include <iostream>

class Settings
{
public:
    // Flag for printing image
    static const bool debug_img = true;

    // Lenght of the path that the robot will do in a step (-1 for infinite)
    static const int path_length = 1;

    // Set the behavioural complexity of the robots (1, 2, 3)
    static const int behavioural_complexity = 3;

    // Offset that is applied to polygon (95mm)
    static const int offset = 95;

    // maximum curvature, critical maximum is 22
    static constexpr double k_max = 20;

    static std::string workspace_path;
    // static const std::string workspace_path = std::string("/home/ubuntu/workspace/");
};

std::string Settings::workspace_path = "/home/ubuntu/workspace/";
