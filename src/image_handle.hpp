#pragma once

#include "../../simulator/src/9_project_interface/include/utils.hpp"
#include <iostream>
#include <experimental/filesystem>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Show a single polygon in the image
 *
 * @param img
 * @param polygon Polygon to be written
 */
void show_polygon(cv::Mat &img, Polygon polygon)
{
    for (int j = 0; j < polygon.size() - 1; j++)
    {
        Point p1 = polygon[j];
        Point p2 = polygon[j + 1];

        int x1 = int(p1.x * 500);
        int y1 = img.size().height - int(p1.y * 500);
        int x2 = int(p2.x * 500);
        int y2 = img.size().height - int(p2.y * 500);

        cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1);
    }

    Point p1 = polygon[polygon.size() - 1];
    Point p2 = polygon[0];

    int x1 = int(p1.x * 500);
    int y1 = img.size().height - int(p1.y * 500);
    int x2 = int(p2.x * 500);
    int y2 = img.size().height - int(p2.y * 500);

    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1);
}

/**
 * @brief Show a list of polygons in the image
 *
 * @param img 
 * @param polygons Polygons to be written
 */
void show_polygon_list(cv::Mat &img, std::vector<Polygon> polygons)
{
    for (int i = 0; i < polygons.size(); i++)
    {
        show_polygon(img, polygons[i]);
    }
}

/**
 * @brief Save the image into the specified path
 *
 * @param img Image to be saved
 * @param path Path in which the image will be saved
 */
void write_img(cv::Mat &img, std::string path)
{
    std::experimental::filesystem::path photo_path(path);

    int last_image = 0;
    for (const auto &file : std::experimental::filesystem::directory_iterator(photo_path))
    {
        last_image++;
    }
    cv::imwrite(path + "image-" + std::to_string(last_image + 1) + ".png", img);
}
