#pragma once

#include "../../../simulator/src/9_project_interface/include/utils.hpp"
#include "dpoint.hpp"
#include "dubins_utils.hpp"

#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief Arc of a Dubins curve
 *
 */
class DubinsArc
{
public:
    DubinsPoint i; // starting point
    DubinsPoint f; // ending point
    double k;      // curvature
    double L;      // lenght

    /**
     * @brief Construct a new Dubins Arc object
     *
     */
    DubinsArc()
    {
        i = DubinsPoint();
        f = DubinsPoint();
        k = 0;
        L = 0;
    }

    /**
     * @brief IntersectionPoints, struct that will contain if a collision has occurred and the intersection points
     *
     */
    struct IntersectionPoints
    {
        bool intersect;
        std::vector<Point> points;
        IntersectionPoints(bool _intersect, std::vector<Point> _points) : intersect(_intersect), points(_points) {}
    };

    /**
     * @brief Construct a new Dubins Arc object
     *
     * @param _i The starting point
     * @param _k The curvature
     * @param _L The lenght
     */
    DubinsArc(DubinsPoint &_i, double _k, double _L) : i(_i), k(_k), L(_L)
    {
        // compute ending point given starting point, curvature and lenght
        f = circLine(_L, _i, _k);
    }

    /**
     * @brief Compute the point on the circle given the starting point, curvature and lenght
     *
     * @param s The lenght of the arc
     * @param p The starting point
     * @param k The curvature
     * @return The ending point of the arc
     */
    DubinsPoint circLine(double s, DubinsPoint &p, double k)
    {
        double x = p.x + s * DubinsUtils::sinc(k * s / 2.0) * cos(p.t + k * s / 2);
        double y = p.y + s * DubinsUtils::sinc(k * s / 2.0) * sin(p.t + k * s / 2);
        double theta = DubinsUtils::mod2pi(p.t + k * s);
        return DubinsPoint(x, y, theta);
    }

    /**
     * @brief Print the Dubins Arc on the image
     *
     * @param img The image to add the Dubins Arc to
     * @param color_index The index of the robot or the index of the Dubins Arc in the Dubins Curve
     */
    void show_darc(cv::Mat &img, int color_index)
    {
        // samples the arc with n_points
        int n_points = 1000;
        std::vector<cv::Point> contours;

        for (int index = 0; index < n_points; index++)
        {
            double s = L / n_points * index;
            DubinsPoint d = circLine(s, i, k);
            contours.push_back(cv::Point(int(d.x * 500), img.size().height - int(d.y * 500)));
        }

        cv::Scalar color;

        // switch (color_index)
        // {
        // case 0: // start green
        //     color = cv::Scalar(0, 255, 0);
        //     break;
        // case 1: // middle blue
        //     color = cv::Scalar(255, 0, 0);
        //     break;
        // case 2: // end red
        //     color = cv::Scalar(0, 0, 255);
        //     break;
        // }

        switch (color_index)
        {
        case 0: // pursuer
            color = cv::Scalar(255, 83, 27);
            break;
        case 1: // evader
            color = cv::Scalar(0, 150, 255);
            break;
        }

        // add every point to the image connecting it to the neighbour points
        for (int i = 1; i < n_points; i++)
        {
            cv::line(img, contours[i], contours[i - 1], color, 2);
        }
    }

    /**
     * @brief Convert the Dubins Arc to a vector of Pose
     *
     * @return A vector of Pose
     */
    std::vector<Pose> to_pose_vect(double initial_l = 0.0)
    {
        std::vector<Pose> pose_vect;

        // prepare parameter to get samples of the Dubins Arc with n points
        double delta = 0.0001;

        // get vector of pose from a Dubins Arc
        // DubinsPoint pi = i;
        for (double l = 0; l < L; l += delta)
        {
            // Calculate the new point at distance delta
            DubinsPoint pi = circLine(l, i, k);
            // Create a Pose(s, x, y, t, k) s = pathlength, x = x coordinate, y = y coordinate, theta = angle, kappa = curvature
            pose_vect.push_back(Pose(initial_l + l, pi.x, pi.y, pi.t, k));
        }

        return pose_vect;
    }

    friend std::ostream &operator<<(std::ostream &os, const DubinsArc &a);

    /**
     * @brief Check for collisions in the DubinsArc
     *
     * @param obstacles Obstacles and borders that will be used to check if collisions happen
     * @return IntersectionPoints, it will contain if a collision has occurred and the intersection points
     */
    IntersectionPoints collision_check(std::vector<Polygon> &obstacles)
    {
        // Store the intersection points to all obstacles
        IntersectionPoints solution = IntersectionPoints(false, {});

        // For every obstacle
        for (int i = 0; i < obstacles.size(); i++)
        {
            IntersectionPoints s = IntersectionPoints(false, {});

            if (k == 0) // Straight dArc
            {
                // s = intersect_line_line(obstacle_point_1, obstacle_point_2);
                s = intersect_line_obstacle(obstacles[i]);
            }
            else // Curve dArc
            {
                // s = intersect_arc_line(obstacle_point_1, obstacle_point_2);
                s = intersect_arc_obstacle(obstacles[i]);
            }

            // If an intersection is found
            if (s.intersect)
            {
                // Set that at least one intersection has occurred
                solution.intersect = true;
                // Add the intersection points to the intersections
                solution.points.insert(solution.points.end(), s.points.begin(), s.points.end());
            }
        }

        return solution;
    }

private:
    /**
     * @brief Check if a collision between an arc and an obstacle occurs
     *
     * @param polygon Polygons to check the collision of the arc
     * @return IntersectionPoints, it will contain if a collision has occurred and the intersection points
     */
    IntersectionPoints intersect_arc_obstacle(Polygon &polygon)
    {
        // Store intersection points to current obstacle
        IntersectionPoints solution = IntersectionPoints(false, {});

        // For every point of the polygon
        for (int j = 0; j < polygon.size(); j++)
        {
            // Store intersection points to current edge of the obstacle
            IntersectionPoints s = IntersectionPoints(false, {});

            // Obstacle
            Point obstacle_point_1;
            Point obstacle_point_2;

            if (j < polygon.size() - 1) // Take the j and j+1 points of the polygon
            {
                obstacle_point_1 = polygon[j];
                obstacle_point_2 = polygon[j + 1];
            }
            else // Take the j (last) and 0 (first) points of the polygon
            {
                obstacle_point_1 = polygon[j];
                obstacle_point_2 = polygon[0];
            }

            // Check collision between arc and line
            s = intersect_arc_line(obstacle_point_1, obstacle_point_2);

            // If an intersection is found
            if (s.intersect)
            {
                // Set that at least one intersection has occurred
                solution.intersect = true;
                // Add the intersection points to the intersections
                solution.points.insert(solution.points.end(), s.points.begin(), s.points.end());
            }
        }

        return solution;
    }

    /**
     * @brief Check if a collision between an line and an obstacle occurs
     *
     * @param polygon Polygons to check the collision of the line
     * @return  IntersectionPoints, it will contain if a collision has occurred and the intersection points
     */
    IntersectionPoints intersect_line_obstacle(Polygon &polygon)
    {
        IntersectionPoints solution = IntersectionPoints(false, {});

        for (int j = 0; j < polygon.size(); j++)
        {
            IntersectionPoints s = IntersectionPoints(false, {});

            // Obstacle
            Point obstacle_point_1;
            Point obstacle_point_2;

            if (j < polygon.size() - 1) // Take the j and j+1 points of the polygon
            {
                obstacle_point_1 = polygon[j];
                obstacle_point_2 = polygon[j + 1];
            }
            else // Take the j (last) and 0 (first) points of the polygon
            {
                obstacle_point_1 = polygon[j];
                obstacle_point_2 = polygon[0];
            }

            s = intersect_line_line(obstacle_point_1, obstacle_point_2);

            if (s.intersect)
            {
                solution.intersect = true;
                solution.points.insert(solution.points.end(), s.points.begin(), s.points.end());
            }
        }

        return solution;
    }

    /**
     * @brief Check if a collision between an arc and a line occurs
     *
     * @param obstacle_point_1 Point 1 of the line
     * @param obstacle_point_2 Point 2 of the line
     * @return IntersectionPoints, it will contain if a collision has occurred and the intersection points
     */
    IntersectionPoints intersect_arc_line(Point &obstacle_point_1, Point &obstacle_point_2)
    {
        // Store the points that will intersect between the line and the circle
        std::vector<Point> points = {};

        // Compute point at distance PI*r, starting from point i, with curvature k
        DubinsPoint pf = circLine(M_PI / k, i, k);

        // Compute centre of arc coordinates
        Point center = Point((pf.x + i.x) / 2.0, (pf.y + i.y) / 2.0);

        // Write the segment eq in the form p1 + t(p2-p1) => q + t s    (1)
        // Write the circle eq as (x-xc)^2 + (y-yc)^2 = r^2             (2)
        // Substutute into (2) x = p1.x + t(p2.x-p1.x) from (1), same with y

        // After some algebra get 6 polinoms listed below:
        double p1 = 2 * obstacle_point_1.x * obstacle_point_2.x;
        double p2 = 2 * obstacle_point_1.y * obstacle_point_2.y;
        double p3 = 2 * center.x * obstacle_point_1.x;
        double p4 = 2 * center.x * obstacle_point_2.x;
        double p5 = 2 * center.y * obstacle_point_1.y;
        double p6 = 2 * center.y * obstacle_point_2.y;

        // We have an eq in the form c1 t^2 + c2 t + c3 = 0 (3)
        double c1 = pow(obstacle_point_1.x, 2) + pow(obstacle_point_2.x, 2) - p1 + pow(obstacle_point_1.y, 2) + pow(obstacle_point_2.y, 2) - p2;
        double c2 = -2 * pow(obstacle_point_2.x, 2) + p1 - p3 + p4 - 2 * pow(obstacle_point_2.y, 2) + p2 - p5 + p6;
        double c3 = pow(obstacle_point_2.x, 2) - p4 + pow(center.x, 2) + pow(obstacle_point_2.y, 2) - p6 + pow(center.y, 2) - pow(1 / k, 2);

        // Compute delta of equation (3)
        double delta = pow(c2, 2) - 4 * c1 * c3;

        double t1 = 0;
        double t2 = 0;

        if (delta < 0) // Delta = 0, no intersections
        {
            return IntersectionPoints(false, points);
        }
        else
        {
            if (delta > 0) // Delta > 0, 2 intersections
            {
                double deltaSq = sqrt(delta);
                t1 = (-c2 + deltaSq) / (2 * c1);
                t2 = (-c2 - deltaSq) / (2 * c1);
            }
            else // Delta = 0, 1 intersections
            {
                t1 = -c2 / (2 * c1);
                t2 = t1;
            }
        }

        double x = -1;
        double y = -1;

        // Check if intersection is between the segment boundaries
        if (t1 >= 0 && t1 <= 1)
        {
            x = obstacle_point_1.x * t1 + obstacle_point_2.x * (1 - t1);
            y = obstacle_point_1.y * t1 + obstacle_point_2.y * (1 - t1);
            points.push_back(Point(x, y));
        }

        // Check if intersection is between the segment boundaries
        if (t2 >= 0 && t2 <= 1 && t2 != t1)
        {
            x = obstacle_point_1.x * t2 + obstacle_point_2.x * (1 - t2);
            y = obstacle_point_1.y * t2 + obstacle_point_2.y * (1 - t2);
            points.push_back(Point(x, y));
        }

        // Store intersection points that belong to the arc
        std::vector<Point> filtered_points = {};

        if (points.size() > 0)
        {
            // Compute start and end angle of the arc given initial point, final point and centre of the arc
            double start_angle = DubinsUtils::mod2pi(atan2(i.y - center.y, i.x - center.x));
            double end_angle = DubinsUtils::mod2pi(atan2(f.y - center.y, f.x - center.x));

            // Negative k is clockwise, positive k is anticlockwise

            // Depending of how we travel through the arc we must modify start and end angles accordingly
            // This is for accounting that the arc can be in a range that cross 2PI
            if (k < 0 && end_angle > start_angle)
            {
                end_angle -= 2 * M_PI;
            }

            if (k > 0 && start_angle > end_angle)
            {
                start_angle -= 2 * M_PI;
            }

            // For all the intersection points that belong to the circle
            for (int i = 0; i < points.size(); i++)
            {
                // Compute the intesection angle
                double intersection_angle = DubinsUtils::mod2pi(atan2(points[i].y - center.y, points[i].x - center.x));

                // As before modify the angle accordingly
                if (k < 0 && intersection_angle > start_angle)
                {
                    intersection_angle -= 2 * M_PI;
                }

                if (k > 0 && intersection_angle > end_angle)
                {
                    intersection_angle -= 2 * M_PI;
                }

                // If the point is part of the arc add it to the list
                if (k < 0 && intersection_angle <= start_angle && intersection_angle >= end_angle)
                {
                    filtered_points.push_back(points[i]);
                }
                else if (k > 0 && intersection_angle >= start_angle && intersection_angle <= end_angle)
                {
                    filtered_points.push_back(points[i]);
                }
            }
        }

        return IntersectionPoints(filtered_points.size() > 0, filtered_points);
    }

    /**
     * @brief Check if a collision between a line and a line occurs
     *
     * @param obstacle_point_1 Point 1 of the line
     * @param obstacle_point_2 Point 2 of the line
     * @return IntersectionPoints, it will contain if a collision has occurred and the intersection points
     */
    IntersectionPoints intersect_line_line(Point &obstacle_point_1, Point &obstacle_point_2)
    {
        // Store intersection points
        std::vector<Point> points = {};

        // Find min and max boundaries
        double minX1 = std::min(i.x, f.x);
        double minY1 = std::min(i.y, f.y);
        double maxX1 = std::max(i.x, f.x);
        double maxY1 = std::max(i.y, f.y);

        double minX2 = std::min(obstacle_point_1.x, obstacle_point_2.x);
        double minY2 = std::min(obstacle_point_1.y, obstacle_point_2.y);
        double maxX2 = std::max(obstacle_point_1.x, obstacle_point_2.x);
        double maxY2 = std::max(obstacle_point_1.y, obstacle_point_2.y);

        // If the point is outside the boundaries there is no intersection
        if (maxX2 < minX1 || // l2 completely left of l1
            minX2 > maxX1 || // l2 completely right of l1
            maxY2 < minY1 || // l2 completely below l1
            minY2 > maxY1)   // l2 completely above l1
        {
            return IntersectionPoints(false, points);
        }

        // Write segment eq as p1 + t(p2-p1) => q + t s
        Point q = Point(i.x, i.y);
        Point s = Point(f.x - q.x, f.y - q.y);

        // Write segment eq as p3 + u(p4-p3) => p + u r
        Point p = Point(obstacle_point_1.x, obstacle_point_1.y);
        Point r = Point(obstacle_point_2.x - p.x, obstacle_point_2.y - p.y);

        // Subtract the two equations and on the right get diffPQ
        Point diffPQ = Point(q.x - p.x, q.y - p.y);

        // Compute the cross product
        double crossRS = DubinsUtils::cprod(r, s);
        double crossDiffR = DubinsUtils::cprod(diffPQ, r);
        double crossDiffS = DubinsUtils::cprod(diffPQ, s);

        Point ts = Point();

        if (crossRS == 0 && crossDiffR == 0)
        {
            double dotRR = DubinsUtils::dprod(r, r);
            double dotSR = DubinsUtils::dprod(s, r);
            double t0 = DubinsUtils::dprod(diffPQ, r) / dotRR;
            double t1 = t0 + dotSR / dotRR;

            if (dotSR < 0)
            {
                if (t0 >= 0 && t1 <= 1)
                {
                    ts = Point(std::max(t1, 0.0), std::min(t0, 1.0));
                }
            }
            else
            {
                if (t1 >= 0 && t0 <= 1)
                {
                    ts = Point(std::max(t0, 0.0), std::min(t1, 1.0));
                }
            }
            points.push_back(ts);
            return IntersectionPoints(true, points);
        }
        else
        {
            if (crossRS == 0 && crossDiffR != 0)
            {
                points.push_back(Point(0, 0));
                return IntersectionPoints(false, points);
            }
            else
            {
                double t = crossDiffS / crossRS;
                double u = crossDiffR / crossRS;

                // Check if intersection is between the segment boundaries
                if (t >= 0 && t <= 1 && u >= 0 && u <= 1)
                {
                    points.push_back(Point(p.x + t * r.x, p.y + t * r.y));
                    return IntersectionPoints(true, points);
                }
            }
        }
    }
};

/**
 * @brief Print Dubins Arc in console
 *
 * @param os
 * @param a
 * @return std::ostream&
 */
std::ostream &operator<<(std::ostream &os, const DubinsArc &a)
{
    return os << setprecision(2) << "[" << a.i << " " << a.f << " k: " << a.k << " L: " << a.L << "]";
}
