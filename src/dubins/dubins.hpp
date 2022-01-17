#pragma once

#include <iostream>

#include <limits>
#include <cmath>

#include "dpoint.hpp"
#include "dcurve.hpp"
#include "dubins_utils.hpp"

using namespace std;

/**
 * @brief Static class to do Dubins calculations
 *
 */
class Dubins
{
public:
    static int k_signs[6][3]; // matrix with six solutions signs

    /**
     * @brief Store a Dubins solution
     *
     */
    struct Solution
    {
        int pidx;      // solution type
        DubinsCurve c; // solution curve

        /**
         * @brief Construct a new Dubins Solution object
         *
         * @param _pidx The solution type
         * @param _c A Dubins Curve
         */
        Solution(int _pidx, DubinsCurve _c) : pidx(_pidx), c(_c) {}
    };

    /**
     * @brief Standard form of the Dubins problem
     *
     */
    struct StandardForm
    {
        double th0;    // initial angle
        double thf;    // final angle
        double k_max;  // maximum curvature
        double lambda; // distance between the initial and final points

        /**
         * @brief Construct a new Dubins Standard Form object
         *
         * @param _th0 The initial angle
         * @param _thf The final angle
         * @param k_max The maximum curvature
         * @param _lambda Distance between the initial and final points
         */
        StandardForm(double _th0, double _thf, double k_max, double _lambda) : th0(_th0), thf(_thf), k_max(k_max), lambda(_lambda) {}
    };

    /**
     * @brief Non-standard form of the Dubins problem
     *
     */
    struct NotStandardForm
    {
        double s1, s2, s3;

        /**
         * @brief Construct a new Dubins Not Standard Form object
         *
         * @param _s1
         * @param _s2
         * @param _s3
         */
        NotStandardForm(double _s1, double _s2, double _s3) : s1(_s1), s2(_s2), s3(_s3) {}
    };

    /**
     * @brief Pieces of a Dubins solution with a boolean check
     *
     */
    struct Ok_Pieces
    {
        bool check;          // check if the solution is ok
        NotStandardForm nsf; // non-standard form of the Dubins solution

        /**
         * @brief Construct a new Ok_Pieces object
         *
         * @param _check Solution is ok
         * @param _nsf Non-standard form of the Dubins solution
         */
        Ok_Pieces(bool _check, NotStandardForm _nsf) : check(_check), nsf(_nsf) {}
    };

    static Ok_Pieces (*primitives[6])(StandardForm &sf); // array of six solution functions

    /**
     * @brief Compute the shortest Dubins path between two points
     *
     * @param pi Start point
     * @param pf End point
     * @param k_max Maximum curvature
     * @return A Dubins solution
     */
    static Solution solve(DubinsPoint &pi, DubinsPoint &pf, double k_max)
    {
        StandardForm input_problem = scaleToStandard(pi, pf, k_max);

        int pidx = -1;                                           // initialize solution type to a null solution
        double best_lenght = std::numeric_limits<double>::max(); // initialize best_lenght to infinite
        StandardForm best_solution = StandardForm(0, 0, 0, 0);   // use this to store the best solution

        // try al six possibile Dubins solutions and keep the one with the shortes lenght
        for (int i = 0; i < 6; i++)
        {
            // current solution
            Ok_Pieces solution = (primitives[i])(input_problem);

            // the lenght of the solution is the sum of the lenght of the three curves
            double lenght = solution.nsf.s1 + solution.nsf.s2 + solution.nsf.s3;

            // if the solution is ok and the lenght is shorter than the previous one then update the best solution
            if (solution.check && (lenght < best_lenght))
            {
                best_lenght = lenght;
                best_solution.th0 = solution.nsf.s1;
                best_solution.thf = solution.nsf.s2;
                best_solution.k_max = solution.nsf.s3;
                pidx = i;
            }
        }

        // if a valid solution was found
        if (pidx >= 0)
        {
            best_solution.lambda = input_problem.lambda;

            // convert the best solution to the non-standard form
            NotStandardForm nsf = scaleFromStandard(best_solution);

            // calculate the curvature of the arcs of the solution
            double k1 = k_signs[pidx][0] * k_max;
            double k2 = k_signs[pidx][1] * k_max;
            double k3 = k_signs[pidx][2] * k_max;

            // create the Dubins solution Curve
            DubinsCurve curve = DubinsCurve(pi, nsf.s1, nsf.s2, nsf.s3, k1, k2, k3);

            // if no valid solution was found return a null solution
            if (!DubinsUtils::check(
                    best_solution.th0,
                    k_signs[pidx][0] * input_problem.k_max,
                    best_solution.thf,
                    k_signs[pidx][1] * input_problem.k_max,
                    best_solution.k_max,
                    k_signs[pidx][2] * input_problem.k_max,
                    input_problem.th0,
                    input_problem.thf))
            {
                cout << "ERROR, dubins.hpp line 106, probably a low precision solution" << endl;
                return Solution(-1, DubinsCurve(DubinsPoint(0, 0, 0), 0, 0, 0, 0, 0, 0));
            }

            // return the solution
            return Solution(pidx, curve);
        }

        // otherwise return a null solution
        return Solution(pidx, DubinsCurve(DubinsPoint(0, 0, 0), 0, 0, 0, 0, 0, 0));
    }

    /**
     * @brief Convert a non-standard form of the Dubins problem to a standard form
     *
     * @param pi Starting point
     * @param pf Ending point
     * @param k_max Maximum curvature
     * @return The standard form of the Dubins problem
     */
    static StandardForm scaleToStandard(DubinsPoint &pi, DubinsPoint &pf, double k_max)
    {
        double dx = pf.x - pi.x;
        double dy = pf.y - pi.y;
        double phi = atan2(dy, dx);
        double lambda = hypot(dx, dy) / 2;

        // scale and normalize angles and curvature
        double th0 = DubinsUtils::mod2pi(pi.t - phi);
        double thf = DubinsUtils::mod2pi(pf.t - phi);
        double kp_max = k_max * lambda;

        return StandardForm(th0, thf, kp_max, lambda);
    }

    /**
     * @brief Convert a standard form of the Dubins problem to a non-standard form
     *
     * @param sf Standard form of the Dubins problem
     * @return The non-standard form of the Dubins problem
     */
    static NotStandardForm scaleFromStandard(StandardForm &sf)
    {
        double s1 = sf.th0 * sf.lambda;
        double s2 = sf.thf * sf.lambda;
        double s3 = sf.k_max * sf.lambda;

        return NotStandardForm(s1, s2, s3);
    }

    /**
     * @brief Find the left - straight - left path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces LSL(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.thf) - cos(sf.th0);
        double S = 2 * sf.k_max + sin(sf.th0) - sin(sf.thf);
        double atan = atan2(C, S);

        double delta = 2 + 4 * pow(sf.k_max, 2) - 2 * cos(sf.th0 - sf.thf) + 4 * sf.k_max * (sin(sf.th0) - sin(sf.thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s1 = invK * DubinsUtils::mod2pi(atan - sf.th0);
        double s2 = invK * sqrt(delta);
        double s3 = invK * DubinsUtils::mod2pi(sf.thf - atan);

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }

    /**
     * @brief Find the right - straight - right path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces RSR(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.th0) - cos(sf.thf);
        double S = 2 * sf.k_max - sin(sf.th0) + sin(sf.thf);
        double atan = atan2(C, S);

        double delta = 2 + 4 * pow(sf.k_max, 2) - 2 * cos(sf.th0 - sf.thf) - 4 * sf.k_max * (sin(sf.th0) - sin(sf.thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s1 = invK * DubinsUtils::mod2pi(sf.th0 - atan);
        double s2 = invK * sqrt(delta);
        double s3 = invK * DubinsUtils::mod2pi(atan - sf.thf);

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }

    /**
     * @brief Find the left - straight - right path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces LSR(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.th0) + cos(sf.thf);
        double S = 2 * sf.k_max + sin(sf.th0) + sin(sf.thf);
        double atan = atan2(-C, S);

        double delta = -2 + 4 * pow(sf.k_max, 2) + 2 * cos(sf.th0 - sf.thf) + 4 * sf.k_max * (sin(sf.th0) + sin(sf.thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s2 = invK * sqrt(delta);
        double s1 = invK * DubinsUtils::mod2pi(atan - atan2(-2, sf.k_max * s2) - sf.th0);
        double s3 = invK * DubinsUtils::mod2pi(atan - atan2(-2, sf.k_max * s2) - sf.thf);

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }

    /**
     * @brief Find the right - straight - left path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces RSL(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.th0) + cos(sf.thf);
        double S = 2 * sf.k_max - sin(sf.th0) - sin(sf.thf);
        double atan = atan2(C, S);

        double delta = -2 + 4 * pow(sf.k_max, 2) + 2 * cos(sf.th0 - sf.thf) - 4 * sf.k_max * (sin(sf.th0) + sin(sf.thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s2 = invK * sqrt(delta);
        double s1 = invK * DubinsUtils::mod2pi(sf.th0 - atan + atan2(2, sf.k_max * s2));
        double s3 = invK * DubinsUtils::mod2pi(sf.thf - atan + atan2(2, sf.k_max * s2));

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }

    /**
     * @brief Find the right - left - right path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces RLR(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.th0) - cos(sf.thf);
        double S = 2 * sf.k_max - sin(sf.th0) + sin(sf.thf);
        double atan = atan2(C, S);

        double delta = 0.125 * (6 - 4 * pow(sf.k_max, 2) + 2 * cos(sf.th0 - sf.thf) + 4 * sf.k_max * (sin(sf.th0) - sin(sf.thf)));
        if (abs(delta) > 1)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s2 = invK * DubinsUtils::mod2pi(2 * M_PI - acos(delta));
        double s1 = invK * DubinsUtils::mod2pi(sf.th0 - atan + 0.5 * sf.k_max * s2);
        double s3 = invK * DubinsUtils::mod2pi(sf.th0 - sf.thf + sf.k_max * (s2 - s1));

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }

    /**
     * @brief Find the left - right - left path
     *
     * @param sf Standard form of the Dubins problem
     * @return Non-standard form of the Dubins problem
     */
    static Ok_Pieces LRL(StandardForm &sf)
    {
        double invK = 1 / sf.k_max;
        double C = cos(sf.thf) - cos(sf.th0);
        double S = 2 * sf.k_max + sin(sf.th0) - sin(sf.thf);
        double atan = atan2(C, S);

        double delta = 0.125 * (6 - 4 * pow(sf.k_max, 2) + 2 * cos(sf.th0 - sf.thf) - 4 * sf.k_max * (sin(sf.th0) - sin(sf.thf)));
        if (abs(delta) > 1)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        double s2 = invK * DubinsUtils::mod2pi(2 * M_PI - acos(delta));
        double s1 = invK * DubinsUtils::mod2pi(atan - sf.th0 + 0.5 * sf.k_max * s2);
        double s3 = invK * DubinsUtils::mod2pi(sf.thf - sf.th0 + sf.k_max * (s2 - s1));

        return Ok_Pieces(true, NotStandardForm(s1, s2, s3));
    }
};

Dubins::Ok_Pieces (*Dubins::primitives[6])(Dubins::StandardForm &sf) = {
    Dubins::LSL,
    Dubins::RSR,
    Dubins::LSR,
    Dubins::RSL,
    Dubins::RLR,
    Dubins::LRL,
};

int Dubins::k_signs[6][3] = {
    {1, 0, 1},   // LSL
    {-1, 0, -1}, // RSR
    {1, 0, -1},  // LSR
    {-1, 0, 1},  // RSL
    {-1, 1, -1}, // RLR
    {1, -1, 1},  // LRL
};
