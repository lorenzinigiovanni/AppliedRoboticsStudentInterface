#pragma once

#include <iostream>

#include <limits>
#include <cmath>
#include <cassert>

#include "dpoint.hpp"
#include "dcurve.hpp"
#include "dubins_utils.hpp"

using namespace std;

class Dubins
{
public:
    static int kSigns[6][3];

    struct Solution
    {
        // tipo di soluzione, curva trovata
        int pidx;
        dCurve c;
        Solution(int _pidx, dCurve _c) : pidx(_pidx), c(_c) {} // probabilmente ce va un puntatore, cosi me sa che è per copia
    };

    struct StandardForm
    {
        double sc_th0;
        double sc_thf;
        double sc_Kmax;
        double lambda;
        StandardForm(double _sc_th0, double _sc_thf, double _sc_Kmax, double _lambda) : sc_th0(_sc_th0), sc_thf(_sc_thf), sc_Kmax(_sc_Kmax), lambda(_lambda) {}
    };

    struct NotStandardForm
    {
        double s1, s2, s3;
        NotStandardForm(double _s1, double _s2, double _s3) : s1(_s1), s2(_s2), s3(_s3) {}
    };

    struct Ok_Pieces
    {
        bool check;
        NotStandardForm nsf;
        Ok_Pieces(bool _check, NotStandardForm _nsf) : check(_check), nsf(_nsf) {}
    };

    static Ok_Pieces (*primitives[6])(StandardForm &sf);

    static Solution solve(dPoint &p1, dPoint &p2, double Kmax)
    {
        {
            StandardForm sf = scaleToStandard(p1, p2, Kmax);

            int pidx = -1;
            double L = std::numeric_limits<double>::max(); // infinito
            StandardForm tmp_sf = StandardForm(0, 0, 0, 0);

            for (int i = 0; i <= 5; i++)
            {
                Ok_Pieces op = (primitives[i])(sf);
                double Lcur = op.nsf.s1 + op.nsf.s2 + op.nsf.s3;

                if (op.check && (Lcur < L))
                {
                    L = Lcur;
                    tmp_sf.sc_th0 = op.nsf.s1;
                    tmp_sf.sc_thf = op.nsf.s2;
                    tmp_sf.sc_Kmax = op.nsf.s3;
                    pidx = i;
                }
            }

            if (pidx >= 0)
            {
                tmp_sf.lambda = sf.lambda;
                NotStandardForm nsf = scaleFromStandard(tmp_sf);

                double k1 = kSigns[pidx][0] * Kmax;
                double k2 = kSigns[pidx][1] * Kmax;
                double k3 = kSigns[pidx][2] * Kmax;
                dCurve curve = dCurve(&p1, nsf.s1, nsf.s2, nsf.s3, k1, k2, k3);

                // assert(DubinsUtils::check(
                //     tmp_sf.sc_th0,
                //     Dubins::kSigns[pidx][0] * sf.sc_Kmax,
                //     tmp_sf.sc_thf,
                //     Dubins::kSigns[pidx][1] * sf.sc_Kmax,
                //     tmp_sf.sc_Kmax,
                //     Dubins::kSigns[pidx][2] * sf.sc_Kmax,
                //     sf.sc_th0,
                //     sf.sc_thf));

                if (!DubinsUtils::check(
                        tmp_sf.sc_th0,
                        kSigns[pidx][0] * sf.sc_Kmax,
                        tmp_sf.sc_thf,
                        kSigns[pidx][1] * sf.sc_Kmax,
                        tmp_sf.sc_Kmax,
                        kSigns[pidx][2] * sf.sc_Kmax,
                        sf.sc_th0,
                        sf.sc_thf))
                {
                    cout << "ERROR, dubins.hpp line 106, probably a low precision solution" << endl;
                    return Solution(-1, dCurve(new dPoint(0, 0, 0), 0, 0, 0, 0, 0, 0));
                }

                return Solution(pidx, curve);
            }

            return Solution(pidx, dCurve(new dPoint(0, 0, 0), 0, 0, 0, 0, 0, 0));
        }
    }

    static StandardForm scaleToStandard(dPoint &pi, dPoint &pf, double Kmax)
    {
        {
            double dx = pf.x - pi.x;
            double dy = pf.y - pi.y;
            double phi = atan2(dy, dx);
            double lambda = hypot(dx, dy) / 2;

            // scale and normalize angles and curvature
            double sc_th0 = DubinsUtils::mod2pi(pi.t - phi);
            double sc_thf = DubinsUtils::mod2pi(pf.t - phi);
            double sc_Kmax = Kmax * lambda;

            return StandardForm(sc_th0, sc_thf, sc_Kmax, lambda);
        }
    }

    static NotStandardForm scaleFromStandard(StandardForm &sf)
    {
        {
            double s1 = sf.sc_th0 * sf.lambda;
            double s2 = sf.sc_thf * sf.lambda;
            double s3 = sf.sc_Kmax * sf.lambda;
            return NotStandardForm(s1, s2, s3);
        }
    }

    static Ok_Pieces LSL(StandardForm &sf)
    {
        {
            double sc_s1;
            double sc_s2;
            double sc_s3;

            double invK = 1 / sf.sc_Kmax;
            double C = cos(sf.sc_thf) - cos(sf.sc_th0);
            double S = 2 * sf.sc_Kmax + sin(sf.sc_th0) - sin(sf.sc_thf);
            double temp1 = atan2(C, S);

            sc_s1 = invK * DubinsUtils::mod2pi(temp1 - sf.sc_th0);
            double delta = 2 + 4 * pow(sf.sc_Kmax, 2) - 2 * cos(sf.sc_th0 - sf.sc_thf) + 4 * sf.sc_Kmax * (sin(sf.sc_th0) - sin(sf.sc_thf));

            if (delta < 0)
            {
                return Ok_Pieces(false, NotStandardForm(0, 0, 0));
            }

            sc_s2 = invK * sqrt(delta);
            sc_s3 = invK * DubinsUtils::mod2pi(sf.sc_thf - temp1);

            return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
        }
    }

    static Ok_Pieces RSR(StandardForm &sf)
    {
        double sc_s1;
        double sc_s2;
        double sc_s3;

        double invK = 1 / sf.sc_Kmax;
        double C = cos(sf.sc_th0) - cos(sf.sc_thf);
        double S = 2 * sf.sc_Kmax - sin(sf.sc_th0) + sin(sf.sc_thf);
        double temp1 = atan2(C, S);
        sc_s1 = invK * DubinsUtils::mod2pi(sf.sc_th0 - temp1);
        double delta = 2 + 4 * pow(sf.sc_Kmax, 2) - 2 * cos(sf.sc_th0 - sf.sc_thf) - 4 * sf.sc_Kmax * (sin(sf.sc_th0) - sin(sf.sc_thf));

        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        sc_s2 = invK * sqrt(delta);
        sc_s3 = invK * DubinsUtils::mod2pi(temp1 - sf.sc_thf);

        return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
    }

    static Ok_Pieces LSR(StandardForm &sf)
    {
        double sc_s1;
        double sc_s2;
        double sc_s3;

        double invK = 1 / sf.sc_Kmax;
        double C = cos(sf.sc_th0) + cos(sf.sc_thf);
        double S = 2 * sf.sc_Kmax + sin(sf.sc_th0) + sin(sf.sc_thf);
        double delta = -2 + 4 * pow(sf.sc_Kmax, 2) + 2 * cos(sf.sc_th0 - sf.sc_thf) + 4 * sf.sc_Kmax * (sin(sf.sc_th0) + sin(sf.sc_thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        sc_s2 = invK * sqrt(delta);
        sc_s1 = invK * DubinsUtils::mod2pi(atan2(-C, S) - atan2(-2, sf.sc_Kmax * sc_s2) - sf.sc_th0);
        sc_s3 = invK * DubinsUtils::mod2pi(atan2(-C, S) - atan2(-2, sf.sc_Kmax * sc_s2) - sf.sc_thf);

        return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
    }

    static Ok_Pieces RSL(StandardForm &sf)
    {
        double sc_s1;
        double sc_s2;
        double sc_s3;

        double invK = 1 / sf.sc_Kmax;
        double C = cos(sf.sc_th0) + cos(sf.sc_thf);
        double S = 2 * sf.sc_Kmax - sin(sf.sc_th0) - sin(sf.sc_thf);

        double delta = -2 + 4 * pow(sf.sc_Kmax, 2) + 2 * cos(sf.sc_th0 - sf.sc_thf) - 4 * sf.sc_Kmax * (sin(sf.sc_th0) + sin(sf.sc_thf));
        if (delta < 0)
        {
            return Ok_Pieces(false, NotStandardForm(0, 0, 0));
        }

        sc_s2 = invK * sqrt(delta);
        sc_s1 = invK * DubinsUtils::mod2pi(sf.sc_th0 - atan2(C, S) + atan2(2, sf.sc_Kmax * sc_s2));
        sc_s3 = invK * DubinsUtils::mod2pi(sf.sc_thf - atan2(C, S) + atan2(2, sf.sc_Kmax * sc_s2));

        return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
    }

    static Ok_Pieces RLR(StandardForm &sf)
    {
        double sc_s1;
        double sc_s2;
        double sc_s3;

        double invK = 1 / sf.sc_Kmax;
        double C = cos(sf.sc_th0) - cos(sf.sc_thf);
        double S = 2 * sf.sc_Kmax - sin(sf.sc_th0) + sin(sf.sc_thf);

        sc_s2 = invK * DubinsUtils::mod2pi(2 * M_PI - acos(0.125 * (6 - 4 * pow(sf.sc_Kmax, 2) + 2 * cos(sf.sc_th0 - sf.sc_thf) + 4 * sf.sc_Kmax * (sin(sf.sc_th0) - sin(sf.sc_thf)))));
        sc_s1 = invK * DubinsUtils::mod2pi(sf.sc_th0 - atan2(C, S) + 0.5 * sf.sc_Kmax * sc_s2);
        sc_s3 = invK * DubinsUtils::mod2pi(sf.sc_th0 - sf.sc_thf + sf.sc_Kmax * (sc_s2 - sc_s1));

        return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
    }

    static Ok_Pieces LRL(StandardForm &sf)
    {
        double sc_s1;
        double sc_s2;
        double sc_s3;

        double invK = 1 / sf.sc_Kmax;
        double C = cos(sf.sc_thf) - cos(sf.sc_th0);
        double S = 2 * sf.sc_Kmax + sin(sf.sc_th0) - sin(sf.sc_thf);

        sc_s2 = invK * DubinsUtils::mod2pi(2 * M_PI - acos(0.125 * (6 - 4 * pow(sf.sc_Kmax, 2) + 2 * cos(sf.sc_th0 - sf.sc_thf) - 4 * sf.sc_Kmax * (sin(sf.sc_th0) - sin(sf.sc_thf)))));
        sc_s1 = invK * DubinsUtils::mod2pi(-sf.sc_th0 + atan2(C, S) + 0.5 * sf.sc_Kmax * sc_s2);
        sc_s3 = invK * DubinsUtils::mod2pi(sf.sc_thf - sf.sc_th0 + sf.sc_Kmax * (sc_s2 - sc_s1));

        return Ok_Pieces(true, NotStandardForm(sc_s1, sc_s2, sc_s3));
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

int Dubins::kSigns[6][3] = {
    {1, 0, 1},   // LSL
    {-1, 0, -1}, // RSR
    {1, 0, -1},  // LSR
    {-1, 0, 1},  // RSL
    {-1, 1, -1}, // RLR
    {1, -1, 1},  // LRL
};
