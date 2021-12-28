#pragma once

#include <cmath>

using namespace std;

class DubinsUtils
{
public:
    double static sinc(double t)
    {
        double s;
        if (abs(t) < 0.002)
        {
            s = 1 - (pow(t, 2) / 6) + (pow(t, 4) / 120);
        }
        else
        {
            s = sin(t) / t;
        }
        return s;
    }

    double static mod2pi(double ang)
    {
        double out = ang;
        while (out < 0)
        {
            out = out + 2 * M_PI;
        }
        while (out >= 2 * M_PI)
        {
            out = out - 2 * M_PI;
        }
        return out;
    }

    double static rangeSymm(double ang)
    {
        double out = ang;
        while (out <= -1 * M_PI)
        {
            out = out + 2 * M_PI;
        }
        while (out > M_PI)
        {
            out = out - 2 * M_PI;
        }

        return out;
    }

    bool static check(double s1, double k0, double s2, double k1, double s3, double k2, double th0, double thf)
    {
        double x0 = -1;
        double y0 = 0;
        double xf = 1;
        double yf = 0;

        double eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
        double eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
        double eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

        bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
    }
};
