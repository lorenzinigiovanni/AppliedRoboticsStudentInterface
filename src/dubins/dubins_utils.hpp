#pragma once

#include <cmath>

using namespace std;

class DubinsUtils
{
public:
    float static sinc(float t)
    {
        float s;
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

    float static mod2pi(float ang)
    {
        float out = ang;
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

    float static rangeSymm(float ang)
    {
        float out = ang;
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

    bool static check(float s1, float k0, float s2, float k1, float s3, float k2, float th0, float thf)
    {
        float x0 = -1;
        float y0 = 0;
        float xf = 1;
        float yf = 0;

        float eq1 = x0 + s1 * sinc((1 / 2.) * k0 * s1) * cos(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * cos(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * cos(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - xf;
        float eq2 = y0 + s1 * sinc((1 / 2.) * k0 * s1) * sin(th0 + (1 / 2.) * k0 * s1) + s2 * sinc((1 / 2.) * k1 * s2) * sin(th0 + k0 * s1 + (1 / 2.) * k1 * s2) + s3 * sinc((1 / 2.) * k2 * s3) * sin(th0 + k0 * s1 + k1 * s2 + (1 / 2.) * k2 * s3) - yf;
        float eq3 = rangeSymm(k0 * s1 + k1 * s2 + k2 * s3 + th0 - thf);

        bool Lpos = (s1 > 0) || (s2 > 0) || (s3 > 0);
        return (sqrt(eq1 * eq1 + eq2 * eq2 + eq3 * eq3) < 1.e-10) && Lpos;
    }
};
