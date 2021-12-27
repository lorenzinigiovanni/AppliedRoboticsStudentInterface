#pragma once

#include <iostream>

class dPoint
{
public:
    float x, y, t;

    dPoint(float _x, float _y, float _t) : x(_x), y(_y), t(_t){};
    dPoint(const Point p, float _t) : x(p.x), y(p.y), t(_t){};

    friend std::ostream &operator<<(std::ostream &os, const dPoint &_p);
};

std::ostream &operator<<(std::ostream &os, const dPoint &_p)
{
    return os << "(" << _p.x << " " << _p.y << " " << _p.t << ")";
}
