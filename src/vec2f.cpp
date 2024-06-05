#include "vec2f.hpp"
#include <cmath>

namespace hexapod
{
namespace
{
    constexpr static const double PI = 3.141592654;
}
double vec2f::getDistance(vec2f first, vec2f second)
{
    return sqrt(first.x * second.x + first.y * second.y);
}

void vec2f::rotate(double angle)
{
    double tmpAngle = angle * PI / 180.0;
    double tmpx = (cos(tmpAngle) * x) - (sin(tmpAngle) * y);
    double tmpy = (sin(tmpAngle) * x) + (cos(tmpAngle) * y);
    x = tmpx;
    y = tmpy;
}

vec2f vec2f::operator+(const vec2f &sum)
{
    return vec2f(x + sum.x, y + sum.y);
}
vec2f vec2f::operator-(const vec2f &sum)
{
    return vec2f(x - sum.x, y - sum.y);
}
vec2f vec2f::operator*(const double size)
{
    return vec2f(x * size, y * size);
}
double vec2f::size()
{
    return (sqrt((x * x) + (y * y)));
}

vec2f &vec2f::operator += (const vec2f &rhs)
{
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

vec2f &vec2f::operator -= (const vec2f &rhs)
{
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

int vec2f::radToDeg(float rad)
{
    return rad * (180 / M_PI);
}

double vec2f::vectorAngle()
{
    if (x == 0) // special cases
        return (y > 0)    ? 90
               : (y == 0) ? 0
                          : 270;
    else if (y == 0) // special cases
        return (x >= 0) ? 0
                        : 180;
    int ret = radToDeg(atanf((float)y / x));
    if (x < 0 && y < 0) // quadrant Ⅲ
        ret = 180 + ret;
    else if (x < 0)             // quadrant Ⅱ
        ret = 180 + ret;        // it actually substracts
    else if (y < 0)             // quadrant Ⅳ
        ret = 270 + (90 + ret); // it actually substracts
    return ret;
}

}
