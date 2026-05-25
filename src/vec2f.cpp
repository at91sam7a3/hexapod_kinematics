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
    double dx = first.x - second.x;
    double dy = first.y - second.y;
    return std::sqrt(dx * dx + dy * dy);
}

void vec2f::rotate(double angle_deg)
{
    double tmpAngle_rad = angle_deg * PI / 180.0;
    double tmpx = (cos(tmpAngle_rad) * x) - (sin(tmpAngle_rad) * y);
    double tmpy = (sin(tmpAngle_rad) * x) + (cos(tmpAngle_rad) * y);
    x = tmpx;
    y = tmpy;
}

vec2f vec2f::operator+(const vec2f &sum) const
{
    return vec2f(x + sum.x, y + sum.y);
}
vec2f vec2f::operator-(const vec2f &sum) const
{
    return vec2f(x - sum.x, y - sum.y);
}
vec2f vec2f::operator*(double size) const
{
    return vec2f(x * size, y * size);
}
double vec2f::size() const
{
    return std::sqrt(x * x + y * y);
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

double vec2f::radToDeg(double rad) const
{
    return rad * (180.0 / PI);
}

double vec2f::vectorAngle() const
{
    return std::atan2(y, x) * 180.0 / PI;
}

}
