#pragma once

namespace hexapod
{

    struct vec2f
    {
        vec2f()
            : x(0), y(0) {}
        vec2f(double x1, double y1)
            : x(x1), y(y1) {}

        double x;
        double y;

        void rotate(double angle_deg);
        static double getDistance(vec2f first, vec2f second);
        vec2f operator+(const vec2f &sum) const;
        vec2f operator-(const vec2f &sum) const;
        vec2f operator*(double size) const;
        vec2f &operator += (const vec2f &rhs);
        vec2f &operator -= (const vec2f &rhs);
        double size() const;
        double radToDeg(double rad) const;
        double vectorAngle() const;
    };

}
