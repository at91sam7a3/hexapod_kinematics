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

        void rotate(double angle);
        static double getDistance(vec2f first, vec2f second);
        vec2f operator+(const vec2f &sum);
        vec2f operator-(const vec2f &sum);
        vec2f operator*(const double size);
        vec2f &operator += (const vec2f &rhs);
        vec2f &operator -= (const vec2f &rhs);
        double size();
        int radToDeg(float rad);
        double vectorAngle();
    };

}
