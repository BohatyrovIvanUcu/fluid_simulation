#pragma once

#include <cmath>

namespace fluid {

struct Vec2 {
    float x;
    float y;

    constexpr Vec2() : x(0.0f), y(0.0f) {}
    constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}

    constexpr Vec2 operator+(const Vec2& other) const {
        return Vec2(x + other.x, y + other.y);
    }

    constexpr Vec2 operator-(const Vec2& other) const {
        return Vec2(x - other.x, y - other.y);
    }

    constexpr Vec2 operator*(float scalar) const {
        return Vec2(x * scalar, y * scalar);
    }

    constexpr Vec2 operator/(float scalar) const {
        return Vec2(x / scalar, y / scalar);
    }

    Vec2& operator+=(const Vec2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2& operator-=(const Vec2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    constexpr float dot(const Vec2& other) const {
        return x * other.x + y * other.y;
    }

    constexpr float length_sq() const {
        return x * x + y * y;
    }

    float length() const {
        return std::sqrt(length_sq());
    }

    Vec2 normalized() const {
        float len = length();
        if (len > 1e-6f) {
            return *this * (1.0f / len);
        }
        return Vec2(0.0f, 0.0f);
    }
};

} // namespace fluid
