//
// Created by Benjamin Lee on 11/9/23.
//

#pragma once

#include <cmath>
#include <limits>

class Vector2 {
public:
    Vector2() = default;
    Vector2(float x, float y) : x(x), y(y) {}

    Vector2 operator +(const Vector2 &p) const;
    Vector2 operator -(const Vector2 &p) const;
    Vector2 operator *(float scalar) const;
    Vector2 operator /(float scalar) const;
    Vector2 operator -() const;

    Vector2 operator +=(const Vector2 &p);
    Vector2 operator -=(const Vector2 &p);
    Vector2 operator *=(float scalar);
    Vector2 operator /=(float scalar);

    bool operator == (const Vector2 &p) const;
    bool operator != (const Vector2 &p) const;

    // return a normal vector with the same magnitude
    [[nodiscard]] Vector2 normal() const;

    // return a unit normal vector
    [[nodiscard]] Vector2 unit_normal() const;

    // return the magnitude/norm of the vector
    [[nodiscard]] float norm() const;

    // return the square of the magnitude/norm of the vector
    [[nodiscard]] float norm_squared() const;

    // dot product with vector v
    [[nodiscard]] float dot(const Vector2 &v) const;

    // cross product with vector v
    [[nodiscard]] float cross(const Vector2 &v) const;

    [[nodiscard]] Vector2 hadamard(const Vector2& v) const;

    // angle that the vector makes with the x-axis
    [[nodiscard]] float angle() const;

    // angle between this vector and vector v
    [[nodiscard]] float angle_between(const Vector2 &v) const;

    // check if the vector is parallel to vector v
    [[nodiscard]] bool is_parallel(const Vector2 &v, float tolerance = 0.001) const;

    [[nodiscard]] Vector2 unit_vector() const;

    // check if vector is undefined
    [[nodiscard]] bool is_undef() const;

    float x = 0;
    float y = 0;
};

Vector2 operator*(float scalar, const Vector2 &v);