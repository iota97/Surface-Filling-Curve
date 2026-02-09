// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <limits>

struct Vec3 {
    float x, y, z;
    Vec3() = default;
    Vec3(float v) : x(v), y(v), z(v) {}
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
    Vec3(const float *data, uint32_t idx) : x(data[3*idx+0]), y(data[3*idx+1]), z(data[3*idx+2]) {}
    float &operator[](uint8_t axis) { return axis ? (axis == 1 ? y : z) : x; }
    float operator[](uint8_t axis) const { return axis ? (axis == 1 ? y : z) : x; }
    Vec3 operator+(const Vec3& o) const { Vec3 res; res.x = x + o.x; res.y = y + o.y; res.z = z + o.z; return res; }
    void operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; }
    void operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; }
    Vec3 operator-(const Vec3& o) const { Vec3 res; res.x = x - o.x; res.y = y - o.y; res.z = z - o.z; return res; }
    Vec3 operator-() const { Vec3 res; res.x = -x; res.y = -y; res.z = -z; return res; }
    Vec3 operator*(float s) const { Vec3 res; res.x = x*s; res.y = y*s; res.z = z*s; return res; }
    Vec3 operator/(float s) const { Vec3 res; res.x = x/s; res.y = y/s; res.z = z/s; return res; }
    float dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    float length() const { return sqrt(x*x + y*y + z*z); }
    float length2() const { return x*x + y*y + z*z; }
    Vec3 normalize() const { Vec3 res; float l = length(); res.x = x/l; res.y = y/l; res.z = z/l; return l > 0 ? res : Vec3(1, 0, 0); }
    Vec3 project(const Vec3& n) const { return n * dot(n); }
    Vec3 orthonormalize(const Vec3& n) const { return (*this - project(n)).normalize(); }
    Vec3 normalToTangent() const { float s = z > 0.0f ? 1.0f : -1.0f, a = -1.0f/(s+z), b = x*y*a; return Vec3(1.0f+s*x*x*a, s*b, -s*x); }
    Vec3 cross(const Vec3 &rhs) const { return Vec3{y * rhs.z - z * rhs.y, -(x * rhs.z - z * rhs.x), x * rhs.y - y * rhs.x}; }
    float normSqr() const { return dot(*this); }
    bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
    bool operator!=(const Vec3& o) const { return x != o.x || y != o.y || z != o.z; }
    bool operator<(const Vec3& o) const { if (x < o.x) return true; if (x > o.x) return false; if (y < o.y) return true; if (y > o.y) return false; if (z < o.z) return true; return false; }
};
inline Vec3 operator*(float s, const Vec3& v) { return v*s; }

inline uint32_t randomUintFromUint(uint32_t v) {
    uint32_t state = 747796405*v + 2891336453;
    uint32_t word = ((state >> ((state >> 28) + 4)) ^ state) * 277803737;
    return (word >> 22) ^ word;
}

inline Vec3 triangleNormal(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    return (a-c).cross(b-c).normalize();
}

inline float randomFloatFromUint(uint32_t v) {
    return float(randomUintFromUint(v))/std::numeric_limits<uint32_t>::max();
}