// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include <vector>
#include <cstddef>
#include <stdint.h>

class ObjLoader {
public:
    ObjLoader() = default;
    ObjLoader(const char* filename);

    size_t getIndexCount() const;
    size_t getVertCount() const;
    size_t getFaceCount() const;

    const uint32_t* getFaces() const;
    const float* getPositions() const;
    const float* getNormals() const;
    const float* getTexCoords() const;
    const float* getTangents() const;

    bool hasNormals() const;
    bool hasTangents() const;

    void load(const char* filename);

private:
    struct Vec3 {
        float x;
        float y;
        float z;
        Vec3 operator-(const Vec3& rhs);
        Vec3& operator+=(const Vec3& rhs);
        Vec3& operator-=(const Vec3& rhs);
        Vec3& normalize();
        static float dot(const Vec3& lhs, const Vec3& rhs);
        static Vec3 cross(Vec3 lhs, Vec3 rhs);
        Vec3 operator*(float rhs);
    };

    struct Tangent {
        Vec3 t;
        float w;
    };

    struct TexCoord {
        float u;
        float v;
        TexCoord operator-(const TexCoord& rhs);
    };

    struct Face {
        uint32_t a;
        uint32_t b;
        uint32_t c;
    };

    struct ObjVert {
        int32_t vert;
        int32_t norm;
        int32_t coord;
    };
    friend bool operator<(const ObjVert &lhs, const ObjVert &rhs);

    std::vector<Face> Faces;
    std::vector<Vec3> Positions;
    std::vector<Vec3> Normals;
    std::vector<Tangent> Tangents;
    std::vector<TexCoord> TexCoords;  
};
