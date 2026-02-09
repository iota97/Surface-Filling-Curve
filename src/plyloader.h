// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include <vector>
#include <cstddef>
#include <stdint.h>

class PlyLoader {
public:
    PlyLoader() = default;
    PlyLoader(const char* filename);

    void load(const char* filename);

    const uint32_t* getFaces() const {
        return (const uint32_t*) Faces.data();
    }

    const float* getPositions() const {
        return (const float*) Positions.data();
    }

    const uint8_t* getColors() const {
        return (const uint8_t*) Colors.data();
    }

    size_t getVertCount() const {
        return Positions.size();
    }

    size_t getFaceCount() const {
        return Faces.size();
    }

private:
    struct Face {
        uint32_t a;
        uint32_t b;
        uint32_t c;
    };

    struct Vec3 {
        float x;
        float y;
        float z;
    };

    struct Color {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    std::vector<Face> Faces;
    std::vector<Vec3> Positions;
    std::vector<Color> Colors;
};