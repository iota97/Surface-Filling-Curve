// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "mathutils.h"
#include <vector>
#include <set>

class Subdivide {
public:
    Subdivide(const float *vertsPosition, const float *scalars, const float *directions, const uint32_t *trianglesID, uint32_t vertsCount, uint32_t trisCount, float maxLength, bool quiet = true, bool cut = true);
    
    const float *getPositions() const {
        return positions.data();
    }

    const float *getScalars() const {
        return scalars.data();
    }

    const float *getDirections() const {
        return directions.data();
    }

    const uint32_t *getFaces() const {
        return triangles.data();
    }

    uint32_t getVertCount() const {
        return positions.size()/3;
    }

    uint32_t getFaceCount() const {
        return triangles.size()/3;
    }

    const std::set<std::array<uint32_t, 2>> &getBorderEdges() const {
        return borderEdges;
    }

private:
    std::vector<float> positions;
    std::vector<float> scalars;
    std::vector<float> directions;
    std::vector<uint32_t> triangles;
    std::set<std::array<uint32_t, 2>> borderEdges;
};