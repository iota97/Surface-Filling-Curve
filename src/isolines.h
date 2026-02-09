// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "mathutils.h"
#include <vector>
#include <array>

class Isolines {
public:
    Isolines(const float *vertsPositions, const float *vertsNormals, const uint32_t *triangleIndices, uint32_t vertsCount, uint32_t triangleCount);
    Isolines() = default;

    void extract(const float *vertsScalars);
    std::vector<std::vector<std::array<Vec3, 2>>> getIsolines() const;
    void resample(float spacing);
    void repulse(float width);
    void saveToPLY(const char *path, float width, bool thick = false) const;
    void saveMeshAndCurveToOBJ(const char *path) const;

private:
    std::vector<std::vector<std::array<Vec3, 2>>> cycles;
    const float *vertsPositions;
    const float *vertsNormals;
    const uint32_t *triangleIndices;
    uint32_t vertsCount;
    uint32_t triangleCount;      
};