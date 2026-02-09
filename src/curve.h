// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "stripe.h"
#include "stitcher.h"
#include "isolines.h"

class Curve {
public:
    Curve(const float *vertsPositions, uint32_t vertsCount, const uint32_t *triangleIndices, uint32_t triangleCount,
        float width, const float *directions = nullptr, Stripe::Mode mode = Stripe::Mode::Extrinsic, bool resample = false, bool repulse = false, bool stitch = true, bool quiet = false);

    std::vector<std::vector<std::array<Vec3, 2>>> getCycles() const;
    void saveDirectionsToPLY(const char *path) const;
    void saveThickDirectionsToPLY(const char *path) const;
    void saveScalarsToPLY(const char *path) const;
    void saveCurveToPLY(const char *path) const;
    void saveThickCurveToPLY(const char *path) const;
    void saveCutsToPLY(const char *path) const;
    void saveMeshAndCurveToOBJ(const char *path) const;
    void saveStripeToPLY(const char* path) const;

private:
    Stripe stripe;
    Stitcher stitcher;
    Isolines isolines;
    float width;
    bool quiet;
};