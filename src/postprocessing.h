// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "mathutils.h"
#include <vector>
#include <array>

void medialAxisRepulsion(std::vector<std::vector<std::array<Vec3, 2>>> &cycles, const std::vector<std::array<Vec3, 3>> &triangles, float spacing, uint32_t iterationsCount);