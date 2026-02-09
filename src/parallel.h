// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include <functional>
#include <array>

namespace Parallel {
    void For(size_t start, size_t stop, std::function<void(size_t)> foo);
    bool ForAny(size_t start, size_t stop, std::function<bool(size_t)> foo);
    std::array<bool, 2> ForAny2(size_t start, size_t stop, std::function<std::array<bool, 2>(size_t)> foo);
    size_t ArgMin(size_t start, size_t stop, std::function<float(size_t)> foo);
}