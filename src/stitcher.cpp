// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "stitcher.h"
#include <memory.h>
#include <iostream>
#include <cassert>
#include <map>
#include <set>
#include <unordered_set>
#include <deque>

Stitcher::Stitcher(const float *vertsPosition, const float *vertsNormal, const uint32_t *triangles, uint32_t vertsCount, uint32_t facesCount, const std::set<std::array<uint32_t, 2>>& borderEdges)
        : vertsCount(vertsCount), facesCount(facesCount), borderEdges(borderEdges) {
    position.resize(3*vertsCount);
    normals.resize(3*vertsCount);
    scalars.resize(vertsCount);
    regionID.resize(vertsCount);
    distances.resize(vertsCount);
    prevID.resize(vertsCount);
    nextHop.resize(vertsCount);
    ringNumber.resize(vertsCount);
    mask.resize(vertsCount);
    isBorder.resize(vertsCount);
    faces.resize(3*facesCount);
    Parallel::For(0, facesCount, [this, triangles](size_t i){
        faces[3*i+0] = triangles[3*i+0];
        faces[3*i+1] = triangles[3*i+1];
        faces[3*i+2] = triangles[3*i+2];
    });

    Parallel::For(0, vertsCount, [this, vertsPosition, vertsNormal](size_t i){
        isBorder[i] = false;
        position[3*i+0] = vertsPosition[3*i+0];
        position[3*i+1] = vertsPosition[3*i+1];
        position[3*i+2] = vertsPosition[3*i+2];
        normals[3*i+0] = vertsNormal[3*i+0];
        normals[3*i+1] = vertsNormal[3*i+1];
        normals[3*i+2] = vertsNormal[3*i+2];
    });

    for (auto &edge : borderEdges) {
        isBorder[edge[0]] = true;
        isBorder[edge[1]] = true;
    }

    generateAdjacency();
}

void Stitcher::getConnectedComponents() {
    std::unordered_set<uint32_t> unexplored;
    for (uint32_t i = 0; i < vertsCount; ++i) {
        unexplored.insert(i);
    }
    std::vector<uint32_t> stack;
    uint32_t label = 0;
    connectedRegionPositiveOnlyCount = 0;
    while (!unexplored.empty()) {
        bool negative = false;
        uint32_t i = *unexplored.begin();
        unexplored.erase(i);
        stack.push_back(i);
        if (scalars[i] < 0) negative = true;
        while (!stack.empty()) {
            i = stack[stack.size()-1];
            stack.pop_back();
            for (auto n : getNeighborhood(i)) {
                uint32_t j = n.id;
                if (unexplored.count(j)) {
                    unexplored.erase(j);
                    stack.push_back(j);
                    if (scalars[j] < 0) negative = true;
                }
            }
        }
        label++;
        if (!negative) {
            connectedRegionPositiveOnlyCount++;
        }
    }
    connectedRegionCount = label - connectedRegionPositiveOnlyCount;
}

void Stitcher::floodRegion() {
    std::unordered_set<uint32_t> unexplored;
    for (uint32_t i = 0; i < vertsCount; ++i) {
        unexplored.insert(i);
    }
    std::vector<uint32_t> stack;
    uint32_t label = 0;
    positiveRegion = 0;
    negativeRegion = 0;
    while (!unexplored.empty()) {
        uint32_t i = *unexplored.begin();
        unexplored.erase(i);
        stack.push_back(i);
        regionID[i] = label;
        if (scalars[i] >= 0) {
            positiveRegion++;
        } else {
            negativeRegion++;
        }
        while (!stack.empty()) {
            i = stack[stack.size()-1];
            stack.pop_back();
            for (auto n : getNeighborhood(i)) {
                uint32_t j = n.id;
                if (unexplored.count(j) && ((scalars[i] >= 0 && scalars[j] >= 0) || (scalars[i] < 0 && scalars[j] < 0))) {
                    unexplored.erase(j);
                    stack.push_back(j);
                    regionID[j] = label;
                }
            }
        }
        label++;
    }
    positiveRegion -= connectedRegionPositiveOnlyCount;
}

void Stitcher::stitch(const float *vertsScalars, float period, bool stitch, bool showProgress) {
    Parallel::For(0, vertsCount, [this, vertsScalars](size_t i) {
        scalars[i] = vertsScalars[i];
    });

    if (!stitch) return;

    cleanIsolated();
    bool success = false;
    bool first = true;
    while (!success) {
        getConnectedComponents();
        floodRegion();
        if (first && showProgress) {
            first = false;
            std::cout << "Connected components having cycles: " << connectedRegionCount << std::endl;
            std::cout << "Regions: " << positiveRegion+negativeRegion << " (" << positiveRegion << " pos, " << negativeRegion << " neg)" << std::endl;
        }
        if (!connectedRegionCount) {
            break;
        }
        success = connectRegions(period, showProgress);
        if (!success) {
            if (showProgress) std::cout << "\nNo path found: subdividing..." << std::endl;
            subdivide();
        }
    }
}

void Stitcher::repulse() {
    computeRingNumber();
    Parallel::For(0, vertsCount, [this](size_t i) {
        if (!isBorder[i] && ringNumber[i] >= 4) {
            scalars[i] = scalars[i] >= 0 ? 1e3 : -1e3;
        }
    });
}

void Stitcher::cleanIsolated() {
    computeRingNumber();
    for (uint32_t i = 0; i < vertsCount; ++i) {
        if (getNeighborhood(i).begin() != getNeighborhood(i).end()) {
            uint32_t j = getNeighborhood(i).begin()->id;
            bool sameSign = (scalars[i] >= 0 && scalars[j] >= 0) || (scalars[i] < 0 && scalars[j] < 0);
            if (ringNumber[i] == 0 && !isBorder[i] && !sameSign) {
                scalars[i] = scalars[j];
            }
        }
    }
}

static void printProgress(const char *what, float percentage) {
    const char* bar = "||||||||||||||||||||||||||||||||||||||||";
    const uint32_t barWidth = std::strlen(bar);
    uint32_t val = (uint32_t) (percentage * 100);
    uint32_t lpad = (uint32_t) (percentage * barWidth);
    uint32_t rpad = barWidth - lpad;
    printf("\r%s: %3d%% [%.*s%*s]", what, val, lpad, bar, rpad, "");
    fflush(stdout);
    if (percentage == 1.0f) printf("\n");
}

bool Stitcher::connectRegions(float period, bool showProgress) {
    uint32_t parity = 0;
    float total = positiveRegion + negativeRegion - 2*connectedRegionCount;

    while (positiveRegion > connectedRegionCount || negativeRegion > connectedRegionCount) {
        bool positive = (++parity) % 2;
        if (positiveRegion == connectedRegionCount) positive = false;
        if (negativeRegion == connectedRegionCount) positive = true;
            
        bool success = connectRegion(period, positive);
        if (!success) {
            return false;
        }

        if (positive) {
            --positiveRegion;
        } else {
            --negativeRegion;
        }
        if (showProgress) {
            printProgress("Progress", 1.0f-(positiveRegion+negativeRegion-2*connectedRegionCount)/total);
        }
    }
    return true;
}

uint32_t Stitcher::getRingNumber(uint32_t i) {
    uint32_t ring = 0;
    bool first = true, start = true, next = true, prev = true;
    for (auto n : getNeighborhood(i)) {
        uint32_t j = n.id;
        next = scalars[j] >= 0;
        if (first) {
            first = false;
            start = next;
            prev = next;
        } else {
            if (next != prev) ++ring;
            prev = next;
        }
        }
    if (prev != start) ++ring;
    return ring;
}

void Stitcher::computeRingNumber() {
    Parallel::For(0, vertsCount, [this](size_t i) {
        ringNumber[i] = getRingNumber(i);
    });
}

void Stitcher::computeMask(bool positives) {
    auto test = [positives](float f) { return positives ? f < 0 : f >= 0; };
    computeRingNumber();

    Parallel::For(0, vertsCount, [this, &test](size_t i) {
        if (!test(scalars[i]) || isBorder[i] || ringNumber[i] > 4) {
            mask[i] = 1;
            return;
        }

        if (ringNumber[i] == 4) {
            mask[i] = 1;
            uint32_t region = uint32_t(-1);
            for (auto n : getNeighborhood(i)) {
                uint32_t j = n.id;
                if (!test(scalars[j])) {
                    if (region == uint32_t(-1)) {
                        region = regionID[j];
                    } else if (region != regionID[j]) {
                        mask[i] = 0;
                        break;
                    }
                }
            }
        } else {
            mask[i] = 0;
        }
    });
}


bool Stitcher::connectRegion(float period, bool positives) {
    auto test = [positives](float f) { return positives ? f < 0 : f >= 0; };
    computeMask(positives);

    Parallel::For(0, vertsCount, [this, &test](size_t i) {
        if (test(scalars[i])) {
            distances[i][0][0] = std::numeric_limits<float>::infinity();
            distances[i][1][0] = std::numeric_limits<float>::infinity();
            distances[i][0][1] = std::numeric_limits<float>::infinity();
            distances[i][1][1] = std::numeric_limits<float>::infinity();
            prevID[i][0][0] = uint32_t(-1);
            prevID[i][1][0] = uint32_t(-1);
            prevID[i][0][1] = uint32_t(-1);
            prevID[i][1][1] = uint32_t(-1);
        } else {
            distances[i][0][0] = 0.0f;
            distances[i][1][0] = 0.0f;
            distances[i][0][1] = 0.0f;
            distances[i][1][1] = 0.0f;
            prevID[i][0][0] = regionID[i];
            prevID[i][1][0] = regionID[i];
            prevID[i][0][1] = regionID[i];
            prevID[i][1][1] = regionID[i];
        }
    });

    for (uint32_t iter = 0; iter < 4; ++iter) {
        auto found = Parallel::ForAny2(0, vertsCount, [this, iter, test](size_t i) {
            if (!mask[i]) {
                bool change = false;
                distances[i][0][1-iter%2] = distances[i][0][iter%2];
                distances[i][1][1-iter%2] = distances[i][1][iter%2];
                prevID[i][0][1-iter%2] = prevID[i][0][iter%2];
                prevID[i][1][1-iter%2] = prevID[i][1][iter%2];
                for (auto n : getNeighborhood(i)) {
                    uint32_t j = n.id;
                    float dist_ij = test(scalars[j]) ? (Vec3(position.data(), i)-Vec3(position.data(), j)).length() : 0.0f;
                    if (distances[i][0][1-iter%2] > distances[j][0][iter%2] + dist_ij) {
                        if (prevID[i][0][1-iter%2] != prevID[j][0][iter%2]) {
                            distances[i][1][1-iter%2] = distances[i][0][1-iter%2];
                            prevID[i][1][1-iter%2] = prevID[i][0][1-iter%2];
                            nextHop[i][1] = nextHop[i][0];
                        }
                        distances[i][0][1-iter%2] = distances[j][0][iter%2] + dist_ij;
                        prevID[i][0][1-iter%2] = prevID[j][0][iter%2];
                        nextHop[i][0] = j;
                        change = true;
                    } else if (prevID[i][0][1-iter%2] != prevID[j][0][iter%2] && distances[i][1][1-iter%2] > distances[j][0][iter%2] + dist_ij) {
                        distances[i][1][1-iter%2] = distances[j][0][iter%2] + dist_ij;
                        prevID[i][1] = prevID[j][0];
                        nextHop[i][1] = j;
                        change = true;
                    } else if (prevID[i][0][1-iter%2] != prevID[j][1][iter%2] && distances[i][1][1-iter%2] > distances[j][1][iter%2] + dist_ij) {
                        distances[i][1][1-iter%2] = distances[j][1][iter%2] + dist_ij;
                        prevID[i][1][1-iter%2] = prevID[j][1][iter%2];
                        nextHop[i][1] = j;
                        change = true;
                    }
                }
                return std::array<bool, 2>{!std::isinf(distances[i][1][1-iter%2]), change};
            }
            return std::array<bool, 2>{false, false};
        });

        if (!found[0]) {
            iter %= 2;
            if (!found[1]) {
                if (positives) {
                    positiveRegion = connectedRegionCount+1;
                    return false;
                } else {
                    negativeRegion = connectedRegionCount+1;
                    return false;
                }
            }
        }
    }

    uint32_t startId = Parallel::ArgMin(0, vertsCount, [this, test](size_t i) {
        return test(scalars[i]) ? distances[i][0][0]+distances[i][1][0] : std::numeric_limits<float>::infinity();
    });

    uint32_t curr = startId;
    while (test(scalars[curr])) {
        curr = nextHop[curr][0];
    } 
    uint32_t closeID = prevID[curr][0][0];

    pathIds.clear();
    curr = startId;
    while (test(scalars[curr])) {
        scalars[curr] = positives ? 1.0f : -1.0f;
        pathIds.push_back(curr);
        regionID[curr] = closeID;
        curr = nextHop[curr][0];
    } 
    scalars[curr] = positives ? 1.0f : -1.0f;

    curr = nextHop[startId][1];
    while (test(scalars[curr])) {
        scalars[curr] = positives ? 1.0f : -1.0f;
        pathIds.push_back(curr);
        regionID[curr] = closeID;
        curr = prevID[curr][0][0] != closeID ? nextHop[curr][0] : nextHop[curr][1];
    } 
    scalars[curr] = positives ? 1.0f : -1.0f;
    uint32_t farID = prevID[curr][0][0];

    Parallel::For(0, vertsCount, [this, farID, closeID](size_t i) {
        if (regionID[i] == farID) {
            regionID[i] = closeID;
        }
        distances[i][0][0] = std::numeric_limits<float>::infinity();
    });

    widthenStitch(period, positives, closeID);
    return true;
}

void Stitcher::widthenStitch(float period, bool positives, uint32_t id) {
    std::deque<uint32_t> queue;
    std::unordered_set<uint32_t> toRetry;
    std::unordered_set<uint32_t> toRetryDeleted;
    std::unordered_set<uint32_t> explored;
    std::unordered_set<uint32_t> frontier;
    float width = 0.5f*period;

    for (auto i : pathIds) {
        distances[i][0][0] = 0.0f;
        for (auto n : getNeighborhood(i)) {
            uint32_t j = n.id;
            queue.push_back(j);
            explored.emplace(j);
            distances[j][0][0] = (Vec3(position.data(), i)-Vec3(position.data(), j)).length();
        }
    }
    while (!queue.empty()) {
        uint32_t i = queue.front();
        queue.pop_front();
        if (!isBorder[i]) {
            if (getRingNumber(i) == 2) {
                regionID[i] = id;
                scalars[i] = positives ? 1.0f : -1.0f;
                for (auto n : getNeighborhood(i)) {
                    uint32_t j = n.id;
                    distances[j][0][0] = std::min(distances[j][0][0], distances[i][0][0] + (Vec3(position.data(), i)-Vec3(position.data(), j)).length());
                    if (distances[j][0][0] < 0.5f*width) {
                        if (!explored.count(j)) {
                            queue.push_back(j);
                            explored.emplace(j);
                        }
                    }
                    if (!isBorder[j] && !frontier.count(j)) {
                        frontier.emplace(j);
                    }
                }
            } else {
                toRetry.emplace(i);
            }
        }

        if (queue.empty()) {
            for (uint32_t j : toRetry) {
                if (getRingNumber(j) == 2) {
                    queue.push_back(j);
                    toRetryDeleted.insert(j);
                }
            }
            for (uint32_t j : toRetryDeleted) {
                toRetry.erase(j);
            }
            toRetryDeleted.clear();
        }
    }
    
    for (auto i : frontier) {
        if (getRingNumber(i) == 2) {
            float weigths = 0;
            float val = 0;
            for (auto n : getNeighborhood(i)) {
                val += n.weight*scalars[n.id];
                weigths += n.weight;
            }
            scalars[i] = weigths == 0 ? scalars[i] : val/weigths;

            for (auto n : getNeighborhood(i)) {
                uint32_t j = n.id;
                if ((scalars[i] >= 0 && scalars[j] >= 0) || (scalars[i] < 0 && scalars[j] < 0)) {
                    regionID[i] = regionID[j];
                    break;
                }
            }
        }
    }
}

std::array<uint8_t, 3> Stitcher::getIDColor(uint32_t i) const {
    static const uint8_t colors[13][3] = {
        {184, 0, 88}, {235, 172, 35}, {0, 140, 249}, {0, 110, 0}, {0, 187, 173}, {209, 99, 230}, {189, 189, 189},
        {178, 69, 2}, {255, 146, 135}, {89, 84, 214}, {0, 198, 248}, {135, 133, 0}, {0, 167, 108}
    };
    uint32_t id = regionID[i];
    return std::array<uint8_t, 3>{colors[id%13][0], colors[id%13][1], colors[id%13][2]};
}

std::array<uint8_t, 3> Stitcher::getScalarColor(uint32_t i) const {
    float t = std::clamp(scalars[i]*0.5f+0.5f, 0.0f, 1.0f);

    static const float c0[3]{0.2777273272234177, 0.005407344544966578, 0.3340998053353061};
    static const float c1[3]{0.1050930431085774, 1.404613529898575, 1.384590162594685};
    static const float c2[3]{-0.3308618287255563, 0.214847559468213, 0.09509516302823659};
    static const float c3[3]{-4.634230498983486, -5.799100973351585, -19.33244095627987};
    static const float c4[3]{6.228269936347081, 14.17993336680509, 56.69055260068105};
    static const float c5[3]{4.776384997670288, -13.74514537774601, -65.35303263337234};
    static const float c6[3]{-5.435455855934631, 4.645852612178535, 26.3124352495832};

    uint8_t r = 255*(c0[0]+t*(c1[0]+t*(c2[0]+t*(c3[0]+t*(c4[0]+t*(c5[0]+t*c6[0]))))));
    uint8_t g = 255*(c0[1]+t*(c1[1]+t*(c2[1]+t*(c3[1]+t*(c4[1]+t*(c5[1]+t*c6[1]))))));
    uint8_t b = 255*(c0[2]+t*(c1[2]+t*(c2[2]+t*(c3[2]+t*(c4[2]+t*(c5[2]+t*c6[2]))))));

    return std::array<uint8_t, 3>{r, g, b};
}

void Stitcher::saveMeshToPLY(const char *path) const {
    std::ofstream plyFile(path);
    plyFile << "ply\nformat ascii 1.0\nelement vertex " << vertsCount << "\n"
            << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
            << "element face " << facesCount << "\nproperty list uchar uint vertex_indices\n"
            << "end_header\n";
        
    for (size_t i = 0; i < vertsCount; ++i) {
        auto color = getScalarColor(i);
        plyFile << position[3*i+0] << " " << position[3*i+1] << " " << position[3*i+2] << " " << uint32_t(color[0]) << " " << uint32_t(color[1]) << " " << uint32_t(color[2]) << std::endl;
    }

    for (size_t i = 0; i < facesCount; ++i) {
        plyFile << "3 " << faces[3*i+0] << " " << faces[3*i+1] << " " << faces[3*i+2] << std::endl;
    }
}

void Stitcher::generateAdjacency() {
    std::vector<std::unordered_map<uint32_t, float>> meshMap(vertsCount);
    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[i*3+0];
        uint32_t v1 = faces[i*3+1];
        uint32_t v2 = faces[i*3+2];

        meshMap[v0][v1] = 0.0f; meshMap[v0][v2] = 0.0f;
        meshMap[v1][v0] = 0.0f; meshMap[v1][v2] = 0.0f;
        meshMap[v2][v0] = 0.0f; meshMap[v2][v1] = 0.0f;
    }

    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[i*3+0];
        uint32_t v1 = faces[i*3+1];
        uint32_t v2 = faces[i*3+2];

        Vec3 p0(position.data(), v0); Vec3 p1(position.data(), v1); Vec3 p2(position.data(), v2);
        float c0 = std::clamp((p1-p0).dot(p2-p0)/((p1-p0).cross(p2-p0).length()), 0.0f, 1e8f);
        float c1 = std::clamp((p2-p1).dot(p0-p1)/((p2-p1).cross(p0-p1).length()), 0.0f, 1e8f);
        float c2 = std::clamp((p0-p2).dot(p1-p2)/((p0-p2).cross(p1-p2).length()), 0.0f, 1e8f);
        if (std::isnan(c0)) c0 = 0.0f;
        if (std::isnan(c1)) c1 = 0.0f;
        if (std::isnan(c2)) c2 = 0.0f;

        meshMap[v0][v1] += c2; meshMap[v0][v2] += c1;
        meshMap[v1][v0] += c2; meshMap[v1][v2] += c0;
        meshMap[v2][v0] += c1; meshMap[v2][v1] += c0;
    }

    uint32_t *edgeCountList = new uint32_t[vertsCount+1];
    edgeCountList[0] = 0;
    for (uint32_t i = 0; i < vertsCount; ++i) {
        edgeCountList[i+1] = edgeCountList[i] + meshMap[i].size();
    }
    edgesCount = edgeCountList[vertsCount];

    Edge **adjMatrixP = new Edge *[vertsCount+1];
    Edge *edgesP = new Edge[edgesCount];
    Parallel::For(0, vertsCount, [&meshMap, edgeCountList, &adjMatrixP, &edgesP](size_t i) {
        adjMatrixP[i] = edgesP + edgeCountList[i];
        uint32_t j = 0;
        for (auto v : meshMap[i]) {
            adjMatrixP[i][j++] = { v.first, v.second };
        }
    });

    adjMatrixP[vertsCount] = edgesP + edgesCount;
    delete[] edgeCountList;
    edges = std::unique_ptr<Edge[]>(edgesP);
    adjMatrix = std::unique_ptr<Edge *[]>(adjMatrixP);

    orderAdjacency();
}

void Stitcher::orderAdjacency() {
    Edge *edgesTMP = new Edge[edgesCount];
    std::unordered_map<uint32_t, std::vector<uint32_t>> vertsFaces;
    std::map<std::array<uint32_t, 2>, std::vector<uint32_t>> edgesFaces;
    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[i*3+0], v1 = faces[i*3+1], v2 = faces[i*3+2];
        vertsFaces[v0].push_back(i); vertsFaces[v1].push_back(i); vertsFaces[v2].push_back(i);
        edgesFaces[{std::min(v0, v1), std::max(v0, v1)}].push_back(i);
        edgesFaces[{std::min(v0, v2), std::max(v0, v2)}].push_back(i);
        edgesFaces[{std::min(v1, v2), std::max(v1, v2)}].push_back(i);
    }

    auto getEdge = [&](uint32_t i, uint32_t j) {
        for (auto n : getNeighborhood(i)) {
            if (n.id == j) return n;
        }
        return Edge();
    };

    Parallel::For(0, vertsCount, [this, edgesTMP, &vertsFaces, &edgesFaces, &getEdge](uint32_t i) {
        uint32_t offset = adjMatrix.get()[i]-adjMatrix.get()[0];
        if (isBorder[i]) {
            for (auto n : getNeighborhood(i)) {
                edgesTMP[offset++] = n;
            }
        } else {
            uint32_t f = vertsFaces[i][0];
            uint32_t v = uint32_t(-1);

            while(offset != adjMatrix.get()[i+1]-adjMatrix.get()[0]) {
                if (i != faces[f*3+0] && v != faces[f*3+0]) { edgesTMP[offset++] = getEdge(i, faces[f*3+0]); v = faces[f*3+0]; }
                else if (i != faces[f*3+1] && v != faces[f*3+1]) { edgesTMP[offset++] = getEdge(i, faces[f*3+1]); v = faces[f*3+1]; }
                else if (i != faces[f*3+2] && v != faces[f*3+2]) { edgesTMP[offset++] = getEdge(i, faces[f*3+2]); v = faces[f*3+2]; }
                f = edgesFaces[{std::min(i, v), std::max(i, v)}][0] != f ? edgesFaces[{std::min(i, v), std::max(i, v)}][0] : edgesFaces[{std::min(i, v), std::max(i, v)}][1];
            }
        }
    });

    Parallel::For(0, edgesCount, [&](uint32_t i) {
        edges.get()[i] = edgesTMP[i];
    });

    delete[] edgesTMP;
}

Stitcher::Neighborhood Stitcher::getNeighborhood(uint32_t i) const {
    Edge **matrix = adjMatrix.get();
    return Neighborhood{ matrix[i], matrix[i+1] };
}

void Stitcher::subdivide() {
    std::vector<uint32_t> newFaces(3*4*facesCount);
    std::map<std::array<uint32_t, 2>, uint32_t> edgesID;

    uint32_t id = 0;
    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[3*i+0], v1 = faces[3*i+1], v2 = faces[3*i+2];
        std::array<uint32_t, 2> e0{std::min(v0, v1), std::max(v0, v1)};
        std::array<uint32_t, 2> e1{std::min(v0, v2), std::max(v0, v2)};
        std::array<uint32_t, 2> e2{std::min(v1, v2), std::max(v1, v2)};

        if (!edgesID.count(e0)) edgesID[e0] = id++;
        if (!edgesID.count(e1)) edgesID[e1] = id++;
        if (!edgesID.count(e2)) edgesID[e2] = id++;
        uint32_t v01 = vertsCount+edgesID[e0], v02 = vertsCount+edgesID[e1], v12 = vertsCount+edgesID[e2];

        newFaces[3*(4*i+0)+0] = v0; newFaces[3*(4*i+0)+1] = v01; newFaces[3*(4*i+0)+2] = v02;
        newFaces[3*(4*i+1)+0] = v2; newFaces[3*(4*i+1)+1] = v02; newFaces[3*(4*i+1)+2] = v12;
        newFaces[3*(4*i+2)+0] = v12; newFaces[3*(4*i+2)+1] = v01; newFaces[3*(4*i+2)+2] = v1;
        newFaces[3*(4*i+3)+0] = v02; newFaces[3*(4*i+3)+1] = v01; newFaces[3*(4*i+3)+2] = v12;
    }

    std::vector<float> newPositions(3*(vertsCount+edgesID.size()));
    std::vector<float> newNormals(3*(vertsCount+edgesID.size()));
    std::vector<float> newScalars(vertsCount+edgesID.size());
    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[3*i+0], v1 = faces[3*i+1], v2 = faces[3*i+2];
        std::array<uint32_t, 2> e0{std::min(v0, v1), std::max(v0, v1)};
        std::array<uint32_t, 2> e1{std::min(v0, v2), std::max(v0, v2)};
        std::array<uint32_t, 2> e2{std::min(v1, v2), std::max(v1, v2)};
        uint32_t v01 = vertsCount+edgesID[e0], v02 = vertsCount+edgesID[e1], v12 = vertsCount+edgesID[e2];

        newPositions[3*v0+0] = position[3*v0+0]; newPositions[3*v0+1] = position[3*v0+1]; newPositions[3*v0+2] = position[3*v0+2];
        newPositions[3*v1+0] = position[3*v1+0]; newPositions[3*v1+1] = position[3*v1+1]; newPositions[3*v1+2] = position[3*v1+2];
        newPositions[3*v2+0] = position[3*v2+0]; newPositions[3*v2+1] = position[3*v2+1]; newPositions[3*v2+2] = position[3*v2+2];
        newNormals[3*v0+0] = normals[3*v0+0]; newNormals[3*v0+1] = normals[3*v0+1]; newNormals[3*v0+2] = normals[3*v0+2];
        newNormals[3*v1+0] = normals[3*v1+0]; newNormals[3*v1+1] = normals[3*v1+1]; newNormals[3*v1+2] = normals[3*v1+2];
        newNormals[3*v2+0] = normals[3*v2+0]; newNormals[3*v2+1] = normals[3*v2+1]; newNormals[3*v2+2] = normals[3*v2+2];

        Vec3 p01 = 0.5f*(Vec3(position.data(), v0)+Vec3(position.data(), v1));
        Vec3 p02 = 0.5f*(Vec3(position.data(), v0)+Vec3(position.data(), v2));
        Vec3 p12 = 0.5f*(Vec3(position.data(), v1)+Vec3(position.data(), v2));
        newPositions[3*v01+0] = p01.x; newPositions[3*v01+1] = p01.y; newPositions[3*v01+2] = p01.z;
        newPositions[3*v02+0] = p02.x; newPositions[3*v02+1] = p02.y; newPositions[3*v02+2] = p02.z;
        newPositions[3*v12+0] = p12.x; newPositions[3*v12+1] = p12.y; newPositions[3*v12+2] = p12.z;

        Vec3 n01 = (Vec3(normals.data(), v0)+Vec3(normals.data(), v1)).normalize();
        Vec3 n02 = (Vec3(normals.data(), v0)+Vec3(normals.data(), v2)).normalize();
        Vec3 n12 = (Vec3(normals.data(), v1)+Vec3(normals.data(), v2)).normalize();
        newPositions[3*v01+0] = n01.x; newPositions[3*v01+1] = n01.y; newPositions[3*v01+2] = n01.z;
        newPositions[3*v02+0] = n02.x; newPositions[3*v02+1] = n02.y; newPositions[3*v02+2] = n02.z;
        newPositions[3*v12+0] = n12.x; newPositions[3*v12+1] = n12.y; newPositions[3*v12+2] = n12.z;

        newScalars[v0] = scalars[v0]; newScalars[v1] = scalars[v1]; newScalars[v2] = scalars[v2];
        newScalars[v01] = 0.5f*(scalars[v0]+scalars[v1]);
        newScalars[v02] = 0.5f*(scalars[v0]+scalars[v2]);
        newScalars[v12] = 0.5f*(scalars[v1]+scalars[v2]);
    }

    auto tmpBorderEdges = borderEdges;
    borderEdges.clear();
    for (uint32_t i = 0; i < facesCount; ++i) {
        uint32_t v0 = faces[3*i+0], v1 = faces[3*i+1], v2 = faces[3*i+2];
        std::array<uint32_t, 2> e0{std::min(v0, v1), std::max(v0, v1)};
        std::array<uint32_t, 2> e1{std::min(v0, v2), std::max(v0, v2)};
        std::array<uint32_t, 2> e2{std::min(v1, v2), std::max(v1, v2)};
        uint32_t v01 = vertsCount+edgesID[e0], v02 = vertsCount+edgesID[e1], v12 = vertsCount+edgesID[e2];

        if (tmpBorderEdges.count(e0)) {
            borderEdges.insert({std::min(v0, v01), std::max(v0, v01)});
            borderEdges.insert({std::min(v01, v1), std::max(v01, v1)});
        }
        if (tmpBorderEdges.count(e1)) {
            borderEdges.insert({std::min(v0, v02), std::max(v0, v02)});
            borderEdges.insert({std::min(v02, v2), std::max(v02, v2)});
        }
        if (tmpBorderEdges.count(e2)) {
            borderEdges.insert({std::min(v1, v12), std::max(v1, v12)});
            borderEdges.insert({std::min(v12, v2), std::max(v12, v2)});
        }
    }

    faces = std::move(newFaces);
    position = std::move(newPositions);
    normals = std::move(newNormals);
    scalars = std::move(newScalars);
    facesCount = faces.size()/3;
    vertsCount = position.size()/3;
    regionID.resize(vertsCount);
    distances.resize(vertsCount);
    prevID.resize(vertsCount);
    nextHop.resize(vertsCount);
    ringNumber.resize(vertsCount);
    mask.resize(vertsCount);
    isBorder.resize(vertsCount);

    Parallel::For(0, vertsCount, [this](size_t i){
        isBorder[i] = false;
    });

    for (auto &edge : borderEdges) {
        isBorder[edge[0]] = true;
        isBorder[edge[1]] = true;
    }

    generateAdjacency();
}
