// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "isolines.h"
#include "parallel.h"
#include "postprocessing.h"
#include <cassert>
#include <unordered_map>
#include <bitset>
#include <fstream>
#include <iostream>

struct Edge {
    uint32_t v0, v1;
    Edge(uint32_t v0, uint32_t v1) : v0(v0), v1(v1) {}
    bool operator==(const Edge &o) const { return v0 == o.v0 && v1 == o.v1; }
    bool operator!=(const Edge &o) const { return v0 != o.v0 || v1 != o.v1; }
};

template <>
struct std::hash<Edge> {
    std::size_t operator()(const Edge& k) const {
        return (std::hash<uint32_t>()(k.v0)^(std::hash<uint32_t>()(k.v1) << 1)) >> 1;
    }
};

Isolines::Isolines(const float *vertsPositions, const float *vertsNormals, const uint32_t *triangleIndices, uint32_t vertsCount, uint32_t triangleCount) :
    vertsPositions(vertsPositions), vertsNormals(vertsNormals), triangleIndices(triangleIndices), vertsCount(vertsCount), triangleCount(triangleCount) {}

std::vector<std::vector<std::array<Vec3, 2>>> Isolines::getIsolines() const {
    return cycles;
}

void Isolines::extract(const float *vertsScalars) {
    std::unordered_multimap<Edge, Edge> edgeMap;
    for (uint32_t i = 0; i < triangleCount; ++i) {
        uint32_t i0 = triangleIndices[i*3+0], i1 = triangleIndices[i*3+1], i2 = triangleIndices[i*3+2];
        float v0 = vertsScalars[i0], v1 = vertsScalars[i1], v2 = vertsScalars[i2];
        uint8_t mask = ((v0 >= 0) << 2) | ((v1 >= 0) << 1) | ((v2 >= 0) << 0);
        switch (mask) {
            case 0b001:
            case 0b110:
                edgeMap.insert({{std::min(i0, i2), std::max(i0, i2)}, {std::min(i1, i2), std::max(i1, i2)}});
                edgeMap.insert({{std::min(i1, i2), std::max(i1, i2)}, {std::min(i0, i2), std::max(i0, i2)}});
                break;
            case 0b010:
            case 0b101:
                edgeMap.insert({{std::min(i0, i1), std::max(i0, i1)}, {std::min(i1, i2), std::max(i1, i2)}});
                edgeMap.insert({{std::min(i1, i2), std::max(i1, i2)}, {std::min(i0, i1), std::max(i0, i1)}});
                break;
            case 0b011:
            case 0b100:
                edgeMap.insert({{std::min(i0, i1), std::max(i0, i1)}, {std::min(i0, i2), std::max(i0, i2)}});
                edgeMap.insert({{std::min(i0, i2), std::max(i0, i2)}, {std::min(i0, i1), std::max(i0, i1)}});
                break;
            default:
                break;
        }
    }

    std::vector<Edge> cycle;
    std::vector<std::array<Vec3, 2>> cycleVerts;
    cycles.clear();
    while (true) {
        if (edgeMap.empty())
            break;

        auto current = edgeMap.begin();
        Edge first = current->first;
        while (true) {
            cycle.push_back(current->first);
            bool found = false;
            for (auto[itr, rangeEnd] = edgeMap.equal_range(current->second); itr != rangeEnd; ++itr) {
                if (itr->second != current->first) {
                    found = true;
                    current = itr;
                    break;
                }
            }
            assert(found);

            if (current->first == first) {
                break;
            }
        }
        for (auto &e : cycle) {
            edgeMap.erase(e);
        }

        for (uint32_t i = 0; i < cycle.size(); ++i) {
            uint32_t i0 = cycle[i].v0, i1 = cycle[i].v1;
            Vec3 p0(vertsPositions, i0), p1(vertsPositions, i1);
            Vec3 n0(vertsNormals, i0), n1(vertsNormals, i1);
            float v0 = vertsScalars[i0] ? vertsScalars[i0] : 1e-8, v1 = vertsScalars[i1] ? vertsScalars[i1] : 1e-8;
            cycleVerts.push_back({p0-v0/(v1-v0)*(p1-p0), (n0-v0/(v1-v0)*(n1-n0)).normalize()});
        }
        cycles.push_back(cycleVerts);
        cycle.clear();
        cycleVerts.clear();
    }
}

void Isolines::saveToPLY(const char *path, float width, bool thick) const {
    if (!thick) {
        std::vector<Vec3> plyVerts;
        std::vector<Vec3> plyNorms;
        std::vector<std::array<uint32_t, 2>> plyEdges;
        uint32_t curr = 0;
        for (auto &cycle : cycles) {
            uint32_t length = cycle.size();
            for (uint32_t i = 0; i < length; ++i) {
                plyVerts.push_back(cycle[i][0]);
                plyNorms.push_back(cycle[i][1]);
                plyEdges.push_back({curr+i, curr+(i+1)%length});
            }
            curr += length;
        }

        std::ofstream plyFile(path);
        plyFile << "ply\nformat ascii 1.0\nelement vertex " << plyVerts.size() << "\n"
                << "property float x\nproperty float y\nproperty float z\nproperty float nx\nproperty float ny\nproperty float nz\n"
                << "element edge " << plyEdges.size() << "\nproperty int vertex1\nproperty int vertex2\n"
                << "end_header\n";
        
        for (uint32_t i = 0; i < plyVerts.size(); ++i) {
            plyFile << plyVerts[i].x << " " << plyVerts[i].y << " " << plyVerts[i].z << " " 
                    << plyNorms[i].x << " " << plyNorms[i].y << " " << plyNorms[i].z << std::endl;
        }

        for (auto &e: plyEdges) {
            plyFile << e[0] << " " << e[1] << std::endl;
        }

    } else {
        static const uint8_t colors[13][3] = {
            {184, 0, 88}, {235, 172, 35}, {0, 140, 249}, {0, 110, 0}, {0, 187, 173}, {209, 99, 230}, {189, 189, 189},
            {178, 69, 2}, {255, 146, 135}, {89, 84, 214}, {0, 198, 248}, {135, 133, 0}, {0, 167, 108}
        };

        std::vector<Vec3> plyVerts;
        std::vector<std::array<uint8_t, 3>> plyColors;
        std::vector<std::array<uint32_t, 4>> plyQuad;
        uint32_t curr = 0;
        uint32_t cycleCount = cycles.size();
        for (uint32_t j = 0; j < cycleCount; ++j) {
            auto &cycle = cycles[j];
            uint32_t length = cycle.size();
            for (size_t i = 0; i < length+1; ++i) {
                Vec3 p0 = cycle[i%length][0], p1 = cycle[(i+1)%length][0];
                Vec3 n0 = cycle[i%length][1];
                Vec3 b0 = n0.cross(p0-p1).normalize();

                plyVerts.push_back(p0 + 0.5f*width*n0 + width*b0);
                plyVerts.push_back(p0 - 0.5f*width*n0 + width*b0);
                plyVerts.push_back(p0 - 0.5f*width*n0 - width*b0);
                plyVerts.push_back(p0 + 0.5f*width*n0 - width*b0);
                plyColors.push_back({colors[j%13][0], colors[j%13][1], colors[j%13][2]});

                if (i < length) {
                    plyQuad.push_back({curr, curr+1, curr+5, curr+4});
                    plyQuad.push_back({curr+2, curr+3, curr+7, curr+6});
                    plyQuad.push_back({curr, curr+3, curr+7, curr+4});
                    plyQuad.push_back({curr+2, curr+1, curr+5, curr+6});
                }
                curr += 4;
            }
        }
        
        std::ofstream plyFile(path);
        plyFile << "ply\nformat ascii 1.0\nelement vertex " << plyVerts.size() << "\n"
                << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
                << "element face " << plyQuad.size() << "\nproperty list uchar uint vertex_indices\n"
                << "end_header\n";
        
        for (uint32_t i = 0; i < plyVerts.size(); ++i) {
            auto &color = plyColors[i/4];
            plyFile << plyVerts[i].x << " " << plyVerts[i].y << " " << plyVerts[i].z << " " 
                    << uint32_t(color[0]) << " " << uint32_t(color[1]) << " " << uint32_t(color[2]) << std::endl;
        }

        for (auto &f: plyQuad) {
            plyFile << "4 " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;
        }
    }
}


struct GridCell {
    int32_t x, y, z;
    bool operator==(const GridCell &o) const { return x == o.x && y == o.y && z == o.z; };
};
template <>
struct std::hash<GridCell> {
    size_t operator()(const GridCell& k) const {
        return ((std::hash<int32_t>()(k.x)
                ^ (hash<int32_t>()(k.y) << 1)) >> 1)
                ^ (hash<int32_t>()(k.z) << 1);
    }
};

void Isolines::repulse(float width) {
    std::vector<std::array<Vec3, 3>> triangles(triangleCount);
    for (uint32_t i = 0; i < triangleCount; ++i){
        Vec3 a(vertsPositions, triangleIndices[3*i+0]);
        Vec3 b(vertsPositions, triangleIndices[3*i+1]);
        Vec3 c(vertsPositions, triangleIndices[3*i+2]);
        triangles[i] = {a, b, c};
    }
    medialAxisRepulsion(cycles, triangles, width, 8);
}

void Isolines::resample(float spacing) {
    std::vector<std::vector<std::array<Vec3, 2>>> newCycles;
    for (auto &cycle : cycles) {
        if (cycle.empty()) continue;
        std::vector<std::array<Vec3, 2>> newCycle;
        newCycle.push_back(cycle[1]);
        for (size_t i = 1; i < cycle.size(); ++i) {
            float segDistance = (cycle[i][0]-newCycle.back()[0]).length();
            if (segDistance >= spacing) {
                float t = spacing/segDistance;
                Vec3 pos = t*cycle[i][0] + (1-t)*cycle[i-1][0];
                Vec3 norm = (t*cycle[i][1] + (1-t)*cycle[i-1][1]).normalize();
                newCycle.push_back({pos, norm});
            }
        }
        newCycles.emplace_back(std::move(newCycle));
    }
    cycles = std::move(newCycles);
}

void Isolines::saveMeshAndCurveToOBJ(const char *path) const {
    std::ofstream objFile(path);
    for (uint32_t i = 0; i < vertsCount; ++i ) {
        objFile << "v " << vertsPositions[3*i+0] << " " << vertsPositions[3*i+2] << " " << vertsPositions[3*i+1] << std::endl; 
    }

    for (auto &cycle : cycles) {
        for (auto &v : cycle) {
            objFile << "v " << v[0].x << " " << v[0].z << " " << v[0].y << std::endl; 
        }
    }

    for (uint32_t i = 0; i < triangleCount; ++i ) {
        objFile << "f " << triangleIndices[3*i+0]+1 << " " << triangleIndices[3*i+1]+1 << " " << triangleIndices[3*i+2]+1 << std::endl; 
    }

    uint32_t offset = vertsCount+1;
    for (auto &cycle : cycles) {
        uint32_t start = offset;
        objFile << "l ";
        for (size_t i = 0; i < cycle.size(); ++i) {
            objFile << offset++ << " "; 
        }
        objFile << start << std::endl;
    }
}