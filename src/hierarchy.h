// MIT License
// Copyright 2026 Giovanni Cocco and Inria

// implementation of the hierarchy from Instant Field-Aligned Meshes (https://igl.ethz.ch/projects/instant-meshes/).
// Templated to support arbitrary data and optimization function.

#pragma once
#include "mathutils.h"
#include "parallel.h"
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <functional>
#include <thread>

struct BaseVertexData {
    Vec3 position;
    Vec3 normal;
};

template <class T>
class MeshHierarchy {
public:
    MeshHierarchy(const float *vertsPositions, const uint32_t *triangleIndices, uint32_t vertsCount, uint32_t triangleCount) : vertsCount(vertsCount) {
        vertex = new T[vertsCount];
        Parallel::For(0, vertsCount, [this, vertsPositions](size_t i) {
            vertex[i].position = Vec3(vertsPositions, i);
            vertex[i].normal = Vec3(0, 0, 0);
        });

        std::vector<std::unordered_map<uint32_t, float>> meshMap(vertsCount);
        faces = new uint32_t[3*triangleCount];
        facesCount = triangleCount;
        for (uint32_t i = 0; i < triangleCount; ++i) {
            uint32_t v0 = triangleIndices[i*3+0];
            uint32_t v1 = triangleIndices[i*3+1];
            uint32_t v2 = triangleIndices[i*3+2];
            faces[i*3+0] = v0; faces[i*3+1] = v1; faces[i*3+2] = v2;

            meshMap[v0][v1] = 0.0f; meshMap[v0][v2] = 0.0f;
            meshMap[v1][v0] = 0.0f; meshMap[v1][v2] = 0.0f;
            meshMap[v2][v0] = 0.0f; meshMap[v2][v1] = 0.0f;

            Vec3 p0(vertsPositions, v0); Vec3 p1(vertsPositions, v1); Vec3 p2(vertsPositions, v2);
            Vec3 n = (p0-p2).cross(p1-p2);
            Vec3 l {(p0-p1).normSqr(), (p1-p2).normSqr(), (p2-p0).normSqr()};

            vertex[v0].normal += n/(l.x + l.z);
            vertex[v1].normal += n/(l.y + l.x);
            vertex[v2].normal += n/(l.z + l.y);
        }

        for (uint32_t i = 0; i < triangleCount; ++i) {
            uint32_t v0 = triangleIndices[i*3+0];
            uint32_t v1 = triangleIndices[i*3+1];
            uint32_t v2 = triangleIndices[i*3+2];

            Vec3 p0(vertsPositions, v0); Vec3 p1(vertsPositions, v1); Vec3 p2(vertsPositions, v2);
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

        Parallel::For(0, vertsCount, [this](size_t i) {
            vertex[i].normal = vertex[i].normal.normalize();
        });

        uint32_t *edgeCountList = new uint32_t[vertsCount+1];
        edgeCountList[0] = 0;
        for (uint32_t i = 0; i < vertsCount; ++i) {
            edgeCountList[i+1] = edgeCountList[i] + meshMap[i].size();
        }
        edgesCount = edgeCountList[vertsCount];

        adjMatrix = new Edge *[vertsCount+1];
        edges = new Edge[edgesCount];
        areas = new float[vertsCount];
        Parallel::For(0, vertsCount, [this](size_t i) {
            areas[i] = 1.0f;
        });

        Parallel::For(0, vertsCount, [this, &meshMap, edgeCountList](size_t i) {
            adjMatrix[i] = edges + edgeCountList[i];
            uint32_t j = 0;
            for (auto v : meshMap[i]) {
                adjMatrix[i][j++] = { v.first, v.second };
            }
        });

        adjMatrix[vertsCount] = edges + edgesCount;
        delete[] edgeCountList;

        coarseMesh();
    }

    MeshHierarchy(const MeshHierarchy &) = delete;

    MeshHierarchy(MeshHierarchy &&other) {
        areas = other.areas; other.areas = nullptr;
        vertex = other.vertex; other.vertex = nullptr;
        adjMatrix = other.adjMatrix; other.adjMatrix = nullptr;
        edges = other.edges; other.edges = nullptr;
        faces = other.faces; other.faces = nullptr;
        toDense = other.toDense; other.toDense = nullptr;
        toCoarse = other.toCoarse; other.toCoarse = nullptr;
        coarse = other.coarse; other.coarse = nullptr;
        dense = other.dense; other.dense = nullptr;
        facesCount = other.facesCount; other.facesCount = 0;
        edgesCount = other.edgesCount; other.edgesCount = 0;
        vertsCount = other.vertsCount; other.vertsCount = 0;
    }

    ~MeshHierarchy() {
        delete[] areas;
        delete[] vertex;
        delete[] toDense;
        delete[] toCoarse;
        delete[] edges;
        delete[] faces;
        delete[] adjMatrix;
        delete coarse;
    }

    MeshHierarchy &operator=(const MeshHierarchy &) = delete;

    MeshHierarchy &operator=(MeshHierarchy &&other) {
        if (this != &other) {
            areas = other.areas; other.areas = nullptr;
            vertex = other.vertex; other.vertex = nullptr;
            adjMatrix = other.adjMatrix; other.adjMatrix = nullptr;
            edges = other.edges; other.edges = nullptr;
            faces = other.faces; other.faces = nullptr;
            toDense = other.toDense; other.toDense = nullptr;
            toCoarse = other.toCoarse; other.toCoarse = nullptr;
            coarse = other.coarse; other.coarse = nullptr;
            dense = other.dense; other.dense = nullptr;
            facesCount = other.facesCount; other.facesCount = 0;
            edgesCount = other.edgesCount; other.edgesCount = 0;
            vertsCount = other.vertsCount; other.vertsCount = 0;
        }
        return *this;
    }

    struct Edge {
        uint32_t id;
        float weight;
        bool operator<(const Edge& o) const { return id < o.id; }
    };

    struct Neighborhood {
        Neighborhood(Edge *s, Edge *e) : s(s), e(e) {}
        Edge *s, *e;
        Edge *begin() const { return s; }
        Edge *end() const { return e; }
    };

    struct Links {
        Links(uint32_t *s, uint32_t *e) : s(s), e(e) {}
        uint32_t*s, *e;
        uint32_t *begin() const { return s; }
        uint32_t *end() const { return e; }
    };

    typedef std::function<void(uint32_t, T *)> InitFunction;
    typedef std::function<void(uint32_t, const Neighborhood&, T *, uint32_t)> SmoothFunction;
    typedef std::function<void(uint32_t, const Links&, T *, T *)> RestrictFunction;
    typedef std::function<void(uint32_t, uint32_t, T *, T *)> ProlongFunction;
    typedef std::function<std::array<uint8_t, 3>(uint32_t, T *)> ColorFunction;

    void saveMeshToPLY(const char *path, ColorFunction colorFunc) const {
        std::ofstream plyFile(path);
        plyFile << "ply\nformat ascii 1.0\nelement vertex " << vertsCount << "\n"
                << "property float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n"
                << "element face " << facesCount << "\nproperty list uchar uint vertex_indices\n"
                << "end_header\n";
        
        for (size_t i = 0; i < vertsCount; ++i) {
            auto color = colorFunc(i, vertex);
            plyFile << vertex[i].position.x << " " << vertex[i].position.y << " " << vertex[i].position.z << " " << uint32_t(color[0]) << " " << uint32_t(color[1]) << " " << uint32_t(color[2]) << std::endl;
        }

        for (size_t i = 0; i < facesCount; ++i) {
            plyFile << "3 " << faces[3*i+0] << " " << faces[3*i+1] << " " << faces[3*i+2] << std::endl;
        }
    }

    void optimize(const InitFunction &initFunc, const SmoothFunction &smoothFunc, const RestrictFunction &restrictFunc, const ProlongFunction &prolongFunc, uint32_t iterationCount = 64) {
        if (!dense)
            init(initFunc);

        if (coarse) {
            restrict(restrictFunc);
            coarse->optimize(initFunc, smoothFunc, restrictFunc, prolongFunc, iterationCount);
            prolong(prolongFunc);
            smooth(smoothFunc, iterationCount);
        }
    }

    const T *getVertexData() const {
        return vertex;
    }

    Edge **getAdjMatrix() const {
        return adjMatrix;
    }

    Edge *getEdges() const {
        return edges;
    }

    uint32_t getEdgeCount() const {
        return edgesCount;
    }

    uint32_t getLodsCount() const {
        uint32_t count = 1;
        MeshHierarchy* curr = coarse;
        while (curr) {
            curr = curr->coarse;
            count++;
        }
        return count;
    }

private:
    MeshHierarchy() = default;

    void coarseMesh() {
        struct EdgeScore {
            uint32_t i, j;
            float score;
            bool operator<(const EdgeScore &e) const { return score > e.score; }
        };

        EdgeScore *edgesScore = new EdgeScore[edgesCount];
        bool *collapsedFlag = new bool[vertsCount];
        Parallel::For(0, vertsCount, [collapsedFlag](size_t i) {
            collapsedFlag[i] = false;
        });

        Parallel::For(0, vertsCount, [this, edgesScore](size_t i) {
            for (uint32_t j = 0; j < adjMatrix[i+1] - adjMatrix[i]; ++j) {
                uint32_t k = adjMatrix[i][j].id;
                edgesScore[adjMatrix[i] - adjMatrix[0] + j].i = i;
                edgesScore[adjMatrix[i] - adjMatrix[0] + j].j = k;
                float ratio = areas[i] > areas[k] ? areas[i]/areas[k] : areas[k]/areas[i];
                edgesScore[adjMatrix[i] - adjMatrix[0] + j].score = vertex[i].normal.dot(vertex[k].normal) * ratio;
            }
        });
        std::sort(edgesScore, edgesScore + edgesCount, std::less<EdgeScore>());

        uint32_t collapasedCount = 0;
        for (uint32_t i = 0; i < edgesCount; ++i) {
            if (!collapsedFlag[edgesScore[i].i] && !collapsedFlag[edgesScore[i].j]) {
                collapsedFlag[edgesScore[i].i] = collapsedFlag[edgesScore[i].j] = true;
                edgesScore[collapasedCount++] = edgesScore[i];
            }
        }

        if (!collapasedCount) {
            delete[] edgesScore;
            delete[] collapsedFlag;
            return;
        }

        coarse = new MeshHierarchy();
        coarse->vertsCount = vertsCount - collapasedCount;
        coarse->dense = this;
        coarse->vertex = new T[coarse->vertsCount];
        coarse->areas = new float[coarse->vertsCount];

        coarse->toDense = new uint32_t[2*coarse->vertsCount];
        toCoarse = new uint32_t[vertsCount];

        Parallel::For(0, collapasedCount, [this, edgesScore](size_t i) {
            float a_i = areas[edgesScore[i].i], a_j = areas[edgesScore[i].j];
            coarse->vertex[i].position = (a_i*vertex[edgesScore[i].i].position + a_j*vertex[edgesScore[i].j].position)/(a_i+a_j);
            coarse->vertex[i].normal = (a_i*vertex[edgesScore[i].i].normal + a_j*vertex[edgesScore[i].j].normal)/(a_i+a_j);
            coarse->vertex[i].normal = coarse->vertex[i].normal.normalize();

            coarse->areas[i] = a_i+a_j;
            coarse->toDense[2*i+0] = edgesScore[i].i;
            coarse->toDense[2*i+1] = edgesScore[i].j;
            toCoarse[edgesScore[i].i] = i;
            toCoarse[edgesScore[i].j] = i;
        });
        delete[] edgesScore;

        uint32_t offset = collapasedCount;
        for (uint32_t i = 0; i < vertsCount; ++i) {
            if (!collapsedFlag[i]) {
                uint32_t idx = offset++;
                coarse->vertex[idx].position = vertex[i].position;
                coarse->vertex[idx].normal = vertex[i].normal;
                coarse->areas[idx] = areas[i];
                coarse->toDense[2*idx+0] = i;
                coarse->toDense[2*idx+1] = -1;
                toCoarse[i] = idx;
            }
        }

        delete[] collapsedFlag;

        uint32_t *edgeCountListCoarse = new uint32_t[coarse->vertsCount+1];

        std::vector<Edge> neighbors;
        for (uint32_t i = 0; i < coarse->vertsCount; ++i) {
            neighbors.clear();
            for (uint32_t j = 0; j < 2; j++) {
                uint32_t d = coarse->toDense[2*i+j];
                if (d != uint32_t(-1)) {
                    for (Edge *n = adjMatrix[d]; n != adjMatrix[d+1]; ++n) {
                        neighbors.push_back({toCoarse[n->id], n->weight});
                    }
                }
            }

            std::sort(neighbors.begin(), neighbors.end());
            uint32_t last = -1, size = 0;
            for (auto n : neighbors) {
                if (n.id != i && n.id != last) {
                    last = n.id;
                    ++size;
                }
            }
            edgeCountListCoarse[i+1] = size;
        }

        edgeCountListCoarse[0] = 0;
        for (uint32_t i = 0; i < coarse->vertsCount; ++i) {
            edgeCountListCoarse[i+1] += edgeCountListCoarse[i];
        }
        coarse->edgesCount = edgeCountListCoarse[coarse->vertsCount];
        coarse->adjMatrix = new Edge *[coarse->vertsCount+1];
        coarse->edges = new Edge[coarse->edgesCount];

        for (uint32_t i = 0; i < coarse->vertsCount; ++i) {
            coarse->adjMatrix[i] = coarse->edges + edgeCountListCoarse[i];
        }
        coarse->adjMatrix[coarse->vertsCount] = coarse->edges + coarse->edgesCount;
        delete[] edgeCountListCoarse;

        for (uint32_t i = 0; i < coarse->vertsCount; ++i) {
            neighbors.clear();
            for (uint32_t j = 0; j < 2; j++) {
                uint32_t d = coarse->toDense[2*i+j];
                if (d != uint32_t(-1)) {
                    for (Edge *n = adjMatrix[d]; n != adjMatrix[d+1]; ++n) {
                        neighbors.push_back({toCoarse[n->id], n->weight});
                    }
                }
            }

            std::sort(neighbors.begin(), neighbors.end());
            Edge *dest = coarse->adjMatrix[i];
            uint32_t last = -1;
            for (auto n : neighbors) {
                if (n.id != i) {
                    if (n.id != last) {
                        *dest++ = n;
                        last = n.id;
                    } else {
                        dest[-1].weight += n.weight;
                    }
                }
            }
        }
        coarse->coarseMesh();
    }

    Neighborhood getNeighborhood(uint32_t i) const {
        return Neighborhood{ adjMatrix[i], adjMatrix[i+1] };
    };

    void init(const InitFunction &initFunc) {
        Parallel::For(0, vertsCount, [this, &initFunc](size_t i) {
            initFunc(i, vertex);
        });
    }

    void smooth(const SmoothFunction &smoothFunc, uint32_t iterationCount) {
        for (uint32_t iter = 0; iter < iterationCount; ++iter) {
            Parallel::For(0, vertsCount, [this, &smoothFunc, iter](size_t i) {
                smoothFunc(i, getNeighborhood(i), vertex, iter);
            });
        }
    }

    void restrict(const RestrictFunction &restrictFunc) {
        Parallel::For(0, coarse->vertsCount, [this, &restrictFunc](size_t i) {
            uint32_t *s = coarse->toDense + 2*i;
            uint32_t *e = coarse->toDense + 2*i + (coarse->toDense[2*i+1] == uint32_t(-1) ? 1 : 2);
            Links link{s, e};
            restrictFunc(i, link, vertex, coarse->vertex);
        });
    }

    void prolong(const ProlongFunction &prolongFunc) {
        Parallel::For(0, vertsCount, [this, &prolongFunc](size_t i) {
            prolongFunc(i, toCoarse[i], vertex, coarse->vertex);
        });
    }

    T *vertex = nullptr;
    float *areas = nullptr;
    Edge **adjMatrix = nullptr;
    Edge *edges = nullptr;
    uint32_t *faces = nullptr;
    uint32_t *toDense = nullptr;
    uint32_t *toCoarse = nullptr;
    MeshHierarchy *coarse = nullptr;
    MeshHierarchy *dense = nullptr;
    uint32_t facesCount = 0;
    uint32_t edgesCount = 0;
    uint32_t vertsCount = 0;
};