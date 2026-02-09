// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "stripe.h"

class Stitcher {
public:
    Stitcher(const float *vertsPosition, const float *vertsNormal, const uint32_t *triangles, uint32_t vertsCount, uint32_t facesCount, const std::set<std::array<uint32_t, 2>>& borderEdges);
    Stitcher() = default;
    
    void stitch(const float *vertsScalars, float period, bool stitch = true, bool showProgress = false);
    void repulse();

    void saveMeshToPLY(const char *path) const;
    const float *getScalars() {
        return scalars.data();
    }

    const float *getPositions() const { return position.data(); }
    const float *getNormals() const { return normals.data(); }
    const float *getScalars() const { return scalars.data(); }
    const uint32_t *getTriangle() const { return faces.data(); }
    uint32_t getVertCount() const { return vertsCount; }
    uint32_t getTriangleCount() const { return facesCount; }

private:
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

    void floodRegion();
    void cleanIsolated();
    bool connectRegions(float period, bool showProgress);
    void computeRingNumber();
    uint32_t getRingNumber(uint32_t i);
    bool connectRegion(float period, bool positives);
    void computeMask(bool positives);
    void getConnectedComponents();
    void widthenStitch(float period, bool positives, uint32_t id);

    void generateAdjacency();
    void orderAdjacency();
    Neighborhood getNeighborhood(uint32_t i) const;
    void subdivide();

    std::array<uint8_t, 3> getIDColor(uint32_t i) const;
    std::array<uint8_t, 3> getScalarColor(uint32_t i) const;

    std::vector<float> scalars;
    std::vector<uint32_t> regionID;
    std::vector<uint32_t> ringNumber;
    std::vector<uint8_t> mask;
    std::vector<uint8_t> isBorder;
    std::vector<std::array<std::array<float, 2>, 2>> distances;
    std::vector<std::array<std::array<uint32_t, 2>, 2>> prevID;
    std::vector<std::array<uint32_t, 2>> nextHop;
    std::vector<uint32_t> pathIds;
    
    uint32_t positiveRegion;
    uint32_t negativeRegion;
    std::vector<float> position;
    std::vector<float> normals;
    std::vector<uint32_t> faces;
    uint32_t vertsCount;
    uint32_t facesCount;
    uint32_t connectedRegionCount;
    uint32_t connectedRegionPositiveOnlyCount;
    std::set<std::array<uint32_t, 2>> borderEdges;

    uint32_t edgesCount;
    std::unique_ptr<Edge*[]> adjMatrix;
    std::unique_ptr<Edge[]> edges;
};