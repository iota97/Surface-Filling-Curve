// Mozilla Public License Version 2.0
// ported from libigl (https://github.com/libigl/libigl).

#include "disk.h"
#include <map>
#include <deque>
#include <algorithm>
#include <array>
#include <vector>

std::set<std::array<uint32_t, 2>> Disk::edgesCut(const uint32_t *triangles, uint32_t triangleCounts, bool cut) {
    std::set<std::array<uint32_t, 2>> edgeCuts;

    uint32_t nfaces = triangleCounts;
    if (nfaces == 0) {
        return edgeCuts;
    }

    auto F = [triangles](uint32_t i, uint32_t j) { return triangles[3*i+j]; };
    std::map<std::pair<uint32_t, uint32_t>, std::vector<uint32_t> > edges;
    for (uint32_t i = 0; i < nfaces; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            uint32_t v0 = F(i, j);
            uint32_t v1 = F(i, (j + 1) % 3);
            std::pair<uint32_t, uint32_t> e;
            e.first = std::min(v0, v1);
            e.second = std::max(v0, v1);
            edges[e].push_back(i);
        }
    }
    uint32_t nedges = edges.size();

    std::vector<uint32_t> ev(2*nedges, uint32_t(-1));
    std::vector<uint32_t> ef(2*nedges, uint32_t(-1));
    std::vector<uint32_t> fe(3*nfaces, uint32_t(-1));
    auto edgeVerts = [&ev](uint32_t i, uint32_t j) -> uint32_t &{ return ev[2*i+j]; };
    auto edgeFaces = [&ef](uint32_t i, uint32_t j) -> uint32_t &{ return ef[2*i+j]; };
    auto faceEdges = [&fe](uint32_t i, uint32_t j) -> uint32_t &{ return fe[3*i+j]; };

    std::set<uint32_t> boundaryEdges;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> edgeidx;
    uint32_t idx = 0;
    for (auto it : edges) {
        edgeidx[it.first] = idx;
        edgeVerts(idx, 0) = it.first.first;
        edgeVerts(idx, 1) = it.first.second;
        edgeFaces(idx, 0) = it.second[0];
        if (it.second.size() > 1) {
            edgeFaces(idx, 1) = it.second[1];
        } else {
            edgeFaces(idx, 1) = uint32_t(-1);
            boundaryEdges.insert(idx);
        }
        idx++;
    }

    if (!cut) {
        for (uint32_t i : boundaryEdges) {
            edgeCuts.insert({ std::min(edgeVerts(i, 0), edgeVerts(i, 1)), std::max(edgeVerts(i, 0), edgeVerts(i, 1)) });
        }
        return edgeCuts;
    }

    for (uint32_t i = 0; i < nfaces; i++) {
        for (uint32_t j = 0; j < 3; j++) {
            uint32_t v0 = F(i, j);
            uint32_t v1 = F(i, (j + 1) % 3);
            std::pair<uint32_t, uint32_t> e;
            e.first = std::min(v0, v1);
            e.second = std::max(v0, v1);
            faceEdges(i, j) = edgeidx[e];
        }
    }

    bool *deleted = new bool[nfaces];
    for (uint32_t i = 0; i < nfaces; i++) {
        deleted[i] = false;
    }

    std::set<uint32_t> deletededges;

    for (uint32_t face = 0; face < nfaces; face++) {
        if (deleted[face])
            continue;
        deleted[face] = true;

        std::deque<uint32_t> processEdges;
        
        for (uint32_t i = 0; i < 3; i++) {
            uint32_t e = faceEdges(face, i);
            if (boundaryEdges.count(e))
                continue;
            uint32_t ndeleted = 0;
            if (deleted[edgeFaces(e, 0)])
                ndeleted++;
            if (deleted[edgeFaces(e, 1)])
                ndeleted++;
            if (ndeleted == 1)
                processEdges.push_back(e);
        }

        while (!processEdges.empty()) {
            uint32_t nexte = processEdges.front();
            processEdges.pop_front();
            uint32_t todelete = nfaces;
            if (!deleted[edgeFaces(nexte, 0)])
                todelete = edgeFaces(nexte, 0);
            if (!deleted[edgeFaces(nexte, 1)])
                todelete = edgeFaces(nexte, 1);
            if (todelete != nfaces) {
                deletededges.insert(nexte);
                deleted[todelete] = true;
                for (uint32_t i = 0; i < 3; i++) {
                    uint32_t e = faceEdges(todelete, i);
                    if (boundaryEdges.count(e))
                        continue;
                    uint32_t ndeleted = 0;
                    if (deleted[edgeFaces(e, 0)])
                        ndeleted++;
                    if (deleted[edgeFaces(e, 1)])
                        ndeleted++;
                    if (ndeleted == 1)
                        processEdges.push_back(e);
                }
            }
        }
    }
    delete[] deleted;

    std::vector<uint32_t> leftedges;
    for (uint32_t i = 0; i < nedges; i++) {
        if (!deletededges.count(i))
            leftedges.push_back(i);
    }

    deletededges.clear();
    std::map<uint32_t, std::vector<uint32_t> > spinevertedges;
    for (uint32_t i : leftedges) {
        spinevertedges[edgeVerts(i, 0)].push_back(i);
        spinevertedges[edgeVerts(i, 1)].push_back(i);
    }

    std::deque<uint32_t> vertsProcess;
    std::map<uint32_t, int> spinevertnbs;
    for (auto it : spinevertedges) {
        spinevertnbs[it.first] = it.second.size();
        if (it.second.size() == 1)
            vertsProcess.push_back(it.first);
    }
    while (!vertsProcess.empty()) {
        uint32_t vert = vertsProcess.front();
        vertsProcess.pop_front();
        for (uint32_t e : spinevertedges[vert]) {
            if (!deletededges.count(e)) {
                deletededges.insert(e);
                for (uint32_t j = 0; j < 2; j++) {
                    spinevertnbs[edgeVerts(e, j)]--;
                    if (spinevertnbs[edgeVerts(e, j)] == 1) {
                        vertsProcess.push_back(edgeVerts(e, j));
                    }
                }
            }
        }
    }

    for (uint32_t i : leftedges) {
        if (!deletededges.count(i)) {
            edgeCuts.insert({std::min(edgeVerts(i, 0), edgeVerts(i, 1)), std::max(edgeVerts(i, 0), edgeVerts(i, 1))});
        }
    }

    return edgeCuts;
}