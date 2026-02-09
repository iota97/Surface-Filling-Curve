// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "mathutils.h"
#include <vector>
#include <array>
#include <unordered_set>
#include <iostream>
#include <fstream>

class BVHTriangle {
public:
    BVHTriangle(const std::vector<std::array<Vec3, 3>> &triangles) {
        trisID.resize(triangles.size());
        for (size_t i = 0; i < trisID.size(); ++i) {
            trisID[i] = i;
        }
        tris.resize(triangles.size());
        for (size_t i = 0; i < tris.size(); ++i) {
            Vec3 centroid = (triangles[i][0]+triangles[i][1]+triangles[i][2])/3.0f;
            tris[i] = {triangles[i][0], triangles[i][1], triangles[i][2], centroid};
        }
        build();
    }

    Vec3 closestNormal(const Vec3 &point) {
        size_t id = 0;
        closestPoint(point, &id);
        return triangleNormal(tris[id].a, tris[id].b, tris[id].c);
    }

    Vec3 closestPoint(const Vec3 &point, size_t *triangleID = nullptr) const {
        Vec3 closest = tris[0].a;
        if (triangleID) *triangleID = 0;
        float minDistance2 = std::numeric_limits<float>::infinity();

        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();
            while (true) {
                if (!sphereAABBIntersection(point, minDistance2 + 1e-4, nodes[idx].minAABB, nodes[idx].maxAABB)) break;
                if (nodes[idx].isLeaf()) {
                   for (size_t offset = nodes[idx].firstLeft, last = offset + nodes[idx].triCount; offset < last; ++offset) {
                        size_t triIdx = trisID[offset];
                        Vec3 triClosest = closestPointOnTriangle(point, tris[triIdx].a, tris[triIdx].b, tris[triIdx].c);
                        float distance2 = (triClosest-point).length2();
                        if (distance2 < minDistance2) {
                            minDistance2 = distance2;
                            closest = triClosest;
                            if (triangleID) *triangleID = triIdx;
                        }
                    }
                    break;
                }
                size_t leftIdx = nodes[idx].firstLeft;
                size_t rightIdx = leftIdx + 1;

                if (pointAABBIntersection(point, nodes[leftIdx].minAABB, nodes[leftIdx].maxAABB)) {
                    idx = leftIdx;
                    stack.push_back(rightIdx);
                } else if (pointAABBIntersection(point, nodes[rightIdx].minAABB, nodes[rightIdx].maxAABB)) {
                    idx = rightIdx;
                    stack.push_back(leftIdx);
                } else {
                    float leftDist2 = pointAABBDistance2(point, nodes[leftIdx].minAABB, nodes[leftIdx].maxAABB);
                    float rightDist2 = pointAABBDistance2(point, nodes[rightIdx].minAABB, nodes[rightIdx].maxAABB);
                    idx = leftDist2 < rightDist2 ? leftIdx : rightIdx;
                    stack.push_back(leftDist2 < rightDist2 ? rightIdx : leftIdx);
                }
            }
        }
        return closest;
    }

    size_t intersectionCount(const Vec3 &origin, const Vec3 &dir, size_t nodeIdx = 0) const {
        if (!intersectAABB(origin, dir, nodes[nodeIdx].minAABB, nodes[nodeIdx].maxAABB)) return 0;
        size_t intersections = 0;
        if (nodes[nodeIdx].isLeaf()) {
            for (size_t i = 0; i < nodes[nodeIdx].triCount; ++i) {
                const auto &tri = tris[trisID[nodes[nodeIdx].firstLeft + i]];
                intersections += intersectTriangle(origin, dir, tri.a, tri.b, tri.c);
            }
        } else {
            intersections += intersectionCount(origin, dir, nodes[nodeIdx].firstLeft);
            intersections += intersectionCount(origin, dir, nodes[nodeIdx].firstLeft+1);
        }
        return intersections;
    }
    
    void saveToPLY(const char *path) {
        std::vector<Vec3> vertices;
        std::vector<std::array<size_t, 4>> faces;

        for (const auto &n : nodes) {
            size_t s = vertices.size();
            vertices.push_back({n.minAABB.x, n.minAABB.y, n.minAABB.z});
            vertices.push_back({n.maxAABB.x, n.minAABB.y, n.minAABB.z});
            vertices.push_back({n.minAABB.x, n.maxAABB.y, n.minAABB.z});
            vertices.push_back({n.maxAABB.x, n.maxAABB.y, n.minAABB.z});
            vertices.push_back({n.minAABB.x, n.minAABB.y, n.maxAABB.z});
            vertices.push_back({n.maxAABB.x, n.minAABB.y, n.maxAABB.z});
            vertices.push_back({n.minAABB.x, n.maxAABB.y, n.maxAABB.z});
            vertices.push_back({n.maxAABB.x, n.maxAABB.y, n.maxAABB.z});

            faces.push_back({s+0, s+1, s+3, s+2});
            faces.push_back({s+4, s+5, s+7, s+6});
            faces.push_back({s+0, s+1, s+5, s+4});
            faces.push_back({s+2, s+3, s+7, s+6});
            faces.push_back({s+1, s+3, s+7, s+5});
            faces.push_back({s+0, s+2, s+6, s+4});
        }

        std::ofstream plyFile(path);
        plyFile << "ply\nformat ascii 1.0\nelement vertex " << vertices.size() << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "element face " << faces.size() << "\nproperty list uchar uint vertex_indices\n"
            << "end_header\n";
        for (const auto &v : vertices) {
            plyFile << v.x << " " << v.y << " " << v.z << std::endl;
        }
        for (const auto &f : faces) {
            plyFile << "4 " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;
        }
    }

    void getAABB(Vec3 &minAABB, Vec3 &maxAABB) const {
        minAABB = nodes[0].minAABB;
        maxAABB = nodes[0].maxAABB;
    }

private:
    struct Node {
        Vec3 minAABB;
        Vec3 maxAABB;
        size_t triCount;
        size_t firstLeft;
        bool isLeaf() const { return triCount > 0; }
    };

    struct Tris {
        Vec3 a, b, c, centroid;
    };

    std::vector<Node> nodes;
    std::vector<Tris> tris;
    std::vector<size_t> trisID;

    void build() {
        nodes.push_back(Node());
        nodes[0].firstLeft = 0;
        nodes[0].triCount = trisID.size();
        updateLastBounds();
        subdivideNodes();
    }

    void updateLastBounds() {
        nodes.back().minAABB = Vec3(std::numeric_limits<float>::infinity());
        nodes.back().maxAABB = Vec3(-std::numeric_limits<float>::infinity());

        for (size_t offset = nodes.back().firstLeft, last = offset + nodes.back().triCount; offset < last; ++offset) {
            size_t trisIdx = trisID[offset];
            for (uint8_t axis = 0; axis < 3; ++axis) {
                nodes.back().minAABB[axis] = std::min(nodes.back().minAABB[axis], tris[trisIdx].a[axis]);
                nodes.back().minAABB[axis] = std::min(nodes.back().minAABB[axis], tris[trisIdx].b[axis]);
                nodes.back().minAABB[axis] = std::min(nodes.back().minAABB[axis], tris[trisIdx].c[axis]);
                nodes.back().maxAABB[axis] = std::max(nodes.back().maxAABB[axis], tris[trisIdx].a[axis]);
                nodes.back().maxAABB[axis] = std::max(nodes.back().maxAABB[axis], tris[trisIdx].b[axis]);
                nodes.back().maxAABB[axis] = std::max(nodes.back().maxAABB[axis], tris[trisIdx].c[axis]);
            }
        }
    }

    static bool intersectTriangle(const Vec3 &origin, const Vec3 &dir, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
        const Vec3 edge1 = b - a;
        const Vec3 edge2 = c - a;
        const Vec3 h = dir.cross(edge2);
        const float k = edge1.dot(h);
        if (k > -0.0001f && k < 0.0001f) return false;
        const float f = 1.0f / k;
        const Vec3 s = origin - a;
        const float u = f * s.dot(h);
        if (u < 0 || u > 1) return false;
        const Vec3 q = s.cross(edge1);
        const float v = f * dir.dot(q);
        if (v < 0 || u + v > 1) return false;
        const float t = f * edge2.dot(q);
        return t > 0.0001f;
    }

    static bool intersectAABB(const Vec3 &origin, const Vec3 &dir, const Vec3 &minAABB, const Vec3 &maxAABB) {
        float tx1 = (minAABB.x - origin.x) / dir.x, tx2 = (maxAABB.x - origin.x) / dir.x;
        float tmin = std::min(tx1, tx2), tmax = std::max(tx1, tx2);
        float ty1 = (minAABB.y - origin.y) / dir.y, ty2 = (maxAABB.y - origin.y) / dir.y;
        tmin = std::max(tmin, std::min(ty1, ty2)), tmax = std::min(tmax, std::max(ty1, ty2));
        float tz1 = (minAABB.z - origin.z) / dir.z, tz2 = (maxAABB.z - origin.z) / dir.z;
        tmin = std::max(tmin, std::min(tz1, tz2)), tmax = std::min(tmax, std::max(tz1, tz2));
        return tmax >= tmin && tmax > 0;
    }

    void subdivideNodes() {
        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();

            if (nodes[idx].triCount > 2) {
                Vec3 extent = nodes[idx].maxAABB - nodes[idx].minAABB;
                uint8_t axis = 0;
                for (uint8_t i = 1; i < 3; ++i) {
                    if (extent[i] > extent[axis])
                        axis = i;
                }

                float split = 0;
                for (size_t i = 0; i < nodes[idx].triCount; ++i) {
                    split += tris[trisID[nodes[idx].firstLeft+i]].centroid[axis];
                }
                split /= nodes[idx].triCount;

                size_t i = nodes[idx].firstLeft;
                size_t j = i + nodes[idx].triCount - 1;
                while (i <= j) {
                    if (tris[trisID[i]].centroid[axis] < split) {
                        i += 1;
                    } else {
                        std::swap(trisID[i], trisID[j]);
                        if (!j) break;
                        j -= 1;
                    }
                }

                size_t leftCount = i - nodes[idx].firstLeft;
                if (leftCount > 0 && leftCount < nodes[idx].triCount) {
                    Node leftNode;
                    leftNode.firstLeft = nodes[idx].firstLeft;
                    leftNode.triCount = leftCount;
                    nodes[idx].firstLeft = nodes.size();
                    stack.push_back(nodes.size());

                    Node rightNode;
                    rightNode.firstLeft = i;
                    rightNode.triCount = nodes[idx].triCount - leftCount;
                    nodes[idx].triCount = 0;
                    stack.push_back(nodes.size()+1);

                    nodes.push_back(rightNode);
                    updateLastBounds();
                    nodes.push_back(leftNode);
                    updateLastBounds();
                }
            }
        }
    }

    static Vec3 bestOfTwo(const Vec3 &p, const Vec3 &a, const Vec3 &b) {
        return (p-a).length2() < (p-b).length2() ? a : b;
    }

    static Vec3 closestPointOnTriangle(const Vec3 &p, const Vec3 &a, const Vec3 &b, const Vec3 &c) {
        Vec3 e0 = b-a, e1 = c-b, e2 = a-c;
        Vec3 p0 = p-a, p1 = p-b, p2 = p-c;
        Vec3 n = e0.cross(e1).normalize();
        Vec3 r0 = e0.cross(n).normalize(), r1 = e1.cross(n).normalize(), r2 = e2.cross(n).normalize();
        float d = p0.dot(n), d0 = p0.dot(r0), d1 = p1.dot(r1), d2 = p2.dot(r2);
        float a01 = r0.dot(r1), a02 = r0.dot(r2), a12 = r1.dot(r2);
        uint8_t flag = (d2 > 0.0f) | ((d1 > 0.0f) << 1) | ((d0 > 0.0f) << 2);
        switch (flag) {
        case 0:
            return p - d*n;
        case 1:
            return closestPointOnSegment(p2, e2, c, a);
        case 2:
            return closestPointOnSegment(p1, e1, b, c);
        case 4:
            return closestPointOnSegment(p0, e0, a, b);
        case 6:
            return a01 < 0 ? b : bestOfTwo(p, closestPointOnSegment(p1, e1, b, c), closestPointOnSegment(p0, e0, a, b));
        case 3:
            return a12 < 0 ? c : bestOfTwo(p, closestPointOnSegment(p2, e2, c, a), closestPointOnSegment(p1, e1, b, c));
        case 5:
            return a02 < 0 ? a : bestOfTwo(p, closestPointOnSegment(p0, e0, a, b), closestPointOnSegment(p2, e2, c, a));
        default:
            return a;
        }
    }

    static Vec3 closestPointOnSegment(const Vec3 &deltaStart, const Vec3 &edge, const Vec3 &start, const Vec3 &end) {
        float l = deltaStart.dot(edge);
        float s = edge.length2();
        if (l > 0 && l < s) {
            return start + l/s * edge;
        } else {
            if (l <= 0) {
                return start;
            } else {
                return end;
            }
        }
    }

    static bool pointAABBIntersection(const Vec3 &point, const Vec3 &minAABB, const Vec3 &maxAABB) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            if (point[axis] < minAABB[axis] || point[axis] > maxAABB[axis]) 
                return false;
        }
        return true;
    }

    static bool sphereAABBIntersection(const Vec3 &center, float radius2, const Vec3 &minAABB, const Vec3 &maxAABB) {
        float dist2 = 0.0f;
        for (uint8_t axis = 0; axis < 3; ++axis) {
            float m = minAABB[axis]-center[axis], M = center[axis]-maxAABB[axis];
            dist2 += (m > 0)*m*m + (M > 0)*M*M;
        }
        return dist2 <= radius2;
    }

    static float pointAABBDistance2(const Vec3 &point, const Vec3 &minAABB, const Vec3 &maxAABB) {
        Vec3 closest(point);
        for (uint8_t axis = 0; axis < 3; ++axis) {
            if (point[axis] < minAABB[axis]) {
                closest[axis] = minAABB[axis];
            } else if (point[axis] > maxAABB[axis]) {
                closest[axis] = maxAABB[axis];
            }
        }
        return (point-closest).length2();
    }
};