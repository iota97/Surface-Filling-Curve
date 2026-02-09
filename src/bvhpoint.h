// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#pragma once
#include "mathutils.h"
#include <vector>
#include <unordered_set>

class BVHPoint {
public:
    BVHPoint(const std::vector<Vec3> &points) : points(points) {
        pointsID.resize(points.size());
        for (size_t i = 0; i < pointsID.size(); ++i) {
            pointsID[i] = i;
        }
        build();
    }

    void removePoint(size_t idx) {
        if (removedPoints.count(idx)) return;
        removedPoints.insert(idx);
        removedSinceBuild++;
        if (float(removedSinceBuild)/(pointsID.size()-removedPoints.size()+removedSinceBuild) > MIN_OCCUPANCY) {
            build();
        }
    }

    size_t anyPointsInCone(const Vec3 &origin, const Vec3 &direction, float halfAngle) const {
        float cosHalfAngle = cosf(halfAngle);
        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();
            while (true) {
                if (!coneAABBMayIntersect(origin, direction, halfAngle, nodes[idx].minAABB, nodes[idx].maxAABB)) break;
                if (nodes[idx].isLeaf()) {
                   for (size_t offset = nodes[idx].firstLeft, last = offset + nodes[idx].pointNumber; offset < last; ++offset) {
                        size_t pointIdx = pointsID[offset];
                        if (removedPoints.count(pointIdx)) continue;
                        if (pointInCone(points[pointIdx], origin, direction, cosHalfAngle)) return pointIdx;
                    }
                    break;
                }
                size_t leftIdx = nodes[idx].firstLeft;
                size_t rightIdx = leftIdx + 1;
                float leftDist2 = ((nodes[leftIdx].minAABB+nodes[leftIdx].maxAABB)*0.5f-origin).length2();
                float rightDist2 = ((nodes[rightIdx].minAABB+nodes[rightIdx].maxAABB)*0.5f-origin).length2();
                idx = leftDist2 < rightDist2 ? leftIdx : rightIdx;
                stack.push_back(leftDist2 < rightDist2 ? rightIdx : leftIdx);
            }
        }
        return size_t(-1);
    }

    size_t nearestPointIndex(const Vec3 &point) const {
        size_t nearestIdx = size_t(-1);
        float minDistance2 = std::numeric_limits<float>::infinity();

        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();
            while (true) {
                if (!sphereAABBIntersection(point, minDistance2 + EPSILON, nodes[idx].minAABB, nodes[idx].maxAABB)) break;
                if (nodes[idx].isLeaf()) {
                   for (size_t offset = nodes[idx].firstLeft, last = offset + nodes[idx].pointNumber; offset < last; ++offset) {
                        size_t pointIdx = pointsID[offset];
                        if (removedPoints.count(pointIdx)) continue;
                        float distance2 = (points[pointIdx]-point).length2();
                        if (distance2 < minDistance2) {
                            minDistance2 = distance2;
                            nearestIdx = pointIdx;
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
        return nearestIdx;
    }

    std::vector<size_t> pointsInSphereIndex(const Vec3 &center, float radius) const {
        std::vector<size_t> index;
        float radius2 = radius*radius;

        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();
            while (true) {
                if (!sphereAABBIntersection(center, radius2 + EPSILON, nodes[idx].minAABB, nodes[idx].maxAABB)) break;
                if (nodes[idx].isLeaf()) {
                   for (size_t offset = nodes[idx].firstLeft, last = offset + nodes[idx].pointNumber; offset < last; ++offset) {
                        size_t pointIdx = pointsID[offset];
                        if (removedPoints.count(pointIdx)) continue;
                        float distance2 = (points[pointIdx]-center).length2();
                        if (distance2 < radius2) {
                            index.push_back(pointIdx);
                        }
                    }
                    break;
                }
                size_t leftIdx = nodes[idx].firstLeft;
                size_t rightIdx = leftIdx + 1;

                if (pointAABBIntersection(center, nodes[leftIdx].minAABB-Vec3(radius), nodes[leftIdx].maxAABB+Vec3(radius))) {
                    idx = leftIdx;
                    stack.push_back(rightIdx);
                } else if (pointAABBIntersection(center, nodes[rightIdx].minAABB-Vec3(radius), nodes[rightIdx].maxAABB+Vec3(radius))) {
                    idx = rightIdx;
                    stack.push_back(leftIdx);
                } else {
                    float leftDist2 = pointAABBDistance2(center, nodes[leftIdx].minAABB, nodes[leftIdx].maxAABB);
                    float rightDist2 = pointAABBDistance2(center, nodes[rightIdx].minAABB, nodes[rightIdx].maxAABB);
                    idx = leftDist2 < rightDist2 ? leftIdx : rightIdx;
                    stack.push_back(leftDist2 < rightDist2 ? rightIdx : leftIdx);
                }
            }
        }
        return index;
    }

    void getAABB(Vec3 &minAABB, Vec3 &maxAABB) const {
        minAABB = nodes[0].minAABB;
        maxAABB = nodes[0].maxAABB;
    }

private:
    struct Node {
        Vec3 minAABB;
        Vec3 maxAABB;
        size_t pointNumber;
        size_t firstLeft;
        bool isLeaf() const { return pointNumber > 0; }
    };

    const size_t MAX_POINTS_PER_NODE = 32;
    const float MIN_OCCUPANCY = 0.5f;
    const float EPSILON = 1e-4f;

    std::vector<Node> nodes;
    std::vector<Vec3> points;
    std::vector<size_t> pointsID;
    std::unordered_set<size_t> removedPoints;
    size_t removedSinceBuild = 0;

    void build() {
        nodes.clear();
        nodes.push_back(Node());
        nodes[0].firstLeft = 0;
        nodes[0].pointNumber = pointsID.size();
        removedSinceBuild = 0;
        updateLastBounds();
        subdivideNodes();
    }

    void updateLastBounds() {
        nodes.back().minAABB = Vec3(std::numeric_limits<float>::infinity());
        nodes.back().maxAABB = Vec3(-std::numeric_limits<float>::infinity());

        for (size_t offset = nodes.back().firstLeft, last = offset + nodes.back().pointNumber; offset < last; ++offset) {
            size_t pointIdx = pointsID[offset];
            if (removedPoints.count(pointIdx)) continue;
            for (uint8_t axis = 0; axis < 3; ++axis) {
                nodes.back().minAABB[axis] = std::min(nodes.back().minAABB[axis], points[pointIdx][axis]);
                nodes.back().maxAABB[axis] = std::max(nodes.back().maxAABB[axis], points[pointIdx][axis]);
            }
        }
    }

    void subdivideNodes() {
        std::vector<size_t> stack{0};
        while (stack.size()) {
            size_t idx = stack.back();
            stack.pop_back();

            size_t realPointCount = 0;
            for (size_t offset = nodes[idx].firstLeft, last = offset + nodes[idx].pointNumber; offset < last; ++offset) {
                realPointCount += !removedPoints.count(pointsID[offset]);
                if (realPointCount > MAX_POINTS_PER_NODE) break;
            }

            if (realPointCount > MAX_POINTS_PER_NODE) {
                Vec3 extent = nodes[idx].maxAABB - nodes[idx].minAABB;
                uint8_t axis = 0;
                for (uint8_t i = 1; i < 3; ++i) {
                    if (extent[i] > extent[axis])
                        axis = i;
                }
                float split = nodes[idx].minAABB[axis] + 0.5f*extent[axis];

                size_t i = nodes[idx].firstLeft;
                size_t j = i + nodes[idx].pointNumber - 1;
                while (i <= j) {
                    if (points[pointsID[i]][axis] < split) {
                        i += 1;
                    } else {
                        std::swap(pointsID[i], pointsID[j]);
                        if (!j) break;
                        j -= 1;
                    }
                }

                size_t leftCount = i - nodes[idx].firstLeft;
                if (leftCount > 0 && leftCount < nodes[idx].pointNumber) {
                    Node leftNode;
                    leftNode.firstLeft = nodes[idx].firstLeft;
                    leftNode.pointNumber = leftCount;
                    nodes[idx].firstLeft = nodes.size();
                    stack.push_back(nodes.size());

                    Node rightNode;
                    rightNode.firstLeft = i;
                    rightNode.pointNumber = nodes[idx].pointNumber - leftCount;
                    nodes[idx].pointNumber = 0;
                    stack.push_back(nodes.size()+1);

                    nodes.push_back(rightNode);
                    updateLastBounds();
                    nodes.push_back(leftNode);
                    updateLastBounds();
                }
            }
        }
    }

    bool sphereAABBIntersection(const Vec3 &center, float radius2, const Vec3 &minAABB, const Vec3 &maxAABB) const {
        float dist2 = 0.0f;
        for (uint8_t axis = 0; axis < 3; ++axis) {
            float m = minAABB[axis]-center[axis], M = center[axis]-maxAABB[axis];
            dist2 += (m > 0)*m*m + (M > 0)*M*M;
        }
        return dist2 <= radius2;
    }

    bool pointAABBIntersection(const Vec3 &point, const Vec3 &minAABB, const Vec3 &maxAABB) const {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            if (point[axis] < minAABB[axis] || point[axis] > maxAABB[axis]) 
                return false;
        }
        return true;
    }

    float pointAABBDistance2(const Vec3 &point, const Vec3 &minAABB, const Vec3 &maxAABB) const {
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

    bool pointInCone(const Vec3 &point, const Vec3 &origin, const Vec3 &direction, float cosHalfAngle) const {
        Vec3 po = point-origin;
        float length = po.length();
        if (length < std::numeric_limits<float>::min()) return false;
        return direction.dot(po)/length >= cosHalfAngle;
    }

    bool coneAABBMayIntersect(const Vec3 &origin, const Vec3 &direction, float halfAngle, const Vec3 &minAABB, const Vec3 &maxAABB) const {
        Vec3 center = (minAABB+maxAABB)*0.5f;
        float radius = 0.5f*(maxAABB-minAABB).length()+EPSILON;
        Vec3 po = center - origin + direction*(radius/sinf(halfAngle));
        float length = po.length();
        if (length < std::numeric_limits<float>::epsilon()) return true;
        return direction.dot(po)/length >= cosf(halfAngle);
    }
};