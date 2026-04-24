// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "postprocessing.h"
#include "bvhpoint.h"
#include "bvhtriangle.h"
#include "parallel.h"
#include <set>

static std::vector<Vec3> resampleCurve(const std::vector<Vec3> &curve, float spacing) {
    std::vector<Vec3> newCurve;
    newCurve.push_back(curve[0]);
    for (size_t i = 1; i < curve.size() + 1; ++i) {
        float segDistance = (curve[i % curve.size()] - newCurve.back()).length();
        if (segDistance <= 0.5f*spacing) continue;
        if (segDistance >= spacing) {
            float t = spacing/segDistance;
            Vec3 pos = t*curve[i % curve.size()] + (1-t) * newCurve.back();
            newCurve.push_back(pos);
            --i;
        }
    }
    return newCurve;
}

static std::vector<Vec3> subdivideBorders(const std::vector<std::array<Vec3, 3>> &triangles, float spacing) {
    struct Edge {
        Edge (Vec3 v0, Vec3 v1) : a(v0 < v1 ? v0 : v1), b(v0 < v1 ? v1 : v0) {}
        bool operator<(const Edge &o) const {
            if (a < o.a) return true;
            if (o.a < a) return false;
            if (b < o.b) return true;
            return false;
        }
        Vec3 a, b;
    };

    std::multiset<Edge> edgeSet;
    for (const auto &t : triangles) {
        edgeSet.insert(Edge(t[0], t[1]));
        edgeSet.insert(Edge(t[1], t[2]));
        edgeSet.insert(Edge(t[2], t[0]));
    }

    std::vector<Edge> borderEdges;
    for (const auto &e : edgeSet) {
        if (edgeSet.count(e) == 1) {
            borderEdges.push_back(e);
        }
    }

    std::vector<bool> usedEdges(borderEdges.size(), false);
    std::vector<Vec3> borderVertices;
    for (size_t i = 0; i < borderEdges.size(); ++i) {
        if (usedEdges[i]) continue;
        usedEdges[i] = true;

        std::vector<Vec3> border;
        border.push_back(borderEdges[i].a);
        border.push_back(borderEdges[i].b);

        for (size_t j = 0; j < borderEdges.size(); ++j) 
        for (size_t k = 0; k < borderEdges.size(); ++k) {
            if (!usedEdges[k] && borderEdges[k].a == border[border.size()-1] && borderEdges[k].b != border[border.size()-2]) {
                border.push_back(borderEdges[k].b);
                usedEdges[k] = true;
            } else if (!usedEdges[k] && borderEdges[k].b == border[border.size()-1] && borderEdges[k].a != border[border.size()-2]) {
                border.push_back(borderEdges[k].a);
                usedEdges[k] = true;
            }
        }

        for (const auto &v : resampleCurve(border, spacing)) {
            borderVertices.push_back(v);
        }
    }

    return borderVertices;
}

void medialAxisRepulsion(std::vector<std::vector<std::array<Vec3, 2>>> &cycles, const std::vector<std::array<Vec3, 3>> &triangles, float spacing, uint32_t iterationsCount) {
    // Extra parameters
    float growSpeed = 1.25f;
    float smoothness = 1.0f;
    float strength = 0.5f;

    // Init the curves
    std::vector<std::vector<Vec3>> curves;
    for (const auto &cycle : cycles) {
        std::vector<Vec3> curve;
        for (const auto &v : cycle) {
            curve.push_back(v[0]);
        }
        curves.push_back(curve);
    }

    // Init domain BVH and boundary.
    BVHTriangle bvhTriangle(triangles);
    std::vector<Vec3> borderVertex = subdivideBorders(triangles, 0.25f*spacing);

    // Resample the curves with even spacing.
    for (auto &curve : curves) {
        curve = resampleCurve(curve, 0.25f*spacing);
    }

    for (uint32_t iter = 0; iter < iterationsCount; ++iter) {
        // Initialize the BVH with all the curve and domain points.
        std::vector<Vec3> points(borderVertex);
        for (const auto &curve : curves) {
            points.insert(points.end(), curve.begin(), curve.end());
        }
        BVHPoint bvhPoint(points);

        auto oldCurves = curves;
        size_t curveIdx = 0;
        for (auto &curve : curves) {
            // Grow the curve.
            std::vector<Vec3> curveBack(curve.size());

            Parallel::For(0, curve.size(), [&](size_t i) {
                // Create a reference frame on the curve.
                Vec3 p = curve[i];
                size_t prevInd = i ? i - 1 : curve.size() - 1;
                size_t nextInd = i < curve.size()-1 ? i + 1 : 0;
                Vec3 T = (curve[nextInd]-curve[prevInd]).normalize();
                Vec3 N = bvhTriangle.closestNormal(p);
                Vec3 B = N.cross(T).normalize();

                // Get all nearby points.
                float maxRadius = growSpeed*spacing;
                auto neighbors = bvhPoint.pointsInSphereIndex(p, maxRadius);

                // Iterate over the directions orthogonal to the curve.
                Vec3 sum(0.0f);
                float weightSum = 0.0f;

                // Find the closest points in the both the positive and negative direction.
                float distanceNegative = -maxRadius, distancePositive = maxRadius;
                float weightNegative = 1.0f, weightPositive = 1.0f;
                float deltaNegative = distanceNegative + 0.5f*spacing, deltaPositive = distancePositive - 0.5f*spacing;
                for (size_t j : neighbors) {
                    if (i == j) continue;
                    Vec3 q = points[j];
                    float distance = (q-p).length();

                    // Use the tangent distance to define closest as this filters points along the tangent directions.
                    float tangentDistance = distance*distance/B.dot(q-p);

                    if (tangentDistance > 0 && tangentDistance < distancePositive) {
                        // The spacing with the border has to be halved.
                        bool isClosestABorder = j < borderVertex.size();
                        // Only move half the distance as the other point will likely move too, unless is a border.
                        deltaPositive = isClosestABorder ? distance - 0.5f*spacing : 0.5f*(distance - spacing);
                        // Increase the weight for the boder to create a stronger energy barrier.
                        weightPositive = isClosestABorder ? 4.0f : 1.0f;
                        distancePositive = tangentDistance;
                    } else if (tangentDistance < 0 && tangentDistance > distanceNegative) {
                        bool isClosestABorder = j < borderVertex.size();
                        deltaNegative = isClosestABorder ? -distance + 0.5f*spacing : -0.5f*(distance - spacing);
                        weightNegative = isClosestABorder ? 4.0f : 1.0f;
                        distanceNegative = tangentDistance;
                    }
                }

                // Find the correct position to have the right spacing.
                Vec3 correctNegative = (p + deltaNegative * B);
                Vec3 correctPositive = (p + deltaPositive * B);

                // Average the contribution of the varius direction.
                sum += weightNegative*correctNegative;
                sum += weightPositive*correctPositive;
                weightSum += weightNegative;
                weightSum += weightPositive;


                // Update the curve with the average.
                curveBack[i] = sum/weightSum;
            });

            // Smooth the curve.
            float lambda = 1.0f/(smoothness + 1e-6);
            Parallel::For(0, curve.size(), [&](size_t i) {
                Vec3 a = lambda*curveBack[i];
                Vec3 b = curveBack[(i+1)%curve.size()];
                Vec3 c = curveBack[i ? i-1 : curve.size()-1];
                curve[i] = (a+b+c)/(lambda+2.0f);
            });

            // Reduce impact
            const auto &old = oldCurves[curveIdx++];
            Parallel::For(0, curve.size(), [&](size_t i) {
                curve[i] = strength*curve[i] + (1-strength)*old[i];
            });

            // Resample the curve.
            curve = resampleCurve(curve, 0.25f*spacing);

            // Reproject the curve on the surface.
            Parallel::For(0, curve.size(), [&](size_t i) {
                curve[i] = bvhTriangle.closestPoint(curve[i]);
            });
            curve = resampleCurve(curve, 0.25f*spacing);
        }
    }

    // Store the curves
    cycles.clear();
    for (const auto &curve : curves) {
        std::vector<std::array<Vec3, 2>> cycle;
        for (const auto &v : curve) {
            cycle.push_back({v, bvhTriangle.closestNormal(v)});
        }
        cycles.push_back(cycle);
    }
}