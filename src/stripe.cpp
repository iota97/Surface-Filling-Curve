// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "stripe.h"
#include "subdivide.h"
#include <cstdlib>
#include <chrono>
#include <iostream>

Stripe::Stripe(const float *vertsPositions, const float *directions, const uint32_t *triangleIndices, uint32_t vertsCount, uint32_t triangleCount, float width, Mode mode, bool cut, bool quiet)
        : width(width), mode(mode) {
    Subdivide subdividedMesh(vertsPositions, mode == Mode::Extrinsic ? nullptr : directions, mode == Mode::Extrinsic ? directions : nullptr, triangleIndices, vertsCount, triangleCount, 0.5f*width, quiet, mode != Mode::Printing && cut);
    auto startTime = std::chrono::steady_clock::now();

    vertsPositions = subdividedMesh.getPositions();
    triangleIndices = subdividedMesh.getFaces();
    vertsCount = subdividedMesh.getVertCount(); this->vertsCount = vertsCount;
    triangleCount = subdividedMesh.getFaceCount(); this->triangleCount = triangleCount;
    borderEdges = subdividedMesh.getBorderEdges();
    directions = subdividedMesh.getScalars();
    auto directionsExtrinsic = subdividedMesh.getDirections();

    position.resize(3*vertsCount);
    normal.resize(3*vertsCount);
    scalars.resize(vertsCount);
    triangle.resize(3*triangleCount);
    borders.resize(vertsCount);
    smoothBase.resize(vertsCount);
    if (mode == Mode::Extrinsic) {
        directionExtrinsic.resize(vertsCount);
    } else {
        directionAngle.resize(vertsCount);
    }

    hierarchy = std::unique_ptr<MeshHierarchy<VertexData>>(new MeshHierarchy<VertexData>(vertsPositions, triangleIndices, vertsCount, triangleCount));
    const VertexData *vertexData = hierarchy->getVertexData();

    if (mode != Mode::Extrinsic) {
        if (mode == Mode::Parallel || mode == Mode::Nearest || mode == Mode::Printing) {
            directionExtrinsic.resize(vertsCount);
            Parallel::For(0, vertsCount, [this](size_t i) {
                directionExtrinsic[i] = Vec3(0, 0, 0);
            });

            for (auto& edge : borderEdges) {
                Vec3 dir = (Vec3(vertsPositions, edge[0]) - Vec3(vertsPositions, edge[1])).normalize();
                directionExtrinsic[edge[0]] += (dir.dot(directionExtrinsic[edge[0]]) < 0 ? -1 : 1) * dir;
                directionExtrinsic[edge[1]] += (dir.dot(directionExtrinsic[edge[1]]) < 0 ? -1 : 1) * dir;
            }
        }
        if (mode == Mode::Nearest || mode == Mode::Printing) {
            optimizeNearest();
        } else {
            optimizeDirection();
        }
    }

    Parallel::For(0, vertsCount, [this, vertsPositions, vertexData, directions, directionsExtrinsic, mode](size_t i) {
        position[3*i+0] = vertsPositions[3*i+0]; position[3*i+1] = vertsPositions[3*i+1]; position[3*i+2] = vertsPositions[3*i+2];
        normal[3*+i+0] = vertexData[i].normal.x; normal[3*i+1] = vertexData[i].normal.y; normal[3*i+2] = vertexData[i].normal.z;
        borders[i] = false;

        if (mode == Mode::Extrinsic) {
            directionExtrinsic[i] = Vec3(directionsExtrinsic, i).cross(vertexData[i].normal).normalize();
        } else {
            directionAngle[i] = directions[i];
            if (hierarchy->getVertexData()[i].direction[0] == Vec3(0, 0, 0)) {
                smoothBase[i][0] = Vec3(1,0,0).orthonormalize(hierarchy->getVertexData()[i].normal);
            } else {
                smoothBase[i][0] = hierarchy->getVertexData()[i].direction[0].orthonormalize(hierarchy->getVertexData()[i].normal);
            }
            smoothBase[i][1] = hierarchy->getVertexData()[i].normal.cross(smoothBase[i][0]).normalize();
        }
    });

    for (auto &edge : borderEdges) {
        borders[edge[0]] = true;
        borders[edge[1]] = true;
    }

    Parallel::For(0, triangleCount, [this, triangleIndices](uint32_t i) {
        triangle[3*i+0] = triangleIndices[i*3+0];
        triangle[3*i+1] = triangleIndices[i*3+1];
        triangle[3*i+2] = triangleIndices[i*3+2];
    });
    if (!quiet) std::cout << "Stripes initialization (" << hierarchy->getLodsCount() << " LODs): " << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() << " ms" << std::endl;
}

void Stripe::optimize() {
    period = 2.0f*width;
    hierarchy->optimize([this](uint32_t i, VertexData *vertexData) { init(i, vertexData); }, smooth, restrict, prolong);
    auto vertexData = hierarchy->getVertexData();
    Parallel::For(0, vertsCount, [this, vertexData](size_t i) {
        scalars[i] = cosf(vertexData[i].phase[0]);
    });
}

void Stripe::saveMeshToPLY(const char *path, bool direction) const {
    if (direction) {
        hierarchy->saveMeshToPLY(path, colorDirection);
    } else {
        hierarchy->saveMeshToPLY(path, colorStripe);
    }
}

void Stripe::init(uint32_t i, VertexData *vertexData) {
    if (mode == Mode::Extrinsic) {
        vertexData[i].direction[0] = directionExtrinsic[i];
    } else if (mode == Mode::Printing) {
        if (vertexData[i].phase[0] > 2.5f * width) {
            vertexData[i].direction[0] = Vec3(cosf(directionAngle[i]), sinf(directionAngle[i]), 0).orthonormalize(vertexData[i].normal);
        } else {
            vertexData[i].direction[0] = smoothBase[i][0];
        }
    } else {
        vertexData[i].direction[0] = cosf(directionAngle[i]) * smoothBase[i][0] + sinf(directionAngle[i]) * smoothBase[i][1];
    }

    vertexData[i].phase[0] = 0.0f;
    vertexData[i].period = period;
    vertexData[i].isFrozen = borders[i];
}

void Stripe::smooth(uint32_t id, const MeshHierarchy<VertexData>::Neighborhood &neighborhood, VertexData *vertexData, uint32_t iteration) {
    uint32_t i = id;
    if (vertexData[i].isFrozen) {
        vertexData[i].phase[1-(iteration % 2)] = vertexData[i].phase[iteration % 2];
        return;
    }

    float x = 0.0f, y = 0.0f;
    for (auto &e : neighborhood) {
        uint32_t j = e.id;
        float w = e.weight;
        Vec3 p_i = vertexData[i].position;
        Vec3 p_j = vertexData[j].position;
        Vec3 p_ij = (p_i+p_j)*0.5f;

        Vec3 d_i = vertexData[i].direction[0];
        Vec3 d_j = vertexData[j].direction[0];
        float phi_j = vertexData[j].phase[(iteration % 2)];

        float a = 2.0f*float(M_PI)/vertexData[i].period * (p_ij-p_i).dot(d_i);
        float b = 2.0f*float(M_PI)/vertexData[j].period * (p_ij-p_j).dot(d_j) + phi_j;
        float angle = d_i.dot(d_j) > 0.0f ? b - a : -b - a;

        x += w*cosf(angle);
        y += w*sinf(angle);
    }
    vertexData[i].phase[1-(iteration % 2)] = atan2f(y, x);

}

void Stripe::restrict(uint32_t id, const MeshHierarchy<VertexData>::Links &links, VertexData *vertexData, VertexData *vertexDataCoarse) {
    Vec3 direction(0, 0, 0);

    float x = 0.0f, y = 0.0f;
    float count = 0.0f;
    vertexDataCoarse[id].period = 0.0f;
    bool isFrozen = false;
    for (auto j : links) {
        Vec3 d_j = vertexData[j].direction[0];
        direction = direction + (d_j.dot(direction) > 0.0f ? d_j : -d_j);
    }
    direction = direction.orthonormalize(vertexDataCoarse[id].normal);
    vertexDataCoarse[id].direction[0] = direction;

    for (auto j : links) {
        Vec3 d_j = vertexData[j].direction[0];
        vertexDataCoarse[id].period += vertexData[j].period;
        count++;
        if (vertexData[j].isFrozen) {
            isFrozen = true;
            Vec3 p_i = vertexDataCoarse[id].position;
            Vec3 p_j = vertexData[j].position;
            Vec3 d_i = vertexDataCoarse[id].direction[0];
            float phi_j = vertexData[j].phase[0];
            float b = 2.0f*float(M_PI)/vertexData[j].period * (p_i-p_j).dot(d_j) + phi_j;
            float angle = d_i.dot(d_j) > 0.0f ? b: -b;

            x += cosf(angle);
            y += sinf(angle);
        }
    }

    vertexDataCoarse[id].period /= count;
    vertexDataCoarse[id].phase[0] = atan2f(y, x);
    vertexDataCoarse[id].isFrozen = isFrozen;
}

void Stripe::prolong(uint32_t id, uint32_t coarse_id, VertexData *vertexData, VertexData *vertexDataCoarse) {
    if (vertexData[id].isFrozen)
        return;

    Vec3 p_i = vertexData[id].position;
    Vec3 p_j = vertexDataCoarse[coarse_id].position;

    Vec3 d_i = vertexData[id].direction[0];
    Vec3 d_j = vertexDataCoarse[coarse_id].direction[0];
    float phi_j = vertexDataCoarse[coarse_id].phase[0];

    float a = 2.0f*float(M_PI)/vertexData[id].period * (p_j-p_i).dot(d_i);
    float b = phi_j;
    float angle = d_i.dot(d_j) > 0.0f ? b - a : -b - a;

    vertexData[id].phase[0] = angle;
}

std::array<uint8_t, 3> Stripe::colorDirection(uint32_t id, VertexData *vertexData) {
    Vec3 dir = vertexData[id].normal.cross(vertexData[id].direction[0]).normalize();
    uint8_t r = 255*(dir.x*0.5f+0.5f);
    uint8_t g = 255*(dir.y*0.5f+0.5f);
    uint8_t b = 255*(dir.z*0.5f+0.5f);
    return std::array<uint8_t, 3>{r, g, b};
}

std::array<uint8_t, 3> Stripe::colorStripe(uint32_t id, VertexData* vertexData) {
    float t = std::clamp(float(cos(vertexData[id].phase[0])) * 0.5f + 0.5f, 0.0f, 1.0f);

    static const float c0[3]{ 0.2777273272234177, 0.005407344544966578, 0.3340998053353061 };
    static const float c1[3]{ 0.1050930431085774, 1.404613529898575, 1.384590162594685 };
    static const float c2[3]{ -0.3308618287255563, 0.214847559468213, 0.09509516302823659 };
    static const float c3[3]{ -4.634230498983486, -5.799100973351585, -19.33244095627987 };
    static const float c4[3]{ 6.228269936347081, 14.17993336680509, 56.69055260068105 };
    static const float c5[3]{ 4.776384997670288, -13.74514537774601, -65.35303263337234 };
    static const float c6[3]{ -5.435455855934631, 4.645852612178535, 26.3124352495832 };

    uint8_t r = 255 * (c0[0] + t * (c1[0] + t * (c2[0] + t * (c3[0] + t * (c4[0] + t * (c5[0] + t * c6[0]))))));
    uint8_t g = 255 * (c0[1] + t * (c1[1] + t * (c2[1] + t * (c3[1] + t * (c4[1] + t * (c5[1] + t * c6[1]))))));
    uint8_t b = 255 * (c0[2] + t * (c1[2] + t * (c2[2] + t * (c3[2] + t * (c4[2] + t * (c5[2] + t * c6[2]))))));

    return std::array<uint8_t, 3>{r, g, b};
}

void Stripe::saveDirectionsToPLY(const char *path, float width, float lenght) const {
    std::vector<Vec3> plyVerts;
    std::vector<std::array<uint32_t, 4>> plyQuad;
    uint32_t curr = 0;
    for (uint32_t i = 0; i < vertsCount; ++i) {
        if (i % 16) continue;
        Vec3 p = hierarchy->getVertexData()[i].position;
        Vec3 n = hierarchy->getVertexData()[i].normal;
        Vec3 b = hierarchy->getVertexData()[i].direction[0];
        Vec3 t = n.cross(b).normalize();

        plyVerts.push_back(p + 0.5f*width*n + width*b + lenght*t);
        plyVerts.push_back(p - 0.5f*width*n + width*b + lenght*t);
        plyVerts.push_back(p - 0.5f*width*n - width*b + lenght*t);
        plyVerts.push_back(p + 0.5f*width*n - width*b + lenght*t);
        plyVerts.push_back(p + 0.5f*width*n + width*b - lenght*t);
        plyVerts.push_back(p - 0.5f*width*n + width*b - lenght*t);
        plyVerts.push_back(p - 0.5f*width*n - width*b - lenght*t);
        plyVerts.push_back(p + 0.5f*width*n - width*b - lenght*t);

        plyQuad.push_back({curr, curr+1, curr+5, curr+4});
        plyQuad.push_back({curr+2, curr+3, curr+7, curr+6});
        plyQuad.push_back({curr, curr+3, curr+7, curr+4});
        plyQuad.push_back({curr+2, curr+1, curr+5, curr+6});
        curr += 8;
    }
        
    std::ofstream plyFile(path);
    plyFile << "ply\nformat ascii 1.0\nelement vertex " << plyVerts.size() << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "element face " << plyQuad.size() << "\nproperty list uchar uint vertex_indices\n"
            << "end_header\n";
        
    for (uint32_t i = 0; i < plyVerts.size(); ++i) {
        plyFile << plyVerts[i].x << " " << plyVerts[i].y << " " << plyVerts[i].z<< std::endl;
    }

    for (auto &f: plyQuad) {
        plyFile << "4 " << f[0] << " " << f[1] << " " << f[2] << " " << f[3] << std::endl;
    }
}

void Stripe::optimizeDirection() {
    hierarchy->optimize([this](uint32_t i, VertexData* vertexData) { initDirection(i, vertexData); }, smoothDirection, restrictDirection, prolongDirection, 32);
}

void Stripe::initDirection(uint32_t id, VertexData *vertexData) {
    if (mode != Mode::Parallel || directionExtrinsic[id].length() == 0) {
        vertexData[id].direction[0] = Vec3(1, 0, 0).orthonormalize(vertexData[id].normal);
        vertexData[id].isFrozen = false;
    }
    else {
        vertexData[id].direction[0] = directionExtrinsic[id].cross(vertexData[id].normal).normalize();
        vertexData[id].isFrozen = true;
    }
}

void Stripe::smoothDirection(uint32_t id, const MeshHierarchy<VertexData>::Neighborhood &neighborhood, VertexData *vertexData, uint32_t iteration) {
    if (vertexData[id].isFrozen) {
        vertexData[id].direction[1 - (iteration % 2)] = vertexData[id].direction[iteration % 2];
        return;
    }
    
    Vec3 dir(0.0f, 0.0f, 0.0f);
    for (auto &e : neighborhood) {
        Vec3 d_j = e.weight * vertexData[e.id].direction[iteration % 2];
        dir += (dir.dot(d_j) < 0 ? -1 : 1) * d_j;
    }
    vertexData[id].direction[1-(iteration % 2)] = dir.orthonormalize(vertexData[id].normal);
}

void Stripe::restrictDirection(uint32_t id, const MeshHierarchy<VertexData>::Links &links, VertexData *vertexData, VertexData *vertexDataCoarse) {
    Vec3 dir(0.0f, 0.0f, 0.0f);
    bool frozen = false;
    for (auto j : links) {
        if (vertexData[j].isFrozen) {
            frozen = true;
            Vec3 d_j = vertexData[j].direction[0];
            dir += (dir.dot(d_j) < 0 ? -1 : 1) * d_j;
        }
    }
    vertexDataCoarse[id].direction[0] = dir.orthonormalize(vertexDataCoarse[id].normal);
    vertexDataCoarse[id].isFrozen = frozen;
}

void Stripe::prolongDirection(uint32_t id, uint32_t coarse_id, VertexData *vertexData, VertexData *vertexDataCoarse) {
    if (!vertexData[id].isFrozen)
        vertexData[id].direction[0] = vertexDataCoarse[coarse_id].direction[0].orthonormalize(vertexData[id].normal);
}

void Stripe::saveCutsToPLY(const char *path) const {
    std::vector<Vec3> plyVerts;
    std::vector<std::array<uint32_t, 2>> plyEdges;
    uint32_t curr = 0;
    for (auto &edge : borderEdges) {
        plyVerts.push_back(Vec3(position.data(), edge[0]));
        plyVerts.push_back(Vec3(position.data(), edge[1]));
        plyEdges.push_back({curr, curr+1});
        curr += 2;
    }

    std::ofstream plyFile(path);
    plyFile << "ply\nformat ascii 1.0\nelement vertex " << plyVerts.size() << "\n"
            << "property float x\nproperty float y\nproperty float z\n"
            << "element edge " << plyEdges.size() << "\nproperty int vertex1\nproperty int vertex2\n"
            << "end_header\n";
        
    for (uint32_t i = 0; i < plyVerts.size(); ++i) {
        plyFile << plyVerts[i].x << " " << plyVerts[i].y << " " << plyVerts[i].z << std::endl;
    }

    for (auto &e: plyEdges) {
        plyFile << e[0] << " " << e[1] << std::endl;
    }
}

std::vector<std::array<Vec3, 2>> Stripe::getDirectionField() const {
    std::vector<std::array<Vec3, 2>> directionField(vertsCount);
    Parallel::For(0, vertsCount, [this, &directionField](uint32_t i) {
        directionField[i][0] = hierarchy->getVertexData()[i].position;
        directionField[i][1] = hierarchy->getVertexData()[i].normal.cross(hierarchy->getVertexData()[i].direction[0]).normalize();
    });
    return directionField;
}

void Stripe::optimizeNearest() {
    hierarchy->optimize([this](uint32_t i, VertexData* vertexData) { initNearest(i, vertexData); }, smoothNearest, restrictNearest, prolongNearest, 32);
}

void Stripe::initNearest(uint32_t id, VertexData* vertexData) {
    if (directionExtrinsic[id].length() == 0) {
        vertexData[id].direction[0] = Vec3(0, 0, 0);
        vertexData[id].phase[0] = std::numeric_limits<float>::infinity();
    }
    else {
        vertexData[id].direction[0] = directionExtrinsic[id].cross(vertexData[id].normal).normalize();
        vertexData[id].phase[0] = 0.0f;
    }
}

void Stripe::smoothNearest(uint32_t id, const MeshHierarchy<VertexData>::Neighborhood& neighborhood, VertexData* vertexData, uint32_t iteration) {
    vertexData[id].phase[1 - (iteration % 2)] = vertexData[id].phase[iteration % 2];
    vertexData[id].direction[1 - (iteration % 2)] = vertexData[id].direction[iteration % 2];

    for (auto& e : neighborhood) {
        float d_ij = (vertexData[id].position - vertexData[e.id].position).length();
        if (d_ij + vertexData[e.id].phase[iteration % 2] < vertexData[id].phase[1 - (iteration % 2)]) {
            vertexData[id].phase[1 - (iteration % 2)] = d_ij + vertexData[e.id].phase[iteration % 2];
            vertexData[id].direction[1 - (iteration % 2)] = vertexData[e.id].direction[iteration % 2].orthonormalize(vertexData[id].normal);
        }
    }
}

void Stripe::restrictNearest(uint32_t id, const MeshHierarchy<VertexData>::Links& links, VertexData* vertexData, VertexData* vertexDataCoarse) {
    vertexDataCoarse[id].direction[0] = Vec3(0, 0, 0);
    vertexDataCoarse[id].phase[0] = std::numeric_limits<float>::infinity();

    for (auto j : links) { 
        float d_ij = (vertexDataCoarse[id].position - vertexData[j].position).length();
        if (d_ij + vertexData[j].phase[0] < vertexDataCoarse[id].phase[0]) {
            vertexDataCoarse[id].phase[0] = d_ij + vertexData[j].phase[0];
            vertexDataCoarse[id].direction[0] = vertexData[j].direction[0].orthonormalize(vertexDataCoarse[id].normal);
        }
    }
}

void Stripe::prolongNearest(uint32_t id, uint32_t coarse_id, VertexData* vertexData, VertexData* vertexDataCoarse) {
    float d_ij = (vertexData[id].position - vertexDataCoarse[coarse_id].position).length();
    if (d_ij + vertexDataCoarse[coarse_id].phase[0] < vertexData[id].phase[0]) {
        vertexData[id].phase[0] = d_ij + vertexDataCoarse[coarse_id].phase[0];
        vertexData[id].direction[0] = vertexDataCoarse[coarse_id].direction[0].orthonormalize(vertexData[id].normal);
    }
}