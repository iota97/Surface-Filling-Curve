// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "plyloader.h"
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cstring>
#include <map>
#include <cmath>


PlyLoader::PlyLoader(const char* filename) {
    load(filename);
}

void PlyLoader::load(const char* filename) {
    std::ifstream file;
    file.open(filename, std::ios_base::in);
    if (!file.is_open()) {
        return;
    }
    char line[1024] = {0};
    int vertsCount = 0;
    int faceCount = 0;
    int addedFaceCount = 0;
    bool parseData = false;
    while (file.good()) {
        file.getline(line, sizeof(line));
        if (file.eof()) break;
        int vc;
        int r, g, b;
        float x, y, z;
        if (sscanf(line, "element vertex %d", &vc) == 1) {
            vertsCount = vc;
        }
        if (sscanf(line, "element face %d", &vc) == 1) {
            faceCount = vc;
        }
        if (memcmp(line, "end_header", strlen("end_header")) == 0) {
            parseData = true;
        }

        if (parseData) {
            if (Positions.size() < size_t(vertsCount)) {
                if (sscanf(line, "%f %f %f %d %d %d", &x, &y, &z, &r, &g, &b) == 6) {
                    Positions.push_back({x, y, z});
                    Colors.push_back({uint8_t(r), uint8_t(g), uint8_t(b)});
                } else if  (sscanf(line, "%f %f %f", &x, &y, &z) == 3) {
                    Positions.push_back({x, y, z});
                    Colors.push_back({0, 0, 0});
                }
            } else if (addedFaceCount < faceCount) {
                if (sscanf(line, "%d", &vc) == 1) {
                    char *lineptr = line;
                    std::vector<uint32_t> face;
                    uint32_t counter = 0;
                    uint32_t limit = vc;
                    while (lineptr[0] != 0 && counter <= limit) {
                        while (lineptr[0] == ' ') ++lineptr;
                        if (counter && sscanf(lineptr, "%d", &vc) == 1) {
                            face.push_back(vc);
                        }
                        counter++;
                        while(lineptr[0] != ' ' && lineptr[0] != '\0') ++lineptr;
                    }
                    for (uint32_t i = 1; i < face.size()-1; ++i) {
                        Faces.push_back({face[0], face[i], face[i+1]});
                    }
                    addedFaceCount++;
                }
            }
        }
    }
    file.close();
}