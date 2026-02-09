// MIT License
// Copyright 2026 Giovanni Cocco and Inria

#include "objloader.h"
#include "plyloader.h"
#include "curve.h"

int main(int argc, char *argv[]) {
    if (argc < 3) {
        std::cout << "Usage: " << argv[0] << " input_mesh.(ply|obj) curve_spacing [-e|-i|-p|-n] [-r] [-q] [-c curve.ply] [DEBUG_OPTIONS]\n\n";
        std::cout << "input_mesh.(ply|obj): Input mesh to be processed. For PLY files, only the ASCII format is supported.\n";
        std::cout << "curve_spacing: Target distance between parallel segments of the curve.\n\n";
        std::cout << "[-e]: Initialize the direction field using vertex colors as 3D directions. [Default]\n";
        std::cout << "[-i]: Initialize the direction field using vertex color luminosity as an angle (0 to 180 degrees) from a smooth direction field.\n";
        std::cout << "[-p]: Initialize the direction field using vertex color luminosity as an angle (0 to 180 degrees) from a smooth direction field parallel to the mesh borders.\n";
        std::cout << "[-n]: Initialize the direction field using vertex color luminosity as an angle (0 to 180 degrees) from the direction of the closest mesh borders.  \n";
        std::cout << "[-r]: Uniformly resample the curve. This does not guarantee that the curve remains on the surface.\n";
        std::cout << "[-R] Repulse the curve and greatly improve spacing [Experimental].\n";
        std::cout << "[-q]: Suppress console output.\n";
        std::cout << "[-c curve.ply]: Export the curve as a polyline.\n\n";
        std::cout << "[DEBUG_OPTIONS]: the following options were used for debugging.\n";
        std::cout << "[-x]: Disables the stitching step.\n";
        std::cout << "[-l]: Initialize the direction field using vertex color luminosity as an angle (0 to 180 degrees) for 3D printing. This option works only on layers like geometries, not on general ones where multiple curves may be produced.\n";
        std::cout << "[-C thick_curve.ply]: Export the curve as a solid mesh.\n";
        std::cout << "[-s scalars.ply]: Export the scalar field as a vertex-colored mesh.\n";
        std::cout << "[-P stripes.ply]: Export the stripe pattern as a vertex-colored mesh.\n";
        std::cout << "[-d directions.ply]: Export the direction field as a vertex-colored mesh.\n";
        std::cout << "[-D thick_directions.ply]: Export the direction field as a solid mesh.\n";
        std::cout << "[-g disk_cut.ply]: Export the edge cuts that convert the mesh into a topological disk.\n";
        std::cout << "[-o medial_axis.obj]: Export the OBJ for Surface-Filling Curve Flows via Implicit Medial Axes.\n";
        return 1;
    }

    bool quiet = false, resample = false, repulse = false, stitch = true;
    Stripe::Mode mode = Stripe::Mode::Extrinsic;
    float width = std::atof(argv[2]);
    const char *curvePath = nullptr;
    const char *thickCurvePath = nullptr;
    const char *scalarsPath = nullptr;
    const char *thickDirectionsPath = nullptr;
    const char *directionsPath = nullptr;
    const char *graphCutPath = nullptr;
    const char* objPath = nullptr;
    const char* stripePath = nullptr;

    bool parse_c = false, parse_C = false, parse_s = false, parse_d = false, parse_D = false, parse_g = false, parse_o = false, parse_P = false; 
    for (int i = 1; i < argc; ++i) {
		if (!strcmp(argv[i], "-q")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            quiet = true;
        } else if (!strcmp(argv[i], "-x")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            stitch = false;
        } else if (!strcmp(argv[i], "-R")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            repulse = true;
        } else if (!strcmp(argv[i], "-e")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            mode = Stripe::Mode::Extrinsic;
        } else if (!strcmp(argv[i], "-i")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            mode = Stripe::Mode::Instrinic;
        } else if (!strcmp(argv[i], "-p")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            mode = Stripe::Mode::Parallel;
        } else if (!strcmp(argv[i], "-n")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            mode = Stripe::Mode::Nearest;
        } else if (!strcmp(argv[i], "-l")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            mode = Stripe::Mode::Printing;
        } else if (!strcmp(argv[i], "-r")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            resample = true;
        } else if (!strcmp(argv[i], "-c")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_c = true;
        } else if (!strcmp(argv[i], "-C")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_C = true;
        } else if (!strcmp(argv[i], "-s")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_s = true;
        } else if (!strcmp(argv[i], "-d")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_d = true;
        } else if (!strcmp(argv[i], "-D")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_D = true;
        } else if (!strcmp(argv[i], "-g")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_g = true;
        } else if (!strcmp(argv[i], "-S")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
        } else if (!strcmp(argv[i], "-o")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_o = true;
        } else if (!strcmp(argv[i], "-P")) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            parse_P = true;
        } else if (parse_c) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            curvePath = argv[i];
        } else if (parse_C) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            thickCurvePath = argv[i];
        } else if (parse_s) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            scalarsPath = argv[i];
        } else if (parse_d) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            directionsPath = argv[i];
        } else if (parse_D) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            thickDirectionsPath = argv[i];
        } else if (parse_g) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            graphCutPath = argv[i];
        } else if (parse_o) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            objPath = argv[i];
        } else if (parse_P) {
            parse_c = parse_C = parse_s = parse_d = parse_D = parse_g  = parse_o = parse_P = false;
            stripePath = argv[i];
        }
    }
    
    ObjLoader objMesh;
    PlyLoader plyMesh;
    const float *positions = nullptr;
    const uint32_t *triangles = nullptr;
    std::vector<float> directions;
    uint32_t vertCount = 0;
    uint32_t triCount = 0;
    auto startTime = std::chrono::steady_clock::now();
    if (strlen(argv[1]) >= strlen(".obj") && !memcmp(argv[1] + strlen(argv[1]) - strlen(".obj"), ".obj", strlen(".obj"))) {
        objMesh.load(argv[1]);
        if (!quiet) std::cout << "Obj loading (" << objMesh.getVertCount() << " Verts): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() << " ms" << std::endl;
        positions = objMesh.getPositions();
        triangles = objMesh.getFaces();
        vertCount = objMesh.getVertCount();
        triCount = objMesh.getFaceCount();
    } else {
        plyMesh.load(argv[1]);
        if (!quiet) std::cout << "Ply loading (" << plyMesh.getVertCount() << " Verts): "
            << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() << " ms" << std::endl;
        positions = plyMesh.getPositions();
        triangles = plyMesh.getFaces();
        vertCount = plyMesh.getVertCount();
        triCount = plyMesh.getFaceCount();
        directions.resize((mode == Stripe::Mode::Extrinsic ? 3 : 1) * vertCount);
        Parallel::For(0, vertCount, [&directions, &plyMesh, mode](uint32_t i) {
            if (mode == Stripe::Mode::Extrinsic) {
                directions[3*i+0] = 2*(plyMesh.getColors()[3*i+0]/255.0)-1; 
                directions[3*i+1] = 2*(plyMesh.getColors()[3*i+1]/255.0)-1;
                directions[3*i+2] = 2*(plyMesh.getColors()[3*i+2]/255.0)-1;
            } else {
                directions[i] = M_PI*(plyMesh.getColors()[3*i+0]+plyMesh.getColors()[3*i+1]+plyMesh.getColors()[3*i+2])/(3*255.0f);
            }
        });
    }

    Curve curve(positions, vertCount, triangles, triCount, width, directions.empty() ? nullptr : directions.data(), mode, resample, repulse, stitch, quiet);

    startTime = std::chrono::steady_clock::now();
    if (curvePath) curve.saveCurveToPLY(curvePath);
    if (thickCurvePath) curve.saveThickCurveToPLY(thickCurvePath);
    if (scalarsPath) curve.saveScalarsToPLY(scalarsPath);
    if (directionsPath) curve.saveDirectionsToPLY(directionsPath);
    if (thickDirectionsPath) curve.saveThickDirectionsToPLY(thickDirectionsPath);
    if (graphCutPath) curve.saveCutsToPLY(graphCutPath);
    if (objPath) curve.saveMeshAndCurveToOBJ(objPath);
    if (stripePath) curve.saveStripeToPLY(stripePath);

    if (!quiet) std::cout << "--------------\nSaving: " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count() << " ms\n";

    return 0;
}