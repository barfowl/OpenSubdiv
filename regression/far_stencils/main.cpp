//
//   Copyright 2018 Dreamworks
//
//   Licensed under the Apache License, Version 2.0 (the "Apache License")
//   with the following modification; you may not use this file except in
//   compliance with the Apache License and the following modification to it:
//   Section 6. Trademarks. is deleted and replaced with:
//
//   6. Trademarks. This License does not grant permission to use the trade
//      names, trademarks, service marks, or product names of the Licensor
//      and its affiliates, except as required to comply with Section 4(c) of
//      the License and to reproduce the content of the NOTICE file.
//
//   You may obtain a copy of the Apache License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the Apache License with the above modification is
//   distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
//   KIND, either express or implied. See the Apache License for the specific
//   language governing permissions and limitations under the Apache License.
//

#include "init_shapes_all.h"

#include "../../regression/common/far_utils.h"

#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/stencilTable.h>
#include <opensubdiv/far/stencilTableFactory.h>
#include <opensubdiv/far/patchTable.h>
#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/patchMap.h>
#include <opensubdiv/far/ptexIndices.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cstdio>

//
// Regression testing for Far stencil tables...
//

using namespace OpenSubdiv;
using namespace OpenSubdiv::OPENSUBDIV_VERSION;

using Far::Index;
using Far::LocalIndex;
using Far::ConstIndexArray;
using Far::ConstLocalIndexArray;


//
//  Command line arguments and their parsing:
//
namespace {
    bool readString(const char *fileName, std::string& fileString) {
        std::ifstream ifs(fileName);
        if (ifs) {
            std::stringstream ss;
            ss << ifs.rdbuf();
            ifs.close();

            fileString = ss.str();
            return true;
        }
        return false;
    }
}

class Args {
public:
    typedef Far::PatchTableFactory::Options::EndCapType IrregularBasis;
    typedef Sdc::Options::FVarLinearInterpolation       FVarInterpType;

    Args() { initialize(); }
    Args(int argc, char **argv);
    ~Args() { }

public:
    //  options related to the testing:
    bool testVertex;
    bool testVarying;
    bool testFVar;
    bool testPatch;
    bool testLimit;
    bool limitStencilArg;
    bool limitPatchArg;

    //  options related to the approximation of shapes:
    bool           refineAdaptive;
    int            refineDepth;
    int            refineSecond;
    IrregularBasis irregBasis;
    bool           infSharp;
    bool           uvLegacy;
    bool           uvInterpSet;
    FVarInterpType uvInterp;

    //  options related to given or specified shapes:
    int                    shapeCount;
    Scheme                 shapeScheme;
    std::string            shapeName;
    std::vector<ShapeDesc> shapes;

private:
    void initialize() {
        testVertex      = true;
        testVarying     = true;
        testFVar        = true;
        testPatch       = true;
        testLimit       = true;
        limitStencilArg = true;
        limitPatchArg   = true;

        refineAdaptive = true;
        refineDepth    = 3;
        refineSecond   = 15;
        irregBasis     = Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS;
        infSharp       = false;
        uvLegacy       = false;
        uvInterpSet    = true;
        uvInterp       = Sdc::Options::FVAR_LINEAR_CORNERS_ONLY;

        shapeCount  = 0;
        shapeScheme = kCatmark;
    }
};

Args::Args(int argc, char **argv) {

    initialize();

    std::string fileString;

    for (int i = 1; i < argc; ++i) {
        char * arg = argv[i];

        //  Options related to input .obj files:
        if (strstr(arg, ".obj")) {
            if (readString(arg, fileString)) {
                //  Use the scheme declared at the time so that multiple shape/scheme
                //  pairs can be specified
                shapes.push_back(ShapeDesc(arg, fileString.c_str(), shapeScheme));
            } else {
                printf("Unable to open/read .obj file '%s'\n", arg);
                exit(0);
            }
        } else if (!strcmp(arg, "-bilinear")) {
            shapeScheme = kBilinear;
        } else if (!strcmp(arg, "-catmark")) {
            shapeScheme = kCatmark;
        } else if (!strcmp(arg, "-loop")) {
            shapeScheme = kLoop;

        //  Options affecting the shapes to be included:
        } else if (!strcmp(arg, "-shape")) {
            if (++i < argc) shapeName = std::string(arg);
        } else if (!strcmp(arg, "-count")) {
            if (++i < argc) shapeCount = atoi(argv[i]);

        //  Options affecting the tests to be applied:
        } else if (!strcmp(arg, "-novtx")) {
            testVertex = false;
        } else if (!strcmp(arg, "-novar")) {
            testVarying = false;
        } else if (!strcmp(arg, "-nofvar")) {
            testFVar = false;
        } else if (!strcmp(arg, "-nopatch")) {
            testPatch = false;
        } else if (!strcmp(arg, "-nolimit")) {
            testLimit = false;
        } else if (!strcmp(arg, "-limit00")) {
            limitStencilArg = false;
            limitPatchArg   = false;
        } else if (!strcmp(arg, "-limit10")) {
            limitStencilArg = true;
            limitPatchArg   = false;
        } else if (!strcmp(arg, "-limit01")) {
            limitStencilArg = false;
            limitPatchArg   = true;
        } else if (!strcmp(arg, "-limit11")) {
            limitStencilArg = true;
            limitPatchArg   = true;

        //  Options approximating the limit surface:
        } else if (!strcmp(arg, "-a")) {
            refineAdaptive = true;
        } else if (!strcmp(arg, "-u")) {
            refineAdaptive = false;
        } else if (!strcmp(arg, "-l")) {
            if (++i < argc) refineDepth = atoi(argv[i]);
        } else if (!strcmp(arg, "-l2")) {
            if (++i < argc) refineSecond = atoi(argv[i]);
        } else if (!strcmp(arg, "-linear")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_BILINEAR_BASIS;
        } else if (!strcmp(arg, "-regular")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_BSPLINE_BASIS;
        } else if (!strcmp(arg, "-gregory")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS;
        } else if (!strcmp(arg, "-inf")) {
            infSharp = true;

        //  Options affecting face-varying UVs:
        } else if (!strcmp(arg, "-uvlegacy")) {
            uvLegacy = true;
        } else if (!strcmp(argv[i], "-uvint")) {
            if (++i < argc) {
                int argint = atoi(argv[i]);
                if (argint == -1) {
                    uvInterpSet = false;
                } else if ((argint >= 0) && (argint <= 5)) {
                    uvInterpSet = true;
                    uvInterp    = (FVarInterpType) argint;
                }
            }
        } else {
            printf("Ignoring unrecognized argument '%s'\n", arg);
        }
    }
}

//
//  Simple primvar data struct with required interface for interpolation:
//
struct Coord3 {
    Coord3() { }
    Coord3(float x, float y, float z) { _xyz[0] = x, _xyz[1] = y, _xyz[2] = z; }

    void Clear() { _xyz[0] = _xyz[1] = _xyz[2] = 0.0f; }

    void AddWithWeight(Coord3 const & src, float weight) {
        _xyz[0] += weight * src._xyz[0];
        _xyz[1] += weight * src._xyz[1];
        _xyz[2] += weight * src._xyz[2];
    }

    float const * Coords() const { return &_xyz[0]; }

private:
    float _xyz[3];
};

typedef std::vector<Coord3> Coord3Vector;

int
compareCoord3Arrays(int size, Coord3 const* a, Coord3 const* b, float tol) {

    int nDiffs = 0;
    for (int i = 0; i < size; ++i) {
        float const * aCoords = a[i].Coords();
        float const * bCoords = b[i].Coords();

        if ((std::abs(aCoords[0] - bCoords[0]) > tol) ||
            (std::abs(aCoords[1] - bCoords[1]) > tol) ||
            (std::abs(aCoords[2] - bCoords[2]) > tol)) {
//printf("Diff:  a(%12.6f %12.6f %12.6f)\n", aCoords[0], aCoords[1], aCoords[2]);
//printf("    != b(%12.6f %12.6f %12.6f)\n", bCoords[0], bCoords[1], bCoords[2]);
            ++ nDiffs;
        }
    }
    return nDiffs;
}

int
compareCoord3Vectors(Coord3Vector const& a, Coord3Vector const& b, float tol) {
    assert(a.size() == b.size());

    return compareCoord3Arrays((int)a.size(), &a[0], &b[0], tol);
}

int
main(int argc, char **argv) {

    Args args(argc, argv);

    //
    //  Options controlling the tests and shapes -- move more of these variables
    //  into members of the command line Args class parsed above...
    //
    //  Options related to testing:
    bool  testRefineStencils = true;
    bool  testPatchStencils  = args.testPatch;
    bool  testLimitStencils  = args.testLimit;

    bool  testVertexData  = args.testVertex;
    bool  testVaryingData = args.testVarying;
    bool  testFVarData    = args.testFVar;
#ifdef _OSD_PRE_3_4
    printf("WARNING:  pre-3.4 version of OpenSubdiv -- varying and other options disabled...\n");
    testVaryingData = false;
#endif

    bool  printOptions  = true;
    bool  printTestInfo = true;
    bool  printSummary  = true;

    //  Options related to shape representation and accuracy:
    bool  useInfSharp     = args.infSharp;
    bool  useSingleCrease = false;
    bool  legacyCorners   = true;

    float xyzTol = 1.0e-5;
    float uvTol  = 1.0e-5;
    int   nDiffs = 0;

    if (printOptions) {
        char const * boolStrings[2]       = { "false", "true" };
        char const * irregBasisStrings[4] = { "None", "Linear", "Regular", "Gregory" };
        char const * fvarInterpStrings[6] = { "None", "Corners Only", "Corners Plus1",
                                              "Corners Plus2", "Boundaries", "All" };

        printf("\n");
        printf("Test options:\n");
        printf("  - Vertex stencils        = %d\n", testVertexData);
        printf("  - Varying stencils       = %d\n", testVaryingData);
        printf("  - Face-Varying stencils  = %d\n", testFVarData);
        printf("  - Refined point stencils = %d\n", testRefineStencils);
        printf("  - Patch point stencils   = %d\n", testPatchStencils);
        printf("  - Limit point stencils   = %d\n", testLimitStencils);
        printf("  -   StencilTable arg     = %d\n", args.limitStencilArg);
        printf("  -   PatchTable arg       = %d\n", args.limitPatchArg);

        printf("\n");
        printf("Shape options:\n");
        printf("  - refine adaptive   = %d\n", args.refineAdaptive);
        printf("  - refinement depth  = %d\n", args.refineDepth);
        if (args.refineSecond < args.refineDepth) {
            printf("  - secondary level   = %d\n", args.refineSecond);
        }
        printf("  - irregular basis   = %s\n", irregBasisStrings[args.irregBasis]);
        printf("  - inf-sharp patches = %s\n", boolStrings[useInfSharp]);
        printf("  - smooth corners    = %s\n", boolStrings[!legacyCorners]);
        printf("  - UV legacy linear  = %s\n", boolStrings[args.uvLegacy]);
        printf("  - UV interp set     = %s\n", boolStrings[args.uvInterpSet]);
        printf("  - UV interp type    = %s\n", !args.uvInterpSet ? "N/A" :
                                               fvarInterpStrings[args.uvInterp]);
        if (args.uvLegacy && args.uvInterpSet) {
            printf("WARNING - conflicting FVar options, legacy option ignored...\n");
        }
        printf("\n");
    }

    bool allLevelsInStencils = args.refineAdaptive;
    bool allLevelsInPatches  = args.refineAdaptive;

    //
    //  Initialize Option structs from command line specification:
    //
    //  PatchTable construction:
    Far::PatchTableFactory::Options patchOptions(args.refineDepth);
    patchOptions.generateAllLevels = allLevelsInPatches;
#ifndef _OSD_PRE_3_4
    patchOptions.includeBaseLevelIndices = true;
    patchOptions.includeFVarBaseLevelIndices = true;

    patchOptions.generateVaryingTables = true;
#endif
    patchOptions.generateFVarTables              = true;
    patchOptions.generateFVarLegacyLinearPatches = args.uvLegacy && !args.uvInterpSet;

    patchOptions.useInfSharpPatch                 = useInfSharp;
    patchOptions.useSingleCreasePatch             = useSingleCrease;
    patchOptions.generateLegacySharpCornerPatches = legacyCorners;
    patchOptions.endCapType                       = args.irregBasis;
    patchOptions.shareEndCapPatchPoints           = true;

    //  Refinement options (depends on above options):
    Far::TopologyRefiner::AdaptiveOptions adaptiveOptions(args.refineDepth);
    adaptiveOptions.secondaryLevel       = args.refineSecond;
    adaptiveOptions.useInfSharpPatch     = patchOptions.useInfSharpPatch;
    adaptiveOptions.useSingleCreasePatch = patchOptions.useSingleCreasePatch;
    adaptiveOptions.considerFVarChannels = patchOptions.generateFVarTables &&
                                          !patchOptions.generateFVarLegacyLinearPatches;

    Far::TopologyRefiner::UniformOptions uniformOptions(args.refineDepth);
    //  Need to set full-topology true when non-linear uniform patches supported...
    uniformOptions.fullTopologyInLastLevel = false;

    //  StencilTable construction (define for vertex and copy to varying and fvar):
    Far::StencilTableFactory::Options vtxStencilOptions;
    vtxStencilOptions.interpolationMode = Far::StencilTableFactory::INTERPOLATE_VERTEX;
    vtxStencilOptions.generateOffsets = true;
    vtxStencilOptions.generateControlVerts = true;
    vtxStencilOptions.generateIntermediateLevels = allLevelsInStencils;

    Far::StencilTableFactory::Options varStencilOptions = vtxStencilOptions;
    varStencilOptions.interpolationMode = Far::StencilTableFactory::INTERPOLATE_VARYING;

    Far::StencilTableFactory::Options fvarStencilOptions = vtxStencilOptions;
    fvarStencilOptions.interpolationMode = Far::StencilTableFactory::INTERPOLATE_FACE_VARYING;
    fvarStencilOptions.fvarChannel = 0;

    //  LimitStencilTable construction:
    Far::LimitStencilTableFactory::Options vtxLimitOptions;
#ifndef _OSD_PRE_3_4
    vtxLimitOptions.interpolationMode = Far::LimitStencilTableFactory::INTERPOLATE_VERTEX;
#endif
    vtxLimitOptions.generate1stDerivatives = true;
    vtxLimitOptions.generate2ndDerivatives = false;

    Far::LimitStencilTableFactory::Options varLimitOptions;
#ifndef _OSD_PRE_3_4
    varLimitOptions.interpolationMode = Far::LimitStencilTableFactory::INTERPOLATE_VARYING;
#endif
    varLimitOptions.generate1stDerivatives = false;
    varLimitOptions.generate2ndDerivatives = false;

    Far::LimitStencilTableFactory::Options fvarLimitOptions;
#ifndef _OSD_PRE_3_4
    fvarLimitOptions.interpolationMode = Far::LimitStencilTableFactory::INTERPOLATE_FACE_VARYING;
    fvarLimitOptions.fvarChannel = 0;
#endif
    fvarLimitOptions.generate1stDerivatives = false;
    fvarLimitOptions.generate2ndDerivatives = false;

    //
    //  Initialize the list of shapes and test each (or only the first):
    //
    //    - currently the internal list can be overridden on the command
    //      line (so use of wildcards is possible)
    //
    //    - still exploring additional command line options, e.g. hoping
    //      to specify a list of shape names from the internal list...
    //
    //  A bit more to be done here...
    //
    std::vector<ShapeDesc>& shapeList = g_shapes;

    if (!args.shapes.empty()) {
        shapeList.swap(args.shapes);
    }
    if (shapeList.empty()) {
        initShapes();
    }

    int shapesToTest  = shapeList.size();
    int shapesIgnored = 0;
    if ((args.shapeCount > 0) && (args.shapeCount < shapesToTest)) {
        shapesIgnored = shapesToTest - args.shapeCount;
        shapesToTest = args.shapeCount;
    }
    int failedShapes = 0;

    if (printTestInfo) {
        printf("Testing %d shapes", shapesToTest);
        if (shapesIgnored) {
            printf(" (%d ignored)", shapesIgnored);
        }
        printf(":\n");
    }
    for (int shapeIndex = 0; shapeIndex < shapesToTest; ++shapeIndex) {
        std::string const & shapeName = shapeList[shapeIndex].name;
        if (printTestInfo) {
            printf("%4d of %d:  '%s'\n", 1 + shapeIndex, shapesToTest,
                    shapeName.c_str());
        }

        //
        //  Load the Shape and verify UV assignment before continuing:
        //
        Shape * shape = Shape::parseObj(shapeList[shapeIndex].data.c_str(),
                                        shapeList[shapeIndex].scheme,
                                        shapeList[shapeIndex].isLeftHanded);
        if (shape == 0) {
            printf("Warning -- shape '%s' ignored (failed to parse)\n",
                    shapeName.c_str());
            continue;
        }

        if (shape->uvs.empty() != shape->faceuvs.empty()) {
            printf("Warning -- ignoring incomplete UV assignment for shape '%s'\n",
                    shapeName.c_str());
            if (!shape->uvs.empty()) shape->uvs.clear();
            if (!shape->faceuvs.empty()) shape->faceuvs.clear();
        }

        //
        //  Construct a refiner from the Shape:
        //
        Sdc::SchemeType sdcType = GetSdcType(*shape);

        Sdc::Options sdcOptions = GetSdcOptions(*shape);
        if (args.uvInterpSet) {
            sdcOptions.SetFVarLinearInterpolation(args.uvInterp);
        }

        Far::TopologyRefiner * refiner =
                Far::TopologyRefinerFactory<Shape>::Create(*shape,
                        Far::TopologyRefinerFactory<Shape>::Options(sdcType, sdcOptions));

        if (refiner == 0) {
            printf("Warning -- shape '%s' ignored (unable to construct refiner)\n",
                    shapeName.c_str());
            ++failedShapes;
            delete shape;
            continue;
        }

        bool shapeHasUVs = (refiner->GetNumFVarChannels() > 0);

        //
        //  Load vertex, varying and face-varying data from the Shape:
        //
        int nVtxBase  = refiner->GetLevel(0).GetNumVertices();
        int nVarBase  = refiner->GetLevel(0).GetNumVertices();
        int nFVarBase = shapeHasUVs ? refiner->GetLevel(0).GetNumFVarValues(0) : 0;

        std::vector<Coord3> vtxCoordsBase(nVtxBase);
        std::vector<Coord3> varCoordsBase(nVarBase);
        std::vector<Coord3> fvarCoordsBase(nFVarBase);

        for (int i = 0; i < nVtxBase; ++i) {
            float const * shapePos = &shape->verts[i * 3];
            vtxCoordsBase[i] = Coord3(shapePos[0], shapePos[1], shapePos[2]);
            varCoordsBase[i] = Coord3(shapePos[0], shapePos[1], shapePos[2]);
        }

        for (int i = 0; i < nFVarBase; ++i) {
            float const * shapeUV = &shape->uvs[i * 2];
            fvarCoordsBase[i] = Coord3(shapeUV[0], shapeUV[1], 0.0f);
        }

        //
        //  Test this Shape:
        //
        bool failuresDetected = false;

        if (args.refineAdaptive) {
            refiner->RefineAdaptive(adaptiveOptions);
        } else {
            refiner->RefineUniform(uniformOptions);
        }

        int maxLevel = refiner->GetMaxLevel();

        int nVtxLast  = refiner->GetLevel(maxLevel).GetNumVertices();
        int nVarLast  = refiner->GetLevel(maxLevel).GetNumVertices();
        int nFVarLast = shapeHasUVs ? refiner->GetLevel(maxLevel).GetNumFVarValues(0) : 0;

        int nVtxRef  = refiner->GetNumVerticesTotal();
        int nVarRef  = refiner->GetNumVerticesTotal();
        int nFVarRef = shapeHasUVs ? refiner->GetNumFVarValuesTotal(0) : 0;

        //
        //  Allocate buffers for refined vertices and populate:
        //
        Coord3Vector vtxCoordsRef  = vtxCoordsBase;
        Coord3Vector varCoordsRef  = varCoordsBase;
        Coord3Vector fvarCoordsRef = fvarCoordsBase;

        vtxCoordsRef.resize(nVtxRef);
        varCoordsRef.resize(nVarRef);
        fvarCoordsRef.resize(nFVarRef);

        if (maxLevel > 0) {
            Far::PrimvarRefiner primvarRefiner(*refiner);

            Coord3 * vtxSrc  = &vtxCoordsRef[0];
            Coord3 * varSrc  = &varCoordsRef[0];
            Coord3 * fvarSrc = shapeHasUVs ? &fvarCoordsRef[0] : 0;

            for (int level = 1; level <= maxLevel; ++level) {
                Coord3 * vtxDst = vtxSrc + refiner->GetLevel(level-1).GetNumVertices();
                Coord3 * varDst = varSrc + refiner->GetLevel(level-1).GetNumVertices();

                primvarRefiner.Interpolate(level, vtxSrc, vtxDst);
                primvarRefiner.InterpolateVarying(level, varSrc, varDst);

                if (shapeHasUVs) {
                    Coord3 * fvarDst = fvarSrc + refiner->GetLevel(level-1).GetNumFVarValues();
                    primvarRefiner.InterpolateFaceVarying(level, fvarSrc, fvarDst);
                    fvarSrc = fvarDst;
                }

                vtxSrc = vtxDst;
                varSrc = varDst;
            }
        }

        //
        //  Use StencilTables (to be used later) to populate and compare:
        //
        Far::StencilTable const * vtxStencilTableRef = 0;
        Far::StencilTable const * varStencilTableRef = 0;
        Far::StencilTable const * fvarStencilTableRef = 0;

        if (testRefineStencils) {
            if (testVertexData) {
                Coord3Vector vtxCoordsRefByStencil(nVtxRef);

                vtxStencilTableRef = Far::StencilTableFactory::Create(*refiner, vtxStencilOptions);
                vtxStencilTableRef->UpdateValues(&vtxCoordsBase[0],
                                                 &vtxCoordsRefByStencil[0]);

                if (allLevelsInStencils) {
                    nDiffs = compareCoord3Vectors(vtxCoordsRef, vtxCoordsRefByStencil, xyzTol);
                } else {
                    nDiffs  = compareCoord3Arrays(nVtxBase,
                                                 &vtxCoordsRef[0], &vtxCoordsRefByStencil[0], xyzTol);
                    nDiffs += compareCoord3Arrays(nVtxLast,
                                                 &vtxCoordsRef[nVtxRef - nVtxLast],
                                                 &vtxCoordsRefByStencil[nVtxBase], xyzTol);
                }
                if (nDiffs) {
                    printf("        %d refined vertex positions differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
            if (testVaryingData) {
                Coord3Vector varCoordsRefByStencil(nVarRef);

                varStencilTableRef = Far::StencilTableFactory::Create(*refiner, varStencilOptions);
                varStencilTableRef->UpdateValues(&varCoordsBase[0],
                                                 &varCoordsRefByStencil[0]);

                if (allLevelsInStencils) {
                    nDiffs = compareCoord3Vectors(varCoordsRef, varCoordsRefByStencil, xyzTol);
                } else {
                    nDiffs  = compareCoord3Arrays(nVarBase,
                                                 &varCoordsRef[0], &varCoordsRefByStencil[0], xyzTol);
                    nDiffs += compareCoord3Arrays(nVarLast,
                                                 &varCoordsRef[nVarRef - nVarLast],
                                                 &varCoordsRefByStencil[nVarBase], xyzTol);
                }
                if (nDiffs) {
                    printf("        %d refined varying values differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
            if (testFVarData && shapeHasUVs) {
                Coord3Vector fvarCoordsRefByStencil(nFVarRef);
/*
printf("\n");
printf("Building FVar StencilTable -- options:\n");
printf("    interpolation mode = %d\n", fvarStencilOptions.interpolationMode);
printf("    gen offsets        = %d\n", fvarStencilOptions.generateOffsets);
printf("    gen control verts  = %d\n", fvarStencilOptions.generateControlVerts);
printf("    gen inter. levels  = %d\n", fvarStencilOptions.generateIntermediateLevels);
printf("    factorize levels   = %d\n", fvarStencilOptions.factorizeIntermediateLevels);
printf("    max level          = %d\n", fvarStencilOptions.maxLevel);
printf("    fvar channel       = %d\n", fvarStencilOptions.fvarChannel);
printf("\n");
*/
                fvarStencilTableRef = Far::StencilTableFactory::Create(*refiner, fvarStencilOptions);
                fvarStencilTableRef->UpdateValues(&fvarCoordsBase[0],
                                                  &fvarCoordsRefByStencil[0]);

                if (allLevelsInStencils) {
                    nDiffs = compareCoord3Vectors(fvarCoordsRef, fvarCoordsRefByStencil, uvTol);
                } else {
                    nDiffs  = compareCoord3Arrays(nFVarBase,
                                                 &fvarCoordsRef[0], &fvarCoordsRefByStencil[0], xyzTol);
                    nDiffs += compareCoord3Arrays(nFVarLast,
                                                 &fvarCoordsRef[nFVarRef - nFVarLast],
                                                 &fvarCoordsRefByStencil[nFVarBase], xyzTol);
                }
                if (nDiffs) {
                    printf("        %d refined face-varying values differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
        }

        //
        //  Create a PatchTable, extend and populate buffers for local points --
        //  use StencilTables (to be used later) to populate and compare:
        //
        Far::PatchTable * patchTable =
                Far::PatchTableFactory::Create(*refiner, patchOptions);

        Far::StencilTable const * vtxStencilTablePatch = 0;
        Far::StencilTable const * varStencilTablePatch = 0;
        Far::StencilTable const * fvarStencilTablePatch = 0;

        if (testPatchStencils) {
            if (testVertexData ) {
                int nVtxLocal  = patchTable->GetNumLocalPoints();
                if (nVtxLocal) {
                    vtxCoordsRef.resize(vtxCoordsRef.size() + nVtxLocal);
                    patchTable->GetLocalPointStencilTable()->UpdateValues(
                            &vtxCoordsRef[0], &vtxCoordsRef[nVtxRef]);

                    Coord3Vector vtxCoordsRefByStencil(vtxCoordsRef.size());

                    vtxStencilTablePatch =
                        Far::StencilTableFactory::AppendLocalPointStencilTable(
                                *refiner, vtxStencilTableRef,
                                patchTable->GetLocalPointStencilTable());

                    vtxStencilTablePatch->UpdateValues(&vtxCoordsBase[0],
                                                       &vtxCoordsRefByStencil[0]);

                    nDiffs = compareCoord3Vectors(vtxCoordsRef, vtxCoordsRefByStencil, xyzTol);
                    if (nDiffs) {
                        printf("        %d refined/patch vertex positions differ\n", nDiffs);
                        failuresDetected = true;
                    }
                } else {
                    vtxStencilTablePatch = vtxStencilTableRef;
                }
            }
            if (testVaryingData) {
                int nVarLocal  = patchTable->GetNumLocalPointsVarying();
                if (nVarLocal) {
                    varCoordsRef.resize(varCoordsRef.size() + nVarLocal);
                    patchTable->GetLocalPointVaryingStencilTable()->UpdateValues(
                            &varCoordsRef[0], &varCoordsRef[nVarRef]);

                    Coord3Vector varCoordsRefByStencil(varCoordsRef.size());

#ifdef _OSD_PRE_3_4
                    varStencilTablePatch = 0;
#else
                    varStencilTablePatch =
                        Far::StencilTableFactory::AppendLocalPointStencilTableVarying(
                                *refiner, varStencilTableRef,
                                patchTable->GetLocalPointVaryingStencilTable());
#endif

                    varStencilTablePatch->UpdateValues(&varCoordsBase[0],
                                                       &varCoordsRefByStencil[0]);

                    nDiffs = compareCoord3Vectors(varCoordsRef, varCoordsRefByStencil, xyzTol);
                    if (nDiffs) {
                        printf("        %d refined/patch varying values differ\n", nDiffs);
                        failuresDetected = true;
                    }
                } else {
                    varStencilTablePatch = varStencilTableRef;
                }
            }
            if (testFVarData && shapeHasUVs) {
                int nFVarLocal = shapeHasUVs ? patchTable->GetNumLocalPointsFaceVarying(0) : 0;
                if (nFVarLocal) {
                    fvarCoordsRef.resize(fvarCoordsRef.size() + nFVarLocal);
                    patchTable->GetLocalPointFaceVaryingStencilTable(0)->UpdateValues(
                            &fvarCoordsRef[0], &fvarCoordsRef[nFVarRef]);

                    Coord3Vector fvarCoordsRefByStencil(fvarCoordsRef.size());

                    fvarStencilTablePatch =
                        Far::StencilTableFactory::AppendLocalPointStencilTableFaceVarying(
                                *refiner, fvarStencilTableRef,
                                patchTable->GetLocalPointFaceVaryingStencilTable(0));

                    fvarStencilTablePatch->UpdateValues(&fvarCoordsBase[0],
                                                        &fvarCoordsRefByStencil[0]);

                    nDiffs = compareCoord3Vectors(fvarCoordsRef, fvarCoordsRefByStencil, uvTol);
                    if (nDiffs) {
                        printf("        %d refined/patch face-varying values differ\n", nDiffs);
                        failuresDetected = true;
                    }
                } else {
                    fvarStencilTablePatch = fvarStencilTableRef;
                }
            }
        }

        //
        //  Compute and compare limit points via PatchTable and LimitStencilTable:
        //
        if (testLimitStencils) {
            //
            //  Define a set of locations for which to compute patch points and limit stencils:
            //
            //  Choose a set of (s,t) coords for each face that work for quad and tri patches:

            int   const nCoords   = 4;
            float const sCoords[] = { 0.0f, 1.0f, 0.0f, 1.0f / 3.0f };
            float const tCoords[] = { 0.0f, 0.0f, 1.0f, 1.0f / 3.0f };

            Far::PtexIndices ptexIndices(*refiner);
            int nPatchFaces = ptexIndices.GetNumFaces();

            Far::LimitStencilTableFactory::LocationArrayVec locations;
            locations.reserve(nPatchFaces);

            int nBaseFaces = refiner->GetLevel(0).GetNumFaces();
            for (int face = 0; face < nBaseFaces; ++face) {
                if (!refiner->GetLevel(0).IsFaceHole(face)) {
                    Far::LimitStencilTableFactory::LocationArray l;

                    l.ptexIdx = ptexIndices.GetFaceId(face);
                    l.numLocations = nCoords;
                    l.s = sCoords;
                    l.t = tCoords;

                    locations.push_back(l);
                }
            }
            int nLimitPoints = (int)locations.size() * nCoords;

            Coord3Vector pByPatch(nLimitPoints);
            Coord3Vector duByPatch(nLimitPoints);
            Coord3Vector dvByPatch(nLimitPoints);
            Coord3Vector varByPatch(nLimitPoints);
            Coord3Vector fvarByPatch(shapeHasUVs ? nLimitPoints : 0);

            //
            //  Evaluate patch points for the defined locations:
            //
            Far::PatchMap patchMap(*patchTable);
            for (int i = 0, l = 0; i < (int)locations.size(); ++i) {
                Far::LimitStencilTableFactory::LocationArray & locArray = locations[i];

                int patchFace = locArray.ptexIdx;
                for (int j = 0; j < locArray.numLocations; ++j, ++l) {
                    float s = locArray.s[j];
                    float t = locArray.t[j];

                    Far::PatchTable::PatchHandle const * handle = patchMap.FindPatch(patchFace, s, t);

                    if (testVertexData) {
                        float wP[20], wDu[20], wDv[20];
                        patchTable->EvaluateBasis(*handle, s, t, wP, wDu, wDv);

                        Coord3 & p  = pByPatch[l];
                        Coord3 & du = duByPatch[l];
                        Coord3 & dv = dvByPatch[l];

                        p.Clear();
                        du.Clear();
                        dv.Clear();

                        ConstIndexArray cvs = patchTable->GetPatchVertices(*handle);
                        for (int k = 0; k < cvs.size(); ++k) {
                            int cvIndex = cvs[k];
                            if (!allLevelsInPatches && (cvIndex >= nVtxBase)) {
                                cvIndex = nVtxRef - nVtxLast - nVtxBase + cvIndex;
                            }
                            Coord3 const & cv = vtxCoordsRef[cvIndex];

                            p.AddWithWeight(cv, wP[k]);
                            du.AddWithWeight(cv, wDu[k]);
                            dv.AddWithWeight(cv, wDv[k]);
                        }
                    }
                    if (testVaryingData) {
                        float w[20];
                        patchTable->EvaluateBasisVarying(*handle, s, t, w);

                        Coord3 & var = varByPatch[l];
                        var.Clear();

                        ConstIndexArray cvs = args.refineAdaptive 
                                            ? patchTable->GetPatchVaryingVertices(*handle)
                                            : patchTable->GetPatchVertices(*handle);
                        for (int k = 0; k < cvs.size(); ++k) {
                            int cvIndex = cvs[k];
                            if (!allLevelsInPatches && (cvIndex >= nVarBase)) {
                                cvIndex = nVarRef - nVarLast - nVarBase + cvIndex;
                            }
                            Coord3 const & cv = varCoordsRef[cvIndex];

                            var.AddWithWeight(cv, w[k]);
                        }
                    }
                    if (testFVarData && shapeHasUVs) {
                        float w[20];
                        patchTable->EvaluateBasisFaceVarying(*handle, s, t, w);

                        Coord3 & fvar = fvarByPatch[l];
                        fvar.Clear();

                        ConstIndexArray cvs = patchTable->GetPatchFVarValues(*handle, 0);
                        for (int k = 0; k < cvs.size(); ++k) {
                            int cvIndex = cvs[k];
                            if (!allLevelsInPatches && (cvIndex >= nFVarBase)) {
                                cvIndex = nFVarRef - nFVarLast - nFVarBase + cvIndex;
                            }
                            Coord3 const & cv = fvarCoordsRef[cvIndex];

                            fvar.AddWithWeight(cv, w[k]);
                        }
                    }
                }
            }

            //
            //  Evaluate defined locations by creating and applying a LimitStencilTable:
            //  
            if (testVertexData) {
                Coord3Vector pByStencil(nLimitPoints);
                Coord3Vector duByStencil(nLimitPoints);
                Coord3Vector dvByStencil(nLimitPoints);

                Far::LimitStencilTable const * vtxLimitTable =
                        Far::LimitStencilTableFactory::Create(*refiner, locations,
                                args.limitStencilArg ? vtxStencilTablePatch : 0,
                                args.limitPatchArg ? patchTable : 0,
                                vtxLimitOptions);
                assert(vtxLimitTable != 0);

                vtxLimitTable->UpdateValues(&vtxCoordsBase[0], &pByStencil[0]);
                vtxLimitTable->UpdateDerivs(&vtxCoordsBase[0], &duByStencil[0], &dvByStencil[0]);
                delete vtxLimitTable;

                //  Compare patch evaluations to LimitStencil applications:
                nDiffs = compareCoord3Vectors(pByPatch, pByStencil, xyzTol);
                if (nDiffs) {
                    printf("        %d evaluated vertex positions differ\n", nDiffs);
                    failuresDetected = true;

                }
                nDiffs = compareCoord3Vectors(duByPatch, duByStencil, xyzTol);
                if (nDiffs) {
                    printf("        %d evaluated vertex U derivatives differ\n", nDiffs);
                    failuresDetected = true;
                }
                nDiffs = compareCoord3Vectors(dvByPatch, dvByStencil, xyzTol);
                if (nDiffs) {
                    printf("        %d evaluated vertex V derivatives differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
            if (testVaryingData) {
                Coord3Vector varByStencil(nLimitPoints);

                Far::LimitStencilTable const * varLimitTable =
                        Far::LimitStencilTableFactory::Create(*refiner, locations,
                                args.limitStencilArg ? varStencilTablePatch : 0,
                                args.limitPatchArg ? patchTable : 0,
                                varLimitOptions);
                assert(varLimitTable != 0);

                varLimitTable->UpdateValues(&varCoordsBase[0], &varByStencil[0]);
                delete varLimitTable;

                //  Compare patch evaluations to LimitStencil applications:
                nDiffs = compareCoord3Vectors(varByPatch, varByStencil, xyzTol);
                if (nDiffs) {
                    printf("        %d evaluated varying results differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
#ifdef _OSD_PRE_3_4
            if (false) {
#else
            if (testFVarData && shapeHasUVs) {
#endif
                Coord3Vector fvarByStencil(shapeHasUVs ? nLimitPoints : 0);

                Far::LimitStencilTable const * fvarLimitTable =
                        Far::LimitStencilTableFactory::Create(*refiner, locations,
                                args.limitStencilArg ? fvarStencilTablePatch : 0,
                                args.limitPatchArg ? patchTable : 0,
                                fvarLimitOptions);
                assert(fvarLimitTable != 0);

                fvarLimitTable->UpdateValues(&fvarCoordsBase[0], &fvarByStencil[0]);
                delete fvarLimitTable;

                //  Compare patch evaluations to LimitStencil applications:
                nDiffs = compareCoord3Vectors(fvarByPatch, fvarByStencil, uvTol);
                if (nDiffs) {
                    printf("        %d evaluated face-varying results differ\n", nDiffs);
                    failuresDetected = true;
                }
            }
        }

        //
        //  Summarize any failures and clean up data allocated for this Shape:
        //
        if (failuresDetected) {
            ++ failedShapes;
            printf("\nFailures detected for shape '%s':\n", shapeName.c_str());
        }

        if (vtxStencilTablePatch  != vtxStencilTableRef)  delete vtxStencilTablePatch;
        if (varStencilTablePatch  != varStencilTableRef)  delete varStencilTablePatch;
        if (fvarStencilTablePatch != fvarStencilTableRef) delete fvarStencilTablePatch;
        delete vtxStencilTableRef;
        delete varStencilTableRef;
        delete fvarStencilTableRef;
        delete patchTable;
        delete refiner;
        delete shape;
    }

    if (printSummary) {
        printf("\n");
        if (failedShapes == 0) {
            printf("All tests passed for %d shapes\n", shapesToTest);
        } else {
            printf("Total failures: %d of %d shapes\n", failedShapes, shapesToTest);
        }
    }
}
