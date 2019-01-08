//
//   Copyright 2016 Dreamworks
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

#include <set>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <cstdio>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/topologyRefinerFactory.h>
#include <opensubdiv/far/topologyLevel.h>
#include <opensubdiv/far/patchTable.h>
#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/patchMap.h>
#include <opensubdiv/far/ptexIndices.h>

#include "../../regression/common/far_utils.h"

#include "init_shapes.h"
#include "testMesh.h"

//
// Regression testing for Far refinement of face-varying channels.  All shapes
// are tested against a constructed FVar channel of matching topology.  Those
// shapes with UVs specified have their UV FVar channel tested for all of the
// non-linear FVar linear interpolation options.
//
// Testing matching topology:
//     - all shapes are tested by creating a single FVar channel whose
//       topology matches the vertices
//     - vertex data (positions) is used as the source for interpolation
//       of both the refined vertices and refined FVar channel
//     - comparison of the refined/interpolated vertex and FVar data is
//       all that is required
//
// Testing asisgned UVs:
//     - a UV mesh is created by extracting the UV topology as UVW coordinates
//       as the vertex data on a new mesh
//     - for each of the non-linear FVar options the boundaries of the UV mesh
//       are sharpened accordingly and the result is compared to the original
//     - comparison of both uniform refinment and patch sampling is applied
//
// Notes:
//     - results are not compared as bitwise identical as there are no
//       guarantees that the order of operations is identical
//     - tolerances for comparison are currently fixed at 1e-6
//

using namespace OpenSubdiv;
using namespace OpenSubdiv::OPENSUBDIV_VERSION;

using Far::Index;
using Far::LocalIndex;
using Far::ConstIndexArray;
using Far::ConstLocalIndexArray;


//
//  Simple structs to capture differences exceeding tolerances:
//
struct VertexDiff {
    VertexDiff() { }
    VertexDiff(int level, int vertex, float diff) :
        levelIndex(level), vertexIndex(vertex), difference(diff) { }

    int   levelIndex;
    int   vertexIndex;
    float difference;
};

struct FaceVertexDiff {
    FaceVertexDiff() { }
    FaceVertexDiff(int level, int face, int corner, float diff) :
        levelIndex(level), faceIndex(face), faceVertex(corner), difference(diff) { }

    int   levelIndex;
    int   faceIndex;
    int   faceVertex;
    float difference;
};

struct PatchDiff {
    PatchDiff() { }
    PatchDiff(int patch, float uCoord, float vCoord, float diff) :
        patchFace(patch), u(uCoord), v(vCoord), difference(diff) { }

    int   patchFace;
    float u, v;
    float difference;
};


//
//  Simple class to encapsulate and simplify Far patch evaluation:
//
class PatchEvaluator {
public:
    PatchEvaluator(Far::PatchTable const & pTable,
                   Far::PatchMap const &   pMap,
                   Coord3Vector const &    pPoints,
                   int                     fvChannel = -1) :
           patchTable(pTable), patchMap(pMap), patchPoints(pPoints), fvarChannel(fvChannel) { }

    Coord3 evaluate(int patch, float u, float v) const;

public:
    Far::PatchTable const & patchTable;
    Far::PatchMap const &   patchMap;
    Coord3Vector const &    patchPoints;
    int                     fvarChannel;
};

Coord3
PatchEvaluator::evaluate(int patch, float u, float v) const {

    Far::PatchTable::PatchHandle const * handle = patchMap.FindPatch(patch, u, v);
    assert(handle);

    float weights[20];
    ConstIndexArray points;
    if (fvarChannel < 0) {
        patchTable.EvaluateBasis(*handle, u, v, weights);
        points = patchTable.GetPatchVertices(*handle);
    } else {
        patchTable.EvaluateBasisFaceVarying(*handle, u, v, weights, 0,0, 0,0,0, fvarChannel);
        points = patchTable.GetPatchFVarValues(*handle, fvarChannel);
    }

    Coord3 result(0,0,0);
    for (int i = 0; i < points.size(); ++i) {
        result.AddWithWeight(patchPoints[points[i]], weights[i]);
    }
    return result;
}


//
//  Global functions to compare vertex and face-varying data between two 
//  meshes or within the same mesh (specify the same mesh for both), and
//  accumulating differences exceeding tolerances.
//
//  Compare correspondingly refined vertex and FVar values:
//
void
compareRefinedValues(TestMesh const & vtxMesh, TestMesh const & fvarMesh,
                     float tolerance,
                     std::vector<VertexDiff> & vertexDiffs) {

    vertexDiffs.resize(0);

    Far::TopologyRefiner const & vtxRefiner  = vtxMesh.GetTopologyRefiner();
    Far::TopologyRefiner const & fvarRefiner = fvarMesh.GetTopologyRefiner();

    Coord3Vector const & vtxCoords   = vtxMesh.GetXyzVector();
    Coord3Vector const & fvarCoords  = fvarMesh.GetUvwVector();
    int                  fvarChannel = fvarMesh.GetUvChannel();

    Coord3 const * vtxValues  = &vtxCoords[0];
    Coord3 const * fvarValues = &fvarCoords[0];

    for (int level = 1; level <= vtxRefiner.GetMaxLevel(); ++level) {

        int nVertices   = vtxRefiner.GetLevel(level).GetNumVertices();
        int nFVarValues = fvarRefiner.GetLevel(level).GetNumFVarValues(fvarChannel);
        assert(nVertices == nFVarValues);

        for (int i = 0; i < nVertices; ++i) {
            float maxDelta = Coord3::MaxCoordDelta(vtxValues[i], fvarValues[i]);
            if (maxDelta > tolerance) {
                vertexDiffs.push_back(VertexDiff(level, i, maxDelta));
            }
        }

        vtxValues  += nVertices;
        fvarValues += nFVarValues;
    }
}

void
compareRefinedFaces(TestMesh const & vtxMesh, TestMesh const & fvarMesh,
                     float tolerance,
                     std::vector<FaceVertexDiff> & faceVertexDiffs) {

    faceVertexDiffs.resize(0);

    Far::TopologyRefiner const & vtxRefiner  = vtxMesh.GetTopologyRefiner();
    Far::TopologyRefiner const & fvarRefiner = fvarMesh.GetTopologyRefiner();

    Coord3 const * vtxCoords   = &vtxMesh.GetXyzVector()[0];
    Coord3 const * fvarCoords  = &fvarMesh.GetUvwVector()[0];
    int            fvarChannel = fvarMesh.GetUvChannel();

    vtxCoords  += vtxRefiner.GetLevel(0).GetNumVertices();
    fvarCoords += fvarRefiner.GetLevel(0).GetNumFVarValues(fvarChannel);

    for (int level = 1; level <= vtxRefiner.GetMaxLevel(); ++level) {

        Far::TopologyLevel const & vtxLevel  = vtxRefiner.GetLevel(level);
        Far::TopologyLevel const & fvarLevel = fvarRefiner.GetLevel(level);
        assert(vtxLevel.GetNumFaces() == fvarLevel.GetNumFaces());

        int nFaces = vtxLevel.GetNumFaces();
        for (int i = 0; i < nFaces; ++i) {
            ConstIndexArray fVerts  = vtxLevel.GetFaceVertices(i);
            ConstIndexArray fValues = fvarLevel.GetFaceFVarValues(i, fvarChannel);

            for (int j = 0; j < fVerts.size(); ++j) {
                float maxDelta = Coord3::MaxCoordDelta(vtxCoords[fVerts[j]], fvarCoords[fValues[j]]);
                if (maxDelta > tolerance) {
                    faceVertexDiffs.push_back(FaceVertexDiff(level, i, j, maxDelta));
                }
            }
        }

        vtxCoords  += vtxLevel.GetNumVertices();
        fvarCoords += fvarLevel.GetNumFVarValues(fvarChannel);
    }
}

//
//  Compare vertex and FVar patches via their corresponding base face:
//
void
comparePatchSamples(TestMesh const & vtxMesh, TestMesh const & fvarMesh,
                    int samplesPerEdge, float tolerance,
                    std::vector<PatchDiff> & patchDiffs) {

    patchDiffs.resize(0);

    //  Construct local PatchEvaluators to simplify evaluation:
    Far::PatchMap  vtxPatchMap(vtxMesh.GetPatchTable());
    PatchEvaluator vtxPatchEval(vtxMesh.GetPatchTable(), vtxPatchMap, vtxMesh.GetXyzVector());

    Far::PatchMap  fvarPatchMap(fvarMesh.GetPatchTable());
    PatchEvaluator fvarPatchEval(fvarMesh.GetPatchTable(), fvarPatchMap, fvarMesh.GetUvwVector(),
                                                                         fvarMesh.GetUvChannel());

    float dt = (samplesPerEdge > 1) ? (1.0f / (float)(samplesPerEdge - 1)) : 1.0;

    //  Iterate through patches for all base faces (and their child faces when irregular):
    Far::TopologyLevel const & baseLevel = vtxMesh.GetTopologyRefiner().GetLevel(0);

    int patchFace = 0;

    int nBaseFaces = baseLevel.GetNumFaces();
    for (int baseFace = 0; baseFace < nBaseFaces; ++baseFace) {
        int nChildFaces = baseLevel.GetFaceVertices(baseFace).size();
        int nChildPatches = (nChildFaces == vtxMesh.GetRegFaceSize()) ? 1 : nChildFaces;

        for (int childPatch = 0; childPatch < nChildPatches; ++childPatch, ++patchFace) {

            //  Putting this condition inside the loop to ensure patchFace incremented
            if (baseLevel.IsFaceHole(baseFace)) continue;

            for (int i = 0; i < samplesPerEdge; ++i) {
                for (int j = 0; j < samplesPerEdge; ++j) {
                    float u = dt * (float)i;
                    float v = dt * (float)j;

                    Coord3 vtxSample  = vtxPatchEval.evaluate(patchFace, u, v);
                    Coord3 fvarSample = fvarPatchEval.evaluate(patchFace, u, v);

                    float maxDelta = Coord3::MaxCoordDelta(vtxSample, fvarSample);
                    if (maxDelta > tolerance) {
                        patchDiffs.push_back(PatchDiff(i, u, v, maxDelta));
                    }
                }
            }
        }
    }
}


//
//  Functions for reporting information on the accumulated differences (may want
//  more verbose options here in future):
//
void
reportVertexDifferences(std::vector<VertexDiff> & vertexDiffs) {

    float maxDiff = 0.0f;
    for (int i = 0; i < (int)vertexDiffs.size(); ++i) {
        maxDiff = std::max(maxDiff, vertexDiffs[i].difference);
    }

    printf("        %d refined values differ, max delta = %g\n", (int)vertexDiffs.size(), maxDiff);
}

void
reportFaceVertexDifferences(std::vector<FaceVertexDiff> & faceVertexDiffs) {

    float maxDiff = 0.0f;
    for (int i = 0; i < (int)faceVertexDiffs.size(); ++i) {
        maxDiff = std::max(maxDiff, faceVertexDiffs[i].difference);
    }

    printf("        %d refined face-vertex values differ, max delta = %g\n", (int)faceVertexDiffs.size(), maxDiff);
}

void
reportPatchSampleDifferences(std::vector<PatchDiff> & patchDiffs) {

    int nBadSamples = 0;
    int nBadPatches = 0;
    int lastMismatch = -1;
    float maxDiff = 0.0f;
    for (int i = 0; i < (int)patchDiffs.size(); ++i) {
        if (patchDiffs[i].difference < 0.0f) {
            if (lastMismatch != patchDiffs[i].patchFace) {
                ++nBadPatches;
                lastMismatch = patchDiffs[i].patchFace;
            }
        } else {
            ++nBadSamples;
            maxDiff = std::max(maxDiff, patchDiffs[i].difference);
        }
    }
    if (nBadPatches) {
        printf("        %d patches with mismatching topology\n", nBadPatches);
    }
    if (nBadSamples) {
        printf("        %d patch samples differ, max delta = %g\n", nBadSamples, maxDiff);
    }
}


//
//  Information about each of the FVar linear interpolation modes tested:
//
//  Note that it will be a while before tests for all linear interpolation options
//  are supported.  Until then, each interpolation option includes a bool member
//  indicating whether testing should be enabled -- which may also be useful as a
//  future command line option (requiring this table be non-const).
//
#define FVAR_OPTION_COUNT   (1 + (int)Sdc::Options::FVAR_LINEAR_ALL)

struct FVarOptionInfo {
    typedef Sdc::Options::FVarLinearInterpolation Type;

    Type         type;
    char const * name;
    bool         test;
};

FVarOptionInfo const g_fvarOptionInfo[] = {
    { Sdc::Options::FVAR_LINEAR_NONE,          "LINEAR_NONE",          true },
    { Sdc::Options::FVAR_LINEAR_CORNERS_ONLY,  "LINEAR_CORNERS_ONLY",  true },
    { Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1, "LINEAR_CORNERS_PLUS1", true },
    { Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2, "LINEAR_CORNERS_PLUS2", true },
    { Sdc::Options::FVAR_LINEAR_BOUNDARIES,    "LINEAR_BOUNDARIES",    true },
    { Sdc::Options::FVAR_LINEAR_ALL,           "LINEAR_ALL",           false }
};


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
    Args() { initialize(); }
    Args(int argc, char **argv);
    ~Args() { }

public:
    int                    refineDepth;
    int                    shapeCount;
    Scheme                 shapeScheme;
    std::string            shapeName;
    std::vector<ShapeDesc> shapes;

private:
    void initialize() {
        refineDepth = 3;
        shapeCount  = 0;
        shapeScheme = kCatmark;
    }
};

Args::Args(int argc, char **argv) {

    initialize();

    std::string fileString;

    for (int i = 1; i < argc; ++i) {
        char * arg = argv[i];

        if (!strcmp(arg, "-shape")) {
            if (++i < argc) shapeName = std::string(arg);
        } else if (!strcmp(arg, "-depth")) {
            if (++i < argc) refineDepth = atoi(argv[i]);
        } else if (!strcmp(arg, "-count")) {
            if (++i < argc) shapeCount = atoi(argv[i]);
        } else if (!strcmp(arg, "-bilinear")) {
            shapeScheme = kBilinear;
        } else if (!strcmp(arg, "-catmark")) {
            shapeScheme = kCatmark;
        } else if (!strcmp(arg, "-loop")) {
            shapeScheme = kLoop;
        } else if (strstr(arg, ".obj")) {
            if (readString(arg, fileString)) {
                //  Use the scheme declared at the time so that multiple shape/scheme
                //  pairs can be specified
                shapes.push_back(ShapeDesc(arg, fileString.c_str(), shapeScheme));
            } else {
                printf("Unable to open/read .obj file '%s'\n", arg);
                exit(0);
            }
        } else {
            printf("Ignoring unrecognized argument '%s'\n", arg);
        }
    }
}


int
main(int argc, char **argv) {

    Args args(argc, argv);

    //
    //  Options controlling the tests and shapes -- move more of these variables
    //  into members of the command line Args class parsed above...
    //
    //  Options related to testing:
    bool  printTestInfo          = true;
    bool  runFVarPerVertexTests  = true;
    bool  runFVarBoundaryTests   = true;
    bool  runRefinementTests     = true;
    bool  runPatchSampleTests    = true;
    float xyzTolerance           = 1.0e-6;
    int   patchSampleRate        = 5;
    int   preUvChannels          = 0;
    int   postUvChannels         = 0;

    //  Options related to shape representation and accuracy:
    bool  useInfSharp     = false;
    bool  refineAdaptive  = true;
    bool  useSingleCrease = false;
    bool  useGregory      = true;
    bool  legacyCorners   = true;
    int   refineLevels    = args.refineDepth;
    int   secondaryLevel  = 12;

    printf("\n");
    printf("Shape options:\n");
    printf("  - patch depth       = %d\n", refineLevels);
    if (secondaryLevel < refineLevels) {
        printf("  - secondary level   = %d\n", secondaryLevel);
    }
    printf("  - gregory patches   = %d\n", (int) useGregory);
    printf("  - inf-sharp patches = %d\n", (int) useInfSharp);
    printf("  - smooth corners    = %d\n", (int) !legacyCorners);
    printf("\n");

    //
    //  Identify the tests enabled -- and verify at least one is...
    //
    bool testsEnabled = false;
    printf("Tests enabled:\n");
    if (runFVarPerVertexTests && (runRefinementTests || runPatchSampleTests)) {
        printf("  - FVar values per vertex (FVar linear interpolation matches Vtx boundary interpolation) \n");
        testsEnabled = true;
    }
    if (runFVarBoundaryTests && (runRefinementTests || runPatchSampleTests)) {
        for (int i = 0; i < FVAR_OPTION_COUNT; ++i) {
            if (!g_fvarOptionInfo[i].test) continue;
            printf("  - FVar values as assigned with linear interpolation '%s'\n", g_fvarOptionInfo[i].name);
            testsEnabled = true;
        }
    }
    if (!testsEnabled) {
        printf("    none\n");
        return EXIT_SUCCESS;
    }
    printf("\n");

    //  A few reminders that should eventually be deleted...
    if (useInfSharp) printf("... warning -- enabled inf-sharp-patches (non default) ...\n\n");
    if (refineLevels <= 2) printf("... warning -- low refinement level (%d) is a weak test ...\n\n", refineLevels);
    if (preUvChannels) printf("... warning -- assigning %d pre-Uv channels ...\n\n", preUvChannels);
    if (postUvChannels) printf("... warning -- assigning %d post-Uv channels ...\n\n", postUvChannels);

    //
    //  Initialize some of the Option structs from potential command line
    //  specification.  The eventual intent it to be able to generate
    //  permutations of these to run variations of the tests for the same
    //  shapes -- particularly the irregular patch types BSpline and Gregory.
    //
    Far::TopologyRefiner::AdaptiveOptions adaptiveOptions(refineLevels);
    adaptiveOptions.secondaryLevel       = secondaryLevel;
    adaptiveOptions.useSingleCreasePatch = useSingleCrease;
    adaptiveOptions.useInfSharpPatch     = useInfSharp;
    adaptiveOptions.considerFVarChannels = true;

    Far::TopologyRefiner::UniformOptions uniformOptions(refineLevels);

    Far::PatchTableFactory::Options patchOptions(refineLevels);
    patchOptions.useSingleCreasePatch            = useSingleCrease;
    patchOptions.useInfSharpPatch                = useInfSharp;
    patchOptions.generateFVarTables              = true;
    patchOptions.generateFVarLegacyLinearPatches = false;
    patchOptions.shareEndCapPatchPoints          = true;
    patchOptions.endCapType                      = useGregory ?
                Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS :
                Far::PatchTableFactory::Options::ENDCAP_BSPLINE_BASIS;
    //  Need to update this branch before we can use this...
    patchOptions.generateLegacySharpCornerPatches = legacyCorners;

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
        //
        //  The Shape class does not have its data in a form that is easily accessible 
        //  to us so build a TestMesh from it and then copy/modify the face-varying
        //  channels of it (or a copy) as needed.
        //
        TestMesh originalMesh;

        std::string const & shapeName = shapeList[shapeIndex].name;
        if (printTestInfo) {
            printf("%4d of %d:  '%s'\n", 1 + shapeIndex, shapesToTest, shapeName.c_str());
        }

        Shape * shape = Shape::parseObj(shapeList[shapeIndex].data.c_str(), shapeList[shapeIndex].scheme);
        if (shape) {
            if (shape->uvs.empty() != shape->faceuvs.empty()) {
                printf("Warning -- ignoring incomplete UV assignment for shape '%s'\n", shapeName.c_str());
                if (!shape->uvs.empty()) shape->uvs.clear();
                if (!shape->faceuvs.empty()) shape->faceuvs.clear();
            }
            originalMesh.BuildFromShape(*shape);
            delete shape;
        } else {
            printf("Warning -- shape '%s' ignored (failed to parse)\n", shapeName.c_str());
            continue;
        }

        //  Determine which tests are applicable to this shape, based on both its
        //  content and specified options:
        //
        bool canRefineAdaptive = refineAdaptive && originalMesh.IsCatmark();

        bool doRefinementTests  = runRefinementTests;
        bool doPatchSampleTests = runPatchSampleTests && canRefineAdaptive;  // may disable for non-manifold

        bool doFVarPerVertexTests = runFVarPerVertexTests;
        bool doFVarBoundaryTests  = runFVarBoundaryTests && originalMesh.HasUvs();

        std::vector<VertexDiff> fvarPerVertexRefinedValueDiffs;
        std::vector<PatchDiff>  fvarPerVertexPatchSampleDiffs;

        std::vector<FaceVertexDiff> fvarBoundaryRefinedValueDiffs[FVAR_OPTION_COUNT];
        std::vector<PatchDiff>      fvarBoundaryPatchSampleDiffs[FVAR_OPTION_COUNT];

        bool failuresDetected = false;

        //  Create a copy of the test mesh with a fvar-channel with per-vertex fvar values
        //  in place of the original
        //
        if (doFVarPerVertexTests) {
            TestMesh vertexMesh(originalMesh);

            vertexMesh.AssignEmptyChannels(preUvChannels, postUvChannels);
            vertexMesh.AssignUvChannelPerVertex();

            vertexMesh.Rebuild(canRefineAdaptive  ? &adaptiveOptions : 0,
                              !canRefineAdaptive  ? &uniformOptions : 0,
                               doPatchSampleTests ? &patchOptions : 0);

            //  Compare refined vertex and fvar values (which should match) and sample points
            //  on the corresponding vertex and fvar patches:
            //
            if (doRefinementTests) {
                compareRefinedValues(vertexMesh, vertexMesh, xyzTolerance,
                        fvarPerVertexRefinedValueDiffs);
                failuresDetected |= (fvarPerVertexRefinedValueDiffs.size() > 0);
            }
            if (doPatchSampleTests) {
                comparePatchSamples(vertexMesh, vertexMesh, patchSampleRate, xyzTolerance,
                        fvarPerVertexPatchSampleDiffs);
                failuresDetected |= (fvarPerVertexPatchSampleDiffs.size() > 0);
            }
        }

        //  Run the face-varying boundary tests for the FVar linear interpolation
        //  choices for which we can easily create a separate UV mesh -- comparing
        //  the FVar refined values and patches of the original FVar channel with
        //  those in the UV mesh (comparing refined results per-face-vertex)
        //
        if (doFVarBoundaryTests) {
            TestMesh xyzMeshOriginalUvs(originalMesh);
            xyzMeshOriginalUvs.AssignEmptyChannels(preUvChannels, postUvChannels);

            TestMesh xyzMeshExpandedUvs(originalMesh);
            xyzMeshExpandedUvs.AssignEmptyChannels(preUvChannels, postUvChannels);
            xyzMeshExpandedUvs.ExpandUvChannel();

            TestMesh uvwMeshReference;
            uvwMeshReference.BuildFromUvChannel(xyzMeshExpandedUvs);

            for (int i = 0; i < FVAR_OPTION_COUNT; ++i) {
                FVarOptionInfo const & fvarOptionInfo = g_fvarOptionInfo[i];

                if (!fvarOptionInfo.test) continue;

                xyzMeshOriginalUvs.AssignUvChannelInterpolation(fvarOptionInfo.type);

                TestMesh uvwMeshPerFVarOption(uvwMeshReference);
                uvwMeshPerFVarOption.SharpenFromUvChannel(xyzMeshExpandedUvs, fvarOptionInfo.type);

                if (doRefinementTests) {
                    xyzMeshOriginalUvs.Rebuild(0, &uniformOptions, 0);
                    uvwMeshPerFVarOption.Rebuild(0, &uniformOptions, 0);

                    compareRefinedFaces(uvwMeshPerFVarOption, xyzMeshOriginalUvs, xyzTolerance,
                            fvarBoundaryRefinedValueDiffs[i]);
                    failuresDetected |= (fvarBoundaryRefinedValueDiffs[i].size() > 0);
                }
                if (doPatchSampleTests) {
                    xyzMeshOriginalUvs.Rebuild(&adaptiveOptions, 0, &patchOptions);
                    uvwMeshPerFVarOption.Rebuild(&adaptiveOptions, 0, &patchOptions);

                    comparePatchSamples(uvwMeshPerFVarOption, xyzMeshOriginalUvs, patchSampleRate, xyzTolerance,
                            fvarBoundaryPatchSampleDiffs[i]);
                    failuresDetected |= (fvarBoundaryPatchSampleDiffs[i].size() > 0);
                }
            }
        }

        //  Detect and report the accumulated results for the tests to the level
        //  of detail specified:
        //
        if (failuresDetected) {
            ++ failedShapes;
            printf("\nFailures detected for shape '%s':\n", shapeName.c_str());

            if ((fvarPerVertexRefinedValueDiffs.size() > 0) ||
                (fvarPerVertexPatchSampleDiffs.size() > 0)) {

                printf("    FVar value per vertex:\n");
                if (fvarPerVertexRefinedValueDiffs.size() > 0) {
                    reportVertexDifferences(fvarPerVertexRefinedValueDiffs);
                }
                if (fvarPerVertexPatchSampleDiffs.size() > 0) {
                    reportPatchSampleDifferences(fvarPerVertexPatchSampleDiffs);
                }
            }
            for (int i = 0; i < FVAR_OPTION_COUNT; ++i) {
                if ((fvarBoundaryRefinedValueDiffs[i].size() > 0) ||
                    (fvarBoundaryPatchSampleDiffs[i].size() > 0)) {

                    printf("    FVar interpolation option '%s':\n", g_fvarOptionInfo[i].name);
                    if (fvarBoundaryRefinedValueDiffs[i].size() > 0) {
                        reportFaceVertexDifferences(fvarBoundaryRefinedValueDiffs[i]);
                    }
                    if (fvarBoundaryPatchSampleDiffs[i].size() > 0) {
                        reportPatchSampleDifferences(fvarBoundaryPatchSampleDiffs[i]);
                    }
                }
            }
            printf("\n");
        }
    }

    if (failedShapes == 0) {
        printf("All tests passed for %d shapes\n", shapesToTest);
    } else {
        printf("Total failures: %d of %d shapes\n", failedShapes, shapesToTest);
    }
}
