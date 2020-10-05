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

#include <regression/common/far_utils.h>
//#include "../../regression/common/far_utils.h"

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

bool debug = false;
if (debug) {
printf("compareRefinedValues(tol = %g)...\n", tolerance);
}
    for (int level = 1; level <= vtxRefiner.GetMaxLevel(); ++level) {

        int nVertices   = vtxRefiner.GetLevel(level).GetNumVertices();
        int nFVarValues = fvarRefiner.GetLevel(level).GetNumFVarValues(fvarChannel);
        assert(nVertices == nFVarValues);

        for (int i = 0; i < nVertices; ++i) {
            float maxDelta = Coord3::MaxCoordDelta(vtxValues[i], fvarValues[i]);
            if (maxDelta > tolerance) {
if (debug) {
printf("    level %d, vertex %d:  maxDelta = %f\n", level, i, maxDelta);
printf("        vtxValue  = % f % f % f\n", vtxValues[i].x, vtxValues[i].y, vtxValues[i].z);
printf("        fvarValue = % f % f % f\n", fvarValues[i].x, fvarValues[i].y, fvarValues[i].z);
}
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
    assert(vtxRefiner.GetMaxLevel() == fvarRefiner.GetMaxLevel());

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

                    if ((vtxMesh.GetRegFaceSize() == 3) && ((u + v) >= 1.0)) continue;

                    Coord3 vtxSample  = vtxPatchEval.evaluate(patchFace, u, v);
                    Coord3 fvarSample = fvarPatchEval.evaluate(patchFace, u, v);

                    float maxDelta = Coord3::MaxCoordDelta(vtxSample, fvarSample);
                    if (maxDelta > tolerance) {
                        patchDiffs.push_back(PatchDiff(i, u, v, maxDelta));
bool debug = false;
if (debug) {
printf("Patch %d.%d (%f,%f): vtx = %7.4f %7.4f, fvar = %7.4f %7.4f\n", baseFace, childPatch, u, v,
                vtxSample.x, vtxSample.y, fvarSample.x, fvarSample.y);
}
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
//  Information about the FVar linear interpolation options that can be tested:
//
struct FVarInterpInfo {
    static const int EnumCount = 1 + (int)Sdc::Options::FVAR_LINEAR_ALL;
    static const int FullMask  = (1 << EnumCount) - 1;
    static const int MaxIndex  = EnumCount - 1;

    static bool IsValidIndex(int index) {
        return (0 <= index) && (index <= MaxIndex);
    }

    static Sdc::Options::FVarLinearInterpolation GetType(int index) {
        assert(IsValidIndex(index));
        return static_cast<Sdc::Options::FVarLinearInterpolation>(index);
    }

    static char const * GetName(int index) {
        assert(IsValidIndex(index));
        switch (GetType(index)) {
            case Sdc::Options::FVAR_LINEAR_NONE:          return "LINEAR_NONE";          break;
            case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:  return "LINEAR_CORNERS_ONLY";  break;
            case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1: return "LINEAR_CORNERS_PLUS1"; break;
            case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2: return "LINEAR_CORNERS_PLUS2"; break;
            case Sdc::Options::FVAR_LINEAR_BOUNDARIES:    return "LINEAR_BOUNDARIES";    break;
            case Sdc::Options::FVAR_LINEAR_ALL:           return "LINEAR_ALL";           break;
            default: return 0;
        }
    }

    static bool IsEnabled(int index, int mask) {
        assert(IsValidIndex(index));

        //  Tests are always disabled for LINEAR_ALL for now...
        if (GetType(index) == Sdc::Options::FVAR_LINEAR_ALL) return false;

        return mask & (1 << index);
    }
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
    std::vector<ShapeDesc> shapes;
    std::string            shapeName;

    Scheme shapeScheme;
    int    shapeCount;
    bool   refineAdaptive;
    int    refineDepth;
    int    refineSecond;
    bool   infSharpFlag;
    bool   singleCreaseFlag;
    bool   smoothCornerFlag;
    int    sampleRate;
    int    uvInterpMask;
    bool   perVertexFlag;
    bool   boundaryFlag;
    bool   refinementFlag;
    bool   patchSampleFlag;
    float  tolerance;
    int    preUvChannels;
    int    postUvChannels;


private:
    static const int dfltUvInterpMask = (1 << FVarInterpInfo::EnumCount) - 1;

    void initialize() {
        shapeScheme      = kCatmark;
        shapeCount       = 0;
        refineAdaptive   = true;
        refineDepth      = 3;
        refineSecond     = 12;
        infSharpFlag     = true;
        singleCreaseFlag = false;
        smoothCornerFlag = true;
        sampleRate       = 5;
        uvInterpMask     = FVarInterpInfo::FullMask;
        perVertexFlag    = true;
        boundaryFlag     = true;
        refinementFlag   = true;
        patchSampleFlag  = true;
        tolerance        = 1.0e-6;
        preUvChannels    = 0;
        postUvChannels   = 0;
    }
};

Args::Args(int argc, char **argv) {

    setlinebuf(stdout);

    initialize();

    std::vector<std::string> objFiles;

    for (int i = 1; i < argc; ++i) {
        char * arg = argv[i];

        if (strstr(arg, ".obj")) {
            objFiles.push_back(std::string(arg));
        } else if (!strcmp(arg, "-bilinear")) {
            shapeScheme = kBilinear;
        } else if (!strcmp(arg, "-catmark")) {
            shapeScheme = kCatmark;
        } else if (!strcmp(arg, "-loop")) {
            shapeScheme = kLoop;
        } else if (!strcmp(arg, "-shape")) {
            if (++i < argc) shapeName = std::string(arg);
        } else if (!strcmp(arg, "-count")) {
            if (++i < argc) shapeCount = atoi(argv[i]);
        } else if (!strcmp(arg, "-novertex")) {
            perVertexFlag   = false;
        } else if (!strcmp(arg, "-noboundary")) {
            boundaryFlag    = false;
        } else if (!strcmp(arg, "-norefine")) {
            refinementFlag  = false;
        } else if (!strcmp(arg, "-nopatch")) {
            patchSampleFlag = false;
        } else if (!strcmp(arg, "-a")) {
            refineAdaptive = true;
        } else if (!strcmp(arg, "-u")) {
            refineAdaptive = false;
        } else if (!strcmp(arg, "-l")) {
            if (++i < argc) refineDepth = atoi(argv[i]);
        } else if (!strcmp(arg, "-l2")) {
            if (++i < argc) refineSecond = atoi(argv[i]);
        } else if (!strcmp(arg, "-res")) {
            if (++i < argc) sampleRate = atoi(argv[i]);
        } else if (!strcmp(arg, "-inf")) {
            infSharpFlag = true;
        } else if (!strcmp(arg, "-noinf")) {
            infSharpFlag = false;
        } else if (!strcmp(arg, "-single")) {
            singleCreaseFlag = true;
        } else if (!strcmp(arg, "-nosingle")) {
            singleCreaseFlag = false;
        } else if (!strcmp(arg, "-smooth")) {
            smoothCornerFlag = true;
        } else if (!strcmp(arg, "-nosmooth")) {
            smoothCornerFlag = false;
        } else if (!strcmp(arg, "-uvint")) {
            if (++i < argc) uvInterpMask = atoi(argv[i]);
        } else if (!strcmp(arg, "-tol")) {
            if (++i < argc) tolerance = atof(argv[i]);
        } else if (!strcmp(arg, "-pre")) {
            if (++i < argc) preUvChannels = atoi(argv[i]);
        } else if (!strcmp(arg, "-post")) {
            if (++i < argc) postUvChannels = atoi(argv[i]);
        } else {
            printf("Ignoring unrecognized argument '%s'\n", arg);
        }
    }

    //
    //  Warnings about conflicting arguments or invalid values:
    //
    if (singleCreaseFlag) {
        printf("Ignoring -single (not yet supported)\n");
        singleCreaseFlag = false;
    }
    if ((uvInterpMask < 0) || (uvInterpMask > FVarInterpInfo::FullMask)) {
        printf("Ignoring invalid mask value for -uvint argument %d\n", uvInterpMask);
        uvInterpMask = FVarInterpInfo::FullMask;
    }

    //
    //  Load Obj file strings (parsing deferred):
    //
    for (size_t i = 0; i < objFiles.size(); ++i) {
        std::string const & fileName = objFiles[i];
        std::string         fileString;

        if (readString(fileName.c_str(), fileString)) {
            shapes.push_back(ShapeDesc(fileName.c_str(), fileString.c_str(), shapeScheme));
        } else {
            printf("Unable to open/read .obj file '%s'\n", fileName.c_str());
            exit(0);
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
    bool runFVarPerVertexTests = args.perVertexFlag;
    bool runFVarBoundaryTests  = args.boundaryFlag;

    bool runRefinementTests  = args.refinementFlag;
    bool runPatchSampleTests = args.patchSampleFlag;

    bool printTestInfo = true;

    //  Options related to shape representation and accuracy:
    bool useGregory = true;

    printf("\n");
    printf("Shape options:\n");
    printf("  - refinement depth    = %d\n", args.refineDepth);
    if (args.refineSecond < args.refineDepth) {
        printf("  - secondary depth   = %d\n", args.refineSecond);
    }
    printf("  - gregory patches     = %d\n", (int) useGregory);
    printf("  - inf-sharp patch     = %d\n", (int) args.infSharpFlag);
    printf("  - single-crease patch = %d\n", (int) args.singleCreaseFlag);
    printf("  - smooth corner patch = %d\n", (int) args.smoothCornerFlag);
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
        for (int i = 0; i < FVarInterpInfo::EnumCount; ++i) {
            if (!FVarInterpInfo::IsEnabled(i, args.uvInterpMask)) continue;
            printf("  - FVar values as assigned with linear interpolation '%s'\n", FVarInterpInfo::GetName(i));
            testsEnabled = true;
        }
    }
    if (!testsEnabled) {
        printf("    none\n");
        return EXIT_SUCCESS;
    }
    printf("\n");

    //  A few reminders/warnings that should eventually be deleted...
    if (args.infSharpFlag)
        printf("... warning -- enabled inf-sharp-patches (non default) ...\n\n");
    if (!args.infSharpFlag)
        printf("... warning -- inf-sharp-patches NOT enabled (may yield non-manifold discrepancies) ...\n\n");
    if (args.refineDepth <= 2)
        printf("... warning -- low refinement level (%d) is a weak test ...\n\n", args.refineDepth);
    if (args.preUvChannels)
        printf("... warning -- assigning %d pre-Uv channels ...\n\n", args.preUvChannels);
    if (args.postUvChannels)
        printf("... warning -- assigning %d post-Uv channels ...\n\n", args.postUvChannels);

    //
    //  Initialize some of the Option structs from potential command line
    //  specification.  The eventual intent it to be able to generate
    //  permutations of these to run variations of the tests for the same
    //  shapes -- particularly the irregular patch types BSpline and Gregory.
    //
    Far::TopologyRefiner::AdaptiveOptions adaptiveOptions(args.refineDepth);
    adaptiveOptions.secondaryLevel       = args.refineSecond;
    adaptiveOptions.useSingleCreasePatch = args.singleCreaseFlag;
    adaptiveOptions.useInfSharpPatch     = args.infSharpFlag;
    adaptiveOptions.considerFVarChannels = true;

    Far::TopologyRefiner::UniformOptions uniformOptions(args.refineDepth);

    Far::PatchTableFactory::Options patchOptions(args.refineDepth);
    patchOptions.useSingleCreasePatch            = args.singleCreaseFlag;
    patchOptions.useInfSharpPatch                = args.infSharpFlag;
    patchOptions.generateFVarTables              = true;
    patchOptions.generateFVarLegacyLinearPatches = false;
    patchOptions.shareEndCapPatchPoints          = true;
    patchOptions.endCapType                      = useGregory ?
                Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS :
                Far::PatchTableFactory::Options::ENDCAP_BSPLINE_BASIS;
    //  Need to update this branch before we can use this...
    patchOptions.generateLegacySharpCornerPatches = !args.smoothCornerFlag;

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
        bool doRefinementTests  = runRefinementTests;
        bool doPatchSampleTests = runPatchSampleTests && args.refineAdaptive;

        bool doFVarPerVertexTests = runFVarPerVertexTests;
        bool doFVarBoundaryTests  = runFVarBoundaryTests && originalMesh.HasUvs();

        std::vector<VertexDiff> fvarPerVertexRefinedValueDiffs;
        std::vector<PatchDiff>  fvarPerVertexPatchSampleDiffs;

        std::vector<FaceVertexDiff> fvarBoundaryRefinedValueDiffs[FVarInterpInfo::EnumCount];
        std::vector<PatchDiff>      fvarBoundaryPatchSampleDiffs[FVarInterpInfo::EnumCount];

        bool failuresDetected = false;

        //  Create a copy of the test mesh with a fvar-channel with per-vertex fvar values
        //  in place of the original
        //
        if (doFVarPerVertexTests) {
            TestMesh vertexMesh(originalMesh);

            vertexMesh.AssignEmptyChannels(args.preUvChannels, args.postUvChannels);
            vertexMesh.AssignUvChannelPerVertex();

            vertexMesh.Rebuild(args.refineAdaptive  ? &adaptiveOptions : 0,
                              !args.refineAdaptive  ? &uniformOptions : 0,
                               doPatchSampleTests   ? &patchOptions : 0);

            //  Compare refined vertex and fvar values (which should match) and sample points
            //  on the corresponding vertex and fvar patches:
            //
            if (doRefinementTests) {
                compareRefinedValues(vertexMesh, vertexMesh, args.tolerance,
                        fvarPerVertexRefinedValueDiffs);
                failuresDetected |= (fvarPerVertexRefinedValueDiffs.size() > 0);
            }
            if (doPatchSampleTests) {
                comparePatchSamples(vertexMesh, vertexMesh, args.sampleRate, args.tolerance,
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
            xyzMeshOriginalUvs.AssignEmptyChannels(args.preUvChannels, args.postUvChannels);

            TestMesh xyzMeshExpandedUvs(originalMesh);
            xyzMeshExpandedUvs.AssignEmptyChannels(args.preUvChannels, args.postUvChannels);
            xyzMeshExpandedUvs.ExpandUvChannel();

            TestMesh uvwMeshReference;
            uvwMeshReference.BuildFromUvChannel(xyzMeshExpandedUvs);

            for (int i = 0; i < FVarInterpInfo::EnumCount; ++i) {
                if (!FVarInterpInfo::IsEnabled(i, args.uvInterpMask)) continue;

                Sdc::Options::FVarLinearInterpolation fvarInterpType = FVarInterpInfo::GetType(i);

                xyzMeshOriginalUvs.AssignUvChannelInterpolation(fvarInterpType);

                TestMesh uvwMeshPerFVarOption(uvwMeshReference);
                uvwMeshPerFVarOption.SharpenFromUvChannel(xyzMeshExpandedUvs, fvarInterpType);

                if (doRefinementTests) {
                    xyzMeshOriginalUvs.Rebuild(0, &uniformOptions, 0);
                    uvwMeshPerFVarOption.Rebuild(0, &uniformOptions, 0);

//printf("Generating uniformly refined meshes...\n");
//xyzMeshOriginalUvs.WriteToObjFile("xyzMesh.obj", -1);
//uvwMeshPerFVarOption.WriteToObjFile("uvwMesh.obj", -1);

                    compareRefinedFaces(uvwMeshPerFVarOption, xyzMeshOriginalUvs, args.tolerance,
                            fvarBoundaryRefinedValueDiffs[i]);
                    failuresDetected |= (fvarBoundaryRefinedValueDiffs[i].size() > 0);
                }
                if (doPatchSampleTests) {
                    xyzMeshOriginalUvs.Rebuild(&adaptiveOptions, 0, &patchOptions);
                    uvwMeshPerFVarOption.Rebuild(&adaptiveOptions, 0, &patchOptions);

//printf("Generating base meshes...\n");
//xyzMeshOriginalUvs.WriteToObjFile("xyzMesh.obj", 0);
//uvwMeshPerFVarOption.WriteToObjFile("uvwMesh.obj", 0);

                    comparePatchSamples(uvwMeshPerFVarOption, xyzMeshOriginalUvs, args.sampleRate, args.tolerance,
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
            for (int i = 0; i < FVarInterpInfo::EnumCount; ++i) {
                if ((fvarBoundaryRefinedValueDiffs[i].size() > 0) ||
                    (fvarBoundaryPatchSampleDiffs[i].size() > 0)) {

                    printf("    FVar interpolation option '%s':\n", FVarInterpInfo::GetName(i));
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
