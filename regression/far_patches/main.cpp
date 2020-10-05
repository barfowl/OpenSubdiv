//
//   Copyright 2018 DreamWorks Animation LLC.
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

#include "init_shapes.h"

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

    Args() { initialize(); }
    Args(int argc, char **argv);
    ~Args() { }

public:
    //  options related to the testing:
    bool testPosition;
    bool testNormal;
    bool testUV;
    bool verboseFlag;

    //  options related to the approximation of shapes:
    int            refineLevel;
    int            refineSecond;
    IrregularBasis irregBasis;

    //  options related to given or specified shapes:
    int                    shapeCount;
    Scheme                 shapeScheme;
    std::string            shapeName;
    std::vector<ShapeDesc> shapes;

private:
    void initialize() {
        testPosition = true;
        testNormal   = true;
        testUV       = true;
        verboseFlag  = true;

        refineLevel  = 3;
        refineSecond = 15;
        irregBasis   = Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS;

        shapeCount  = 0;
        shapeScheme = kCatmark;
    }
};

Args::Args(int argc, char **argv) {

    initialize();

    std::vector<char const *> objfiles;

    for (int i = 1; i < argc; ++i) {
        char * arg = argv[i];

        if (!strcmp(arg, "-shape")) {
            if (++i < argc) shapeName = std::string(arg);
        } else if (!strcmp(arg, "-count")) {
            if (++i < argc) shapeCount = atoi(argv[i]);
        } else if (!strcmp(arg, "-nopos")) {
            testPosition = false;
        } else if (!strcmp(arg, "-nonorm")) {
            testNormal = false;
        } else if (!strcmp(arg, "-nouv")) {
            testUV = false;
        } else if (!strcmp(arg, "-terse")) {
            verboseFlag = false;
        } else if (!strcmp(arg, "-verbose")) {
            verboseFlag = true;
        } else if (!strcmp(arg, "-level")) {
            if (++i < argc) refineLevel = atoi(argv[i]);
        } else if (!strcmp(arg, "-second")) {
            if (++i < argc) refineSecond = atoi(argv[i]);
        } else if (!strcmp(arg, "-linear")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_BILINEAR_BASIS;
        } else if (!strcmp(arg, "-regular")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_BSPLINE_BASIS;
        } else if (!strcmp(arg, "-gregory")) {
            irregBasis = Far::PatchTableFactory::Options::ENDCAP_GREGORY_BASIS;
        } else if (!strcmp(arg, "-bilinear")) {
            shapeScheme = kBilinear;
        } else if (!strcmp(arg, "-catmark")) {
            shapeScheme = kCatmark;
        } else if (!strcmp(arg, "-loop")) {
            shapeScheme = kLoop;
        } else if (strstr(arg, ".obj")) {
            objfiles.push_back(arg);
        } else {
            printf("Ignoring unrecognized argument '%s'\n", arg);
        }
    }

    std::string fileString;
    for (int i = 0; i < (int)objfiles.size(); ++i) {
        if (readString(objfiles[i], fileString)) {
            shapes.push_back(ShapeDesc(objfiles[i], fileString.c_str(), shapeScheme));
        } else {
            printf("Unable to open/read .obj file '%s'\n", objfiles[i]);
            exit(0);
        }
    }
}

//
//  Simple primvar data struct with required interface for interpolation:
//
struct Coord3 {
    Coord3() { }
    Coord3(float a, float b, float c) : x(a), y(b), z(c) { }

    float const * Coords() const { return &x; }
    float       * Coords()       { return &x; }

    void Clear()                                  { x  = 0.0f,  y  = 0.0f,  z  = 0.0f; }
    void Add(          Coord3 const & p         ) { x +=   p.x, y +=   p.y, z +=   p.z; }
    void Subtract(     Coord3 const & p         ) { x -=   p.x, y -=   p.y, z -=   p.z; }
    void AddWithWeight(Coord3 const & p, float s) { x += s*p.x, y += s*p.y, z += s*p.z; }
    void Scale(                          float s) { x *= s,     y *= s,     z *= s;     }

    Coord3 operator+(Coord3 const & p) { Coord3 q = *this; q.Add(p);      return q; }
    Coord3 operator-(Coord3 const & p) { Coord3 q = *this; q.Subtract(p); return q; }
    float Length() const {
        return sqrtf(x*x + y*y + z*z);
    }
    Coord3 Normalize(float tol = 1e-6) const {
        float len = Length();
        if (len < tol) {
            return Coord3(0, 0, 0);
        } else {
            return Coord3(x / len, y / len, z / len);
        }
    }

    static Coord3 Cross(Coord3 const & a, Coord3 const & b) {
        return Coord3(a.y * b.z - a.z * b.y,
                      a.z * b.x - a.x * b.z,
                      a.x * b.y - a.y * b.x);
    }

    static Coord3 Normal(Coord3 const & du, Coord3 const & dv, float tol = 1e-6) {
        return Cross(du, dv).Normalize(tol);
    }

private:
    float x, y, z;
};
typedef std::vector<Coord3> Coord3Vector;

bool
doCoord3sDiffer(Coord3 const & a, Coord3 const & b, float tol) {

    float const * aCoords = a.Coords();
    float const * bCoords = b.Coords();

    return (std::abs(aCoords[0] - bCoords[0]) > tol) ||
           (std::abs(aCoords[1] - bCoords[1]) > tol) ||
           (std::abs(aCoords[2] - bCoords[2]) > tol);
}

int
compareCoord3Arrays(int size, Coord3 const* a, Coord3 const* b, float tol) {

    int nDiffs = 0;
    for (int i = 0; i < size; ++i) {
        nDiffs += doCoord3sDiffer(a[i], b[i], tol);
    }
    return nDiffs;
}

int
compareCoord3Vectors(Coord3Vector const& a, Coord3Vector const& b, float tol) {
    assert(a.size() == b.size());

    return compareCoord3Arrays((int)a.size(), &a[0], &b[0], tol);
}

struct RefinedData {
    RefinedData() : fvarPresent(false) {}
    RefinedData(Far::TopologyRefiner const & refiner, Shape const & shape) {

        int nVtxBase = refiner.GetLevel(0).GetNumVertices();
        vtxBase.resize(nVtxBase);
        for (int i = 0; i < nVtxBase; ++i) {
            float const * shapePos = &shape.verts[i * 3];
            vtxBase[i] = Coord3(shapePos[0], shapePos[1], shapePos[2]);
        }

        fvarPresent = refiner.GetNumFVarChannels() > 0;
        if (fvarPresent) {
            int nFVarBase = refiner.GetLevel(0).GetNumFVarValues(0);
            fvarBase.resize(nFVarBase);
            for (int i = 0; i < nFVarBase; ++i) {
                float const * shapeUV = &shape.uvs[i * 2];
                fvarBase[i] = Coord3(shapeUV[0], shapeUV[1], 0.0f);
            }
        }
    }

    void
    Refine(Far::TopologyRefiner const & refiner) {

        vtxRefined  = vtxBase;
        vtxRefined.resize(refiner.GetNumVerticesTotal());

        if (fvarPresent) {
            fvarRefined = fvarBase;
            fvarRefined.resize(refiner.GetNumFVarValuesTotal(0));
        }

        int maxLevel = refiner.GetMaxLevel();
        if (maxLevel > 0) {
            Far::PrimvarRefiner primvarRefiner(refiner);

            Coord3 * vtxSrc  = &vtxRefined[0];
            Coord3 * fvarSrc = fvarPresent ? &fvarRefined[0] : 0;

            for (int level = 1; level <= maxLevel; ++level) {
                Coord3 * vtxDst = vtxSrc + refiner.GetLevel(level-1).GetNumVertices();

                primvarRefiner.Interpolate(level, vtxSrc, vtxDst);

                if (fvarPresent) {
                    Coord3 * fvarDst = fvarSrc + refiner.GetLevel(level-1).GetNumFVarValues();
                    primvarRefiner.InterpolateFaceVarying(level, fvarSrc, fvarDst);
                    fvarSrc = fvarDst;
                }

                vtxSrc = vtxDst;
            }
        }
    }

    void
    Limit(Far::TopologyRefiner const & refiner) {

        Far::PrimvarRefiner primvarRefiner(refiner);

        Far::TopologyLevel const & lastLevel = refiner.GetLevel(refiner.GetMaxLevel());

        bool vtxPresent = true;
        if (vtxPresent) {
            int nLimitPoints = lastLevel.GetNumVertices();

            vtxLimitPos.resize(nLimitPoints);
            vtxLimitTan1.resize(nLimitPoints);
            vtxLimitTan2.resize(nLimitPoints);
            vtxLimitNorm.resize(nLimitPoints);

            int vtxLast = refiner.GetNumVerticesTotal() - nLimitPoints;
            Coord3 const * vtxSrc = &vtxRefined[vtxLast];
            Coord3 *       vtxPos = &vtxLimitPos[0];
            primvarRefiner.Limit(vtxSrc, vtxPos, vtxLimitTan1, vtxLimitTan2);

            for (int i = 0; i < nLimitPoints; ++i) {
                vtxLimitNorm[i] = Coord3::Normal(vtxLimitTan1[i], vtxLimitTan2[i]);
            }
        }
        if (fvarPresent) {
            int nLimitPoints = lastLevel.GetNumFVarValues(0);

            fvarLimitPos.resize(nLimitPoints);

            int fvarLast = refiner.GetNumFVarValuesTotal(0) - nLimitPoints;
            Coord3 const * fvarSrc = &fvarRefined[fvarLast];
            primvarRefiner.LimitFaceVarying(fvarSrc, fvarLimitPos);
        }
    }


    Coord3Vector vtxBase;
    Coord3Vector vtxRefined;
    Coord3Vector vtxLimitPos;
    Coord3Vector vtxLimitTan1;
    Coord3Vector vtxLimitTan2;
    Coord3Vector vtxLimitNorm;

    bool         fvarPresent;
    Coord3Vector fvarBase;
    Coord3Vector fvarRefined;
    Coord3Vector fvarLimitPos;
};

struct PatchData {
    PatchData() { }
    PatchData(Far::PatchTable const & patchTable, RefinedData const & refPoints) {

        vtxLocalPoints = refPoints.vtxRefined;
        if (patchTable.GetNumLocalPoints()) {
            int nVtxRefined = refPoints.vtxRefined.size();
            vtxLocalPoints.resize(nVtxRefined + patchTable.GetNumLocalPoints());

            patchTable.GetLocalPointStencilTable()->UpdateValues(
                        &vtxLocalPoints[0], &vtxLocalPoints[nVtxRefined]);
        }

        fvarLocalPoints = refPoints.fvarRefined;
        if (patchTable.GetNumLocalPointsFaceVarying()) {
            int nFVarRefined = refPoints.fvarRefined.size();
            fvarLocalPoints.resize(nFVarRefined + patchTable.GetNumLocalPointsFaceVarying());

            patchTable.GetLocalPointFaceVaryingStencilTable()->UpdateValues(
                        &fvarLocalPoints[0], &fvarLocalPoints[nFVarRefined]);
        }
    }

    Coord3Vector vtxLocalPoints;
    Coord3Vector fvarLocalPoints;
};

bool
isVertexIncidentSemiSharpFeature(Far::TopologyLevel const & level, Index v) {

    if (Sdc::Crease::IsSemiSharp(level.GetVertexSharpness(v))) {
        return true;
    }
    ConstIndexArray vEdges = level.GetVertexEdges(v);
    for (int j = 0; j < vEdges.size(); ++j) {
        if (Sdc::Crease::IsSemiSharp(level.GetEdgeSharpness(vEdges[j]))) {
            return true;
        }
    }
    return false;
}
bool
isVertexIncidentInteriorInfSharpFeature(Far::TopologyLevel const & level, Index v) {

    if (Sdc::Crease::IsInfinite(level.GetVertexSharpness(v))) {
        if (level.GetVertexFaces(v).size() > 1) {
            return true;
        }
    }
    ConstIndexArray vEdges = level.GetVertexEdges(v);
    for (int j = 0; j < vEdges.size(); ++j) {
        if (Sdc::Crease::IsInfinite(level.GetEdgeSharpness(vEdges[j]))) {
            if (!level.IsVertexBoundary(v)) {
                return true;
            }
        }
    }
    return false;
}

int
main(int argc, char **argv) {

    Args args(argc, argv);

    //
    //  Options controlling the tests and shapes -- move more of these variables
    //  into members of the command line Args class parsed above...
    //
    //  Options related to testing:
    bool  printOptions  = true;
    bool  printTestInfo = args.verboseFlag;
    bool  printSummary  = true;

    //  Options related to shape representation and accuracy:
    bool  useInfSharp     = true;
    bool  useSmoothCorner = true;
    bool  useSingleCrease = false;  // still ignored in CPU patch evaluation...
    bool  legacyFVar      = false;
    bool  forceSmoothFVar = true;

    //  Tolerances used for comparison -- position tolerance should vary with shape:
    float posTol  = 1.0e-5;
    float normTol = 1.0e-4;
    float uvTol   = 1.0e-5;

    if (printOptions) {
        char const * irregBasisStrings[4] = { "None", "Linear", "Regular", "Gregory" };

        printf("\n");
        printf("Test options:\n");
        printf("  - Vertex positions    = %d\n", args.testPosition);
        printf("  - Vertex normals      = %d\n", args.testNormal);
        printf("  - Face-Varying values = %d\n", args.testUV);

        printf("\n");
        printf("Shape options:\n");
        printf("  - refinement level      = %d\n", args.refineLevel);
        if (args.refineSecond < args.refineLevel) {
            printf("  - secondary level   = %d\n", args.refineSecond);
        }
        printf("  - inf-sharp patches     = %d\n", (int) useInfSharp);
        printf("  - smooth corner patches = %d\n", (int) useSmoothCorner);
        printf("  - irregular patch basis = %s\n", irregBasisStrings[args.irregBasis]);
        printf("  - legacy linear FVar    = %d\n", (int) legacyFVar);
        printf("  - force smooth FVar     = %d\n", (int) forceSmoothFVar);
        if (legacyFVar && forceSmoothFVar) {
            printf("WARNING - conflicting FVar options, one ignored.\n");
        }
        if (useSingleCrease) {
            //  This option is a place-holder for future support...
            printf("WARNING - single-crease patch still not supported, ignored.\n");
            useSingleCrease = false;
        }
        printf("\n");
    }

    //
    //  Initialize Option structs from command line specification:
    //
    //  For uniform PatchTable and refinement:
    Far::PatchTableFactory::Options uniformPatchOptions(args.refineLevel);
    uniformPatchOptions.generateAllLevels  = false;
    uniformPatchOptions.generateFVarTables = true;

    Far::TopologyRefiner::UniformOptions uniformRefineOptions(args.refineLevel);
    uniformRefineOptions.fullTopologyInLastLevel = true;

    //  For adaptive PatchTable and refinement (latter depends on former):
    Far::PatchTableFactory::Options adaptPatchOptions(args.refineLevel);
    adaptPatchOptions.generateAllLevels               = true;
    adaptPatchOptions.generateFVarTables              = true;
    adaptPatchOptions.generateFVarLegacyLinearPatches = legacyFVar && !forceSmoothFVar;

    adaptPatchOptions.useInfSharpPatch                 = useInfSharp;
    adaptPatchOptions.generateLegacySharpCornerPatches = !useSmoothCorner;
    adaptPatchOptions.useSingleCreasePatch             = useSingleCrease;
    adaptPatchOptions.endCapType                       = args.irregBasis;
    adaptPatchOptions.shareEndCapPatchPoints           = true;

    Far::TopologyRefiner::AdaptiveOptions adaptRefineOptions(args.refineLevel);
    adaptRefineOptions.secondaryLevel       = args.refineSecond;
    adaptRefineOptions.useInfSharpPatch     = adaptPatchOptions.useInfSharpPatch;
    adaptRefineOptions.useSingleCreasePatch = adaptPatchOptions.useSingleCreasePatch;
    adaptRefineOptions.considerFVarChannels = adaptPatchOptions.generateFVarTables &&
                                             !adaptPatchOptions.generateFVarLegacyLinearPatches;

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
        if (!shape->uvs.empty() && forceSmoothFVar) {
            sdcOptions.SetFVarLinearInterpolation(Sdc::Options::FVAR_LINEAR_CORNERS_ONLY);
        }

        Far::TopologyRefiner * baseRefiner =
                Far::TopologyRefinerFactory<Shape>::Create(*shape,
                        Far::TopologyRefinerFactory<Shape>::Options(sdcType, sdcOptions));

        if (baseRefiner == 0) {
            printf("Warning -- shape '%s' ignored (unable to construct refiner)\n",
                    shapeName.c_str());
            delete shape;
            continue;
        }
        bool testShapePositions = args.testPosition;
        bool testShapeNormals   = args.testNormal && (sdcType != Sdc::SCHEME_BILINEAR);
        bool testShapeUVs       = args.testUV     && (baseRefiner->GetNumFVarChannels() > 0);

        //
        //  Refine uniform and adaptive and populate data buffers with both
        //  refined and -- in the case of uniform -- limit points:
        //
        Far::TopologyRefiner * uniformRefiner =
                Far::TopologyRefinerFactory<Shape>::Create(*baseRefiner);
        Far::TopologyRefiner * adaptRefiner =
                Far::TopologyRefinerFactory<Shape>::Create(*baseRefiner);

        assert(uniformRefiner && adaptRefiner);

        uniformRefiner->RefineUniform(uniformRefineOptions);
        adaptRefiner->RefineAdaptive(adaptRefineOptions);

        RefinedData uniformData(*baseRefiner, *shape);
        uniformData.Refine(*uniformRefiner);
        uniformData.Limit(*uniformRefiner);

        RefinedData adaptData(*baseRefiner, *shape);
        adaptData.Refine(*adaptRefiner);

        //
        //  Create uniform and adaptive patch tables -- uniform is only
        //  needed to obtain PatchParam's for correspondence:
        //
        Far::PatchTable * uniformPatches =
                Far::PatchTableFactory::Create(*uniformRefiner, uniformPatchOptions);

        Far::PatchTable * adaptPatches =
                Far::PatchTableFactory::Create(*adaptRefiner, adaptPatchOptions);

        PatchData patchData(*adaptPatches, adaptData);

        Far::PatchMap patchMap(*adaptPatches);

        //
        //  Compute points for patches at the locations of uniformly refined points:
        //
        int regFaceSize = (sdcType == Sdc::SCHEME_LOOP) ? 3 : 4;

        float const quadCorners[][2] = {{0,0}, {1,0}, {1,1}, {0,1}};
        float const triCorners[][2]  = {{0,0}, {1,0}, {0,1}};

        float const (*stCorners)[2] = (regFaceSize == 3) ? triCorners : quadCorners;

        Far::TopologyLevel const & uniformLevel =
                uniformRefiner->GetLevel(uniformRefiner->GetMaxLevel());

        int nUniformFaces = uniformLevel.GetNumFaces();

        Coord3Vector const & vtxCoords  = patchData.vtxLocalPoints;
        Coord3Vector const & fvarCoords = patchData.fvarLocalPoints;

        int nPosDiffs  = 0;
        int nNormDiffs = 0;
        int nFVarDiffs = 0;

//printf("\nPatch face evaluation, regFaceSize = %d...\n", regFaceSize);
        int facePatchIndex = 0;
        for (int face = 0; face < nUniformFaces; ++face) {
            if (uniformLevel.IsFaceHole(face)) continue;

            ConstIndexArray fVerts   = uniformLevel.GetFaceVertices(face);
            ConstIndexArray fEdges   = uniformLevel.GetFaceEdges(face);
            ConstIndexArray fvValues = testShapeUVs ? uniformLevel.GetFaceFVarValues(face)
                                     : ConstIndexArray();

            Far::PatchParam patchParam = uniformPatches->GetPatchParam(0, facePatchIndex);
            Index           patchFace  = patchParam.GetFaceId();
//printf("    face %d, uniform patch %d, patch face %d:\n", face, facePatchIndex, patchFace);

            for (int i = 0; i < regFaceSize; ++i) {
                bool testThisPosition = testShapePositions;
                bool testThisNormal   = testShapeNormals;
                bool testThisUV       = testShapeUVs;

                //
                //  Don't test anything at semi-sharp features and don't test normals
                //  at inf-sharp features or Bilinear shapes:
                //
                if (isVertexIncidentSemiSharpFeature(uniformLevel, fVerts[i])) {
                    continue;
                }
                if (isVertexIncidentInteriorInfSharpFeature(uniformLevel, fVerts[i])) {
                    testThisNormal = false;
                }

                //
                //  Map normalized parametric corner locations to the patch interior:
                //
                float s = stCorners[i][0];
                float t = stCorners[i][1];
                if (regFaceSize == 4) {
                    patchParam.Unnormalize(s, t);
                } else {
                    patchParam.UnnormalizeTriangle(s, t);
                }

                Far::PatchTable::PatchHandle const * handle = patchMap.FindPatch(patchFace, s, t);

                //  Evaluate position, normal and/or UV and compare to limit values:
                if (testThisPosition || testThisNormal) {
                    float wP[20], wDu[20], wDv[20];
                    adaptPatches->EvaluateBasis(*handle, s, t, wP, wDu, wDv);
                    ConstIndexArray cvs = adaptPatches->GetPatchVertices(*handle);

                    Coord3 P(0,0,0);
                    Coord3 du(0,0,0);
                    Coord3 dv(0,0,0);
                    for (int k = 0; k < cvs.size(); ++k) {
                        P.AddWithWeight(vtxCoords[cvs[k]], wP[k]);
                        du.AddWithWeight(vtxCoords[cvs[k]], wDu[k]);
                        dv.AddWithWeight(vtxCoords[cvs[k]], wDv[k]);
                    }
                    Coord3 N = Coord3::Normal(du, dv, normTol);

                    //  Don't test this normal if degenerate:
                    if (N.Length() < normTol) {
                        testThisNormal = false;
                    }
                    //  Resolve degenerate normal:
                    if (N.Length() < normTol) {
//printf("    resolving deg normal on patch face %d (%f, %f)...\n", patchFace, s, t);
                        float wDuu[20], wDuv[20], wDvv[20];
                        adaptPatches->EvaluateBasis(*handle, s, t, wP, wDu, wDv, wDuu, wDuv, wDvv);

                        Coord3 duu(0,0,0);
                        Coord3 duv(0,0,0);
                        Coord3 dvv(0,0,0);
                        for (int k = 0; k < cvs.size(); ++k) {
                            duu.AddWithWeight(vtxCoords[cvs[k]], wDuu[k]);
                            duv.AddWithWeight(vtxCoords[cvs[k]], wDuv[k]);
                            dvv.AddWithWeight(vtxCoords[cvs[k]], wDvv[k]);
                        }
                        Coord3 dNu = Coord3::Cross(duu, dv) + Coord3::Cross(du, duv);
                        Coord3 dNv = Coord3::Cross(duv, dv) + Coord3::Cross(du, dvv);

                        float usign = (s < 1.0f) ? 1.0f : -1.0f;
                        float vsign = (t < 1.0f) ? 1.0f : -1.0f;

                        dNu.Scale(usign);
                        dNv.Scale(vsign);

                        N = dNu + dNv;
                        N = N.Normalize(normTol);

                    }

                    if (testThisPosition) {
                        Coord3 refP = uniformData.vtxLimitPos[fVerts[i]];
                        nPosDiffs += doCoord3sDiffer(P, refP, posTol);
                    }
                    if (testThisNormal) {
                        Coord3 refN = uniformData.vtxLimitNorm[fVerts[i]];
                        nNormDiffs += doCoord3sDiffer(N, refN, normTol);

bool printNormDiff = false;
if (doCoord3sDiffer(N, refN, normTol) && printNormDiff) {
printf("    Normal diff on patch face %d (%f, %f):\n", patchFace, s, t);
float const * a = N.Coords();
float const * b = refN.Coords();
printf("        N-patch = %9.6f %9.6f %9.6f\n", a[0], a[1], a[2]);
printf("        N-limit = %9.6f %9.6f %9.6f\n", b[0], b[1], b[2]);

}
                    }
                }
                if (testThisUV) {
                    float w[20];
                    adaptPatches->EvaluateBasisFaceVarying(*handle, s, t, w);
                    ConstIndexArray cvs = adaptPatches->GetPatchFVarValues(*handle, 0);

                    Coord3 fvar(0,0,0);
                    for (int k = 0; k < cvs.size(); ++k) {
                        fvar.AddWithWeight(fvarCoords[cvs[k]], w[k]);
                    }

                    Coord3 refFVar = uniformData.fvarLimitPos[fvValues[i]];
                    nFVarDiffs += doCoord3sDiffer(fvar, refFVar, uvTol);
                    if (doCoord3sDiffer(fvar, refFVar, uvTol)) {
                    printf("UV patch vs limit - face %d (%f,%f):  (%f %f) vs (%f %f)\n",
                        face, s, t, fvar.Coords()[0], fvar.Coords()[1],
                        refFVar.Coords()[0], refFVar.Coords()[1]);
                    }
                }
            }
            ++facePatchIndex;
        }

        //
        //  Compare patch evaluations to uniformly refined limit points and report diffs:
        //  
        if (nPosDiffs || nNormDiffs || nFVarDiffs) {
            ++ failedShapes;
            if (printTestInfo) {
                printf("    Differences detected:\n");
            } else {
                printf("Failures detected for shape '%s':\n", shapeName.c_str());
            }
            if (nPosDiffs) {
                printf("        %d evaluated vertex positions differ\n", nPosDiffs);
            }
            if (nNormDiffs) {
                printf("        %d evaluated vertex normals differ\n", nNormDiffs);
            }
            if (nFVarDiffs) {
                printf("        %d evaluated face-varying results differ\n", nFVarDiffs);
            }
            printf("\n");
        }

        delete adaptPatches;
        delete uniformPatches;
        delete adaptRefiner;
        delete uniformRefiner;
        delete baseRefiner;
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
