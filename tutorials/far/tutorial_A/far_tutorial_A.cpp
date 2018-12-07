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

//
//  Description:
//      This tutorial illustrates the use of the new Far::BasePatch class -- a
//      hierarchical patch that defines the limit surface for the base face of
//      a mesh.  BasePatches are implemented with the new Far::PatchTree class,
//      which are topologically independent and can be cached and shared for 
//      use with all faces of the mesh that share the same topology.
//

#include "../../../regression/common/far_utils.h"

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/patchTreeFactory.h>
#include <opensubdiv/far/patchTreeCache.h>
#include <opensubdiv/far/patchTree.h>
#include <opensubdiv/far/basePatchFactory.h>
#include <opensubdiv/far/basePatch.h>

#include <cassert>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

using namespace OpenSubdiv;

using Far::Index;
using Far::IndexArray;
using Far::ConstIndexArray;


//
//  Global utilities in this namespace are not relevant to the tutorial.
//  They simply serve to construct some default geometry to be processed
//  in the form of a TopologyRefiner and vector of vertex positions.
//
namespace {
    //
    //  Simple interpolatable struct for (x,y,z) positions and normals:
    //
    struct Coord3 {
        Coord3() { }
        Coord3(float x, float y, float z) { p[0] = x, p[1] = y, p[2] = z; }

        //  Clear() and AddWithWeight() required for interpolation:
        void Clear( void * =0 ) { p[0] = p[1] = p[2] = 0.0f; }

        void AddWithWeight(Coord3 const & src, float weight) {
            p[0] += weight * src.p[0];
            p[1] += weight * src.p[1];
            p[2] += weight * src.p[2];
        }

        //  Compute normal vector:
        static Coord3 ComputeNormal(Coord3 const & Du, Coord3 const & Dv) {
            Coord3 N(Du.p[1]*Dv.p[2] - Du.p[2]*Dv.p[1],
                     Du.p[2]*Dv.p[0] - Du.p[0]*Dv.p[2],
                     Du.p[0]*Dv.p[1] - Du.p[1]*Dv.p[0]);
            float lenSqrd = N.p[0]*N.p[0] + N.p[1]*N.p[1] + N.p[2]*N.p[2];
            if (lenSqrd > 0.0f) {
                float len = std::sqrt(lenSqrd);
                return Coord3(N.p[0]/len, N.p[1]/len, N.p[2]/len);
            } else {
                return Coord3(0.0f, 0.0f, 0.0f);
            }
        }

        //  Member variables (coordinates):
        float p[3];
    };
    typedef std::vector<Coord3> Coord3Vector;


    //
    //  Simple class for a tessellation pattern, including simple public
    //  structs for a parameteric location and tessellated face.  Two
    //  uniform tessellation patterns can be constructed -- one for quads
    //  and one for triangles.
    //
    class TessPattern {
    public:
        struct Param {
            Param() { uv[0] = uv[1] = -1.0; }
            float uv[2];
        };
        typedef std::vector<Param> ParamVector;

        struct Face {
            Face() { v[0] = v[1] = v[2] = v[3] = -1; }
            int v[4];
        };
        typedef std::vector<Face> FaceVector;

    public:
        TessPattern() { }
        ~TessPattern() { }

        void BuildUniformQuads(int res);
        void BuildUniformTris(int res);

        int GetNumVertices() const { return (int) _params.size(); }
        int GetNumFaces() const    { return (int) _faces.size(); }

        ParamVector const & GetParamVector() const { return _params; }
        FaceVector const &  GetFaceVector() const  { return _faces; }

    private:
        ParamVector _params;
        FaceVector  _faces;
    };

    void
    TessPattern::BuildUniformQuads(int res) {

        int edgesPerSide = res;
        int pointsPerSide = edgesPerSide + 1;

        int nPoints = pointsPerSide * pointsPerSide;
        int nFaces  = edgesPerSide * edgesPerSide;

        //
        //  Initialize the parametric locations for each sample point:
        //
        _params.resize(nPoints);

        float uvDelta = 1.0 / (float)edgesPerSide;

        ParamVector::iterator p = _params.begin();

        for (int i = 0; i < pointsPerSide; ++i) {
            for (int j = 0; j < pointsPerSide; ++j, ++p) {
                (*p).uv[0] = (float)i * uvDelta;
                (*p).uv[1] = (float)j * uvDelta;
            }
        }

        //
        //  Initialize the face vertex indices (all quads):
        //
        _faces.resize(nFaces);

        FaceVector::iterator f = _faces.begin();

        for (int i = 0; i < edgesPerSide; ++i) {
            for (int j = 0; j < edgesPerSide; ++j, ++f) {
                (*f).v[0] =   i   * pointsPerSide +   j;
                (*f).v[1] = (i+1) * pointsPerSide +   j;
                (*f).v[2] = (i+1) * pointsPerSide + (j+1);
                (*f).v[3] =   i   * pointsPerSide + (j+1);
            }
        }
    }

    void
    TessPattern::BuildUniformTris(int res) {

        int edgesPerSide = res;
        int pointsPerSide = edgesPerSide + 1;

        int nPoints = pointsPerSide * (pointsPerSide + 1) / 2;
        int nFaces  = edgesPerSide * edgesPerSide;

        //
        //  Initialize the parametric locations for each sample point:
        //
        _params.resize(nPoints);

        float uvDelta = 1.0 / (float)edgesPerSide;

        ParamVector::iterator p = _params.begin();

        for (int i = 0; i < pointsPerSide; ++i) {
            for (int j = 0; j < (pointsPerSide - i); ++j, ++p) {
                (*p).uv[0] = (j == edgesPerSide) ? 1.0f : (uvDelta * (float)j);
                (*p).uv[1] = (i == edgesPerSide) ? 1.0f : (uvDelta * (float)i);
            }
        }

        //
        //  Initialize the face vertex indices (all triangles):
        //
        //  A uniform tessellation of a triangle into triangles begins at the base
        //  of the triangular patch and progresses upward.  A row of triangles is
        //  created for each edge per side -- consisting of a sequence of "upright"
        //  triangles (apex up, base down) interleaved with one less "inverted"
        //  triangles (base up, apex down).
        //
        _faces.resize(nFaces);

        FaceVector::iterator f = _faces.begin();

        int rowPoint = 0;
        for (int row = 0; row < edgesPerSide; ++row) {
            int nUprightTris  = edgesPerSide - row;
            int nInvertedTris = nUprightTris - 1;

            for (int i = 0, j = rowPoint; i < nUprightTris; ++i, ++j, ++f) {
                (*f).v[0] = j;
                (*f).v[1] = j + 1;
                (*f).v[2] = j + 1 + nUprightTris;
            }
            for (int i = 0, j = rowPoint + 1; i < nInvertedTris; ++i, ++j, ++f) {
                (*f).v[0] = j;
                (*f).v[1] = j + nUprightTris + 1;
                (*f).v[2] = j + nUprightTris;
            }
            rowPoint += nUprightTris + 1;
        }
    }


    //
    //  Create a TopologyRefiner from a specified Obj file:
    //
    Far::TopologyRefiner *
    createTopologyRefinerFromObj(std::string const & objFileName,
                                 Sdc::SchemeType schemeType,
                                 Coord3Vector & posVector) {

        const char *  filename = objFileName.c_str();
        const Shape * shape = 0;

        std::ifstream ifs(filename);
        if (ifs) {
            std::stringstream ss;
            ss << ifs.rdbuf();
            ifs.close();
            std::string shapeString = ss.str();

            shape = Shape::parseObj(
                shapeString.c_str(), ConvertSdcTypeToShapeScheme(schemeType), false);
            if (shape == 0) {
                fprintf(stderr, "Error:  Cannot create Shape from .obj file '%s'\n", filename);
                return 0;
            }
        } else {
            fprintf(stderr, "Error:  Cannot open .obj file '%s'\n", filename);
            return 0;
        }

        Sdc::SchemeType sdcType    = GetSdcType(*shape);
        Sdc::Options    sdcOptions = GetSdcOptions(*shape);

        Far::TopologyRefiner * refiner = Far::TopologyRefinerFactory<Shape>::Create(*shape,
            Far::TopologyRefinerFactory<Shape>::Options(sdcType, sdcOptions));
        if (refiner == 0) {
            fprintf(stderr,
                "Error:  Unable to construct TopologyRefiner from .obj file '%s'\n",
                filename);
            return 0;
        }

        int numVertices = refiner->GetNumVerticesTotal();
        posVector.resize(numVertices);
        std::memcpy(&posVector[0], &shape->verts[0], numVertices * 3 * sizeof(float));

        return refiner;
    }
} // end namespace


//
//  The MeshFaceEvaluator is a simple client wrapper that creates a Far::BasePatch
//  for a given face and associates primvar values for the control points of that
//  face for evaluation (could easily be templated for primvar type):
//
class MeshFaceEvaluator {
public:
    typedef Far::BasePatchFactory::Options Options;

public:
    MeshFaceEvaluator(Far::TopologyRefiner const & meshTopology,
                      Index                        meshFace,
                      std::vector<Coord3> const &  meshCoords,
                      Options const &              options);
    ~MeshFaceEvaluator() { delete _basePatch; }

    void Evaluate(float u, float v, Coord3 & P, Coord3 & Du, Coord3 & Dv,
                  int evalDepth = -1) const;

private:
    Index            _baseFace;
    Far::BasePatch * _basePatch;

    std::vector<Coord3> _patchCoords;
};

MeshFaceEvaluator::MeshFaceEvaluator(Far::TopologyRefiner const & meshTopology,
                                     Index                        meshFace,
                                     std::vector<Coord3> const &  meshCoords,
                                     Options const &              options) :
        _baseFace(meshFace), _basePatch(0) {

    //  Construct the BasePatch and initialize the required patch point values:
    _basePatch = Far::BasePatchFactory::Create(meshTopology, meshFace, options);

    _patchCoords.resize(_basePatch->GetNumPointsTotal());

    //  The following intialization can also be used to refresh the patch points:
    _basePatch->GetControlPointValues(meshCoords, _patchCoords);

    if (_basePatch->GetNumSubPatchPoints()) {
        _basePatch->GetSubPatchPointStencilTable()->UpdateValues(_patchCoords,
                _patchCoords, _basePatch->GetNumControlPoints());
    }
}

void
MeshFaceEvaluator::Evaluate(float u, float v, Coord3 & P, Coord3 & Du, Coord3 & Dv,
                            int evalDepth) const {

    int patchIndex = _basePatch->FindSubPatch(u, v, evalDepth);
    assert(patchIndex >= 0);

    float wP[20], wDu[20], wDv[20];
    _basePatch->EvalSubPatchBasis(patchIndex, u, v, wP, wDu, wDv);

    Far::ConstIndexArray cvIndices = _basePatch->GetSubPatchPoints(patchIndex);

    P.Clear();
    Du.Clear();
    Dv.Clear();
    for (int cv = 0; cv < cvIndices.size(); ++cv) {
        P.AddWithWeight(_patchCoords[cvIndices[cv]], wP[cv]);
        Du.AddWithWeight(_patchCoords[cvIndices[cv]], wDu[cv]);
        Dv.AddWithWeight(_patchCoords[cvIndices[cv]], wDv[cv]);
    }
}


//
//  Command line arguments parsed to provide run-time options:
//
class Args {
public:
    typedef Far::PatchTreeFactory::Options::BasisType BasisType;
public:
    std::string     inputObjFile;
    Sdc::SchemeType schemeType;
    int             maxPatchDepth;
    int             evalDepth;
    BasisType       irregularBasis;
    int             uniformRes;
    bool            noTessFlag;
    bool            noOutputFlag;

public:
    Args(int argc, char ** argv) :
        inputObjFile(),
        schemeType(Sdc::SCHEME_CATMARK),
        maxPatchDepth(3),
        evalDepth(-1),
        irregularBasis(Far::PatchTreeFactory::Options::GREGORY),
        uniformRes(5),
        noTessFlag(false),
        noOutputFlag(false) {

        for (int i = 1; i < argc; ++i) {
            if (strstr(argv[i], ".obj")) {
                if (inputObjFile.empty()) {
                    inputObjFile = std::string(argv[i]);
                } else {
                    fprintf(stderr, "Warning: .obj file '%s' ignored\n", argv[i]);
                }
            } else if (!strcmp(argv[i], "-bilinear")) {
                schemeType = Sdc::SCHEME_BILINEAR;
            } else if (!strcmp(argv[i], "-catmark")) {
                schemeType = Sdc::SCHEME_CATMARK;
            } else if (!strcmp(argv[i], "-loop")) {
                schemeType = Sdc::SCHEME_LOOP;
            } else if (!strcmp(argv[i], "-linear")) {
                irregularBasis = Far::PatchTreeFactory::Options::LINEAR;
            } else if (!strcmp(argv[i], "-regular")) {
                irregularBasis = Far::PatchTreeFactory::Options::REGULAR;
            } else if (!strcmp(argv[i], "-gregory")) {
                irregularBasis = Far::PatchTreeFactory::Options::GREGORY;
            } else if (!strcmp(argv[i], "-depth")) {
                if (++i < argc) maxPatchDepth = atoi(argv[i]);
            } else if (!strcmp(argv[i], "-eval")) {
                if (++i < argc) evalDepth = atoi(argv[i]);
            } else if (!strcmp(argv[i], "-res")) {
                if (++i < argc) uniformRes = atoi(argv[i]);
            } else if (!strcmp(argv[i], "-notess")) {
                noTessFlag = true;
            } else if (!strcmp(argv[i], "-nooutput")) {
                noOutputFlag = true;
            } else {
                fprintf(stderr, "Warning: Argument '%s' ignored\n", argv[i]);
            }
        }
    }

private:
    Args() { }
};


//
//  Load geometry and iterate through all base faces of the mesh to tessellate:
//
int
run(Args const & args) {

    //
    //  Load the topology and positions for the given .obj file:
    //
    Coord3Vector basePositions;

    Far::TopologyRefiner * baseRefiner = createTopologyRefinerFromObj(
            args.inputObjFile, args.schemeType, basePositions);

    assert(baseRefiner);
    Far::TopologyRefiner & baseMesh = *baseRefiner;

    //
    //  Construct a tessellation pattern to use for all regular faces, along
    //  with vectors that store tessellation topology and evaulated data:
    //
    int regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(baseMesh.GetSchemeType());

    TessPattern tessPattern;
    if (regFaceSize == 4) {
        tessPattern.BuildUniformQuads(args.uniformRes);
    } else {
        tessPattern.BuildUniformTris(args.uniformRes);
    }

    TessPattern::ParamVector const & tessParams = tessPattern.GetParamVector();
    TessPattern::FaceVector const &  tessFaces  = tessPattern.GetFaceVector();

    Coord3Vector tessPos(tessPattern.GetNumVertices());
    Coord3Vector tessDu(tessPattern.GetNumVertices());
    Coord3Vector tessDv(tessPattern.GetNumVertices());

    //
    //  Construct a Far::PatchTreeCache (stub for now) to store shared topology
    //  for the mesh and initialize Factory::Options to specify its usage:
    //
    Far::PatchTreeCache patchTreeCache;

    Far::BasePatchFactory::Options basePatchOptions;
    basePatchOptions.patchTreeOptions.maxPatchDepth = args.maxPatchDepth;
    basePatchOptions.patchTreeOptions.irregularBasis = args.irregularBasis;
    basePatchOptions.patchTreeCache = &patchTreeCache;
    basePatchOptions.updatePatchTreeCache = true;

    //
    //  Construct a Far::BasePatch for each base face -- in this case, within a simple
    //  wrapper class that manages the position data for the control vertices:
    //
    int objVertCount = 0;

    int numBaseFaces = baseMesh.GetNumFacesTotal();
    for (int baseFace = 0; baseFace < numBaseFaces; ++baseFace) {

        //  Irregular faces not yet unsupported, so skip them:
        if (baseMesh.GetLevel(0).GetFaceVertices(baseFace).size() != regFaceSize) {
            fprintf(stderr, "Warning: Skipping irregular face %d\n", baseFace);
            continue;
        }

        //
        //  Construct a MeshFaceEvaluator for this face and evaluate positions
        //  and normals for the defined tessellation pattern:
        //
        MeshFaceEvaluator faceEval(baseMesh, baseFace, basePositions, basePatchOptions);

        if (args.noTessFlag) continue;

        int numVerts = tessPattern.GetNumVertices();
        for (int i = 0; i < numVerts; ++i) {
            float const * uv = tessParams[i].uv;
            faceEval.Evaluate(uv[0], uv[1], tessPos[i], tessDu[i], tessDv[i],
                              args.evalDepth);
        }

        //
        //  Print tessellated vertices and faces in Obj format to standard output:
        //
        if (args.noOutputFlag) continue;

        printf("g baseFace_%d\n", baseFace);

        for (int i = 0; i < numVerts; ++i) {
            Coord3 const & P = tessPos[i];
            printf("v %f %f %f\n", P.p[0], P.p[1], P.p[2]);
        }
        for (int i = 0; i < numVerts; ++i) {
            Coord3 N = Coord3::ComputeNormal(tessDu[i], tessDv[i]);
            printf("vn %f %f %f\n", N.p[0], N.p[1], N.p[2]);
        }
        int vBase = objVertCount + 1;

        int numFaces = tessPattern.GetNumFaces();
        for (int i = 0; i < numFaces; ++i) {
            int const * v = tessFaces[i].v;
            printf("f ");
            for (int j = 0; j < 4; ++j) {
                if (v[j] >= 0) {
                    printf(" %d//%d", vBase + v[j], vBase + v[j]);
                }
            }
            printf("\n");
        }
        objVertCount += numVerts;
    }
    delete baseRefiner;

    return EXIT_SUCCESS;
}

//
//  Load command line arguments and guarantee minimal requirements before executing:
//
int
main(int argc, char **argv) {

    Args args(argc, argv);

    if (args.inputObjFile.empty()) {
        fprintf(stderr, "Error: Expecting .obj file as argument\n");
        return EXIT_FAILURE;
    }

    return run(args);
}
