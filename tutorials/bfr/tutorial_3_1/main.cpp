//
//   Copyright 2021
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
//      This tutorial builds on others using the SurfaceFactory, Surface
//      and Tessellation classes by using more of the functionality of the
//      Tessellation class to construct a tessellation of the mesh that is
//      topologically watertight, i.e. resulting points evaluated along
//      shared edges or vertices are shared and not duplicated.
//
//      Since Tessellation provides points around its boundary first, the
//      evaluated points for shared vertices and edges are identified when
//      constructed and reused when shared later. The boundary of the
//      tessellation of a face is therefore a collection of shared points
//      and methods of Tessellation help to remap the faces generated to
//      the shared set of points.
//

#include "../../../regression/common/far_utils.h"

#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/bfr/refinerSurfaceFactory.h>
#include <opensubdiv/bfr/surface.h>
#include <opensubdiv/bfr/tessellation.h>

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
    struct Vec3f {
        Vec3f() { }
        Vec3f(float x, float y, float z) { p[0] = x, p[1] = y, p[2] = z; }

        //  Clear() and AddWithWeight() required for interpolation:
        void Clear( void * =0 ) { p[0] = p[1] = p[2] = 0.0f; }

        void AddWithWeight(Vec3f const & src, float weight) {
            p[0] += weight * src.p[0];
            p[1] += weight * src.p[1];
            p[2] += weight * src.p[2];
        }

        //  Element access via []:
        float const & operator[](int i) const { return p[i]; }
        float       & operator[](int i)       { return p[i]; }

        //  Additional useful mathematical operations:
        Vec3f operator-(Vec3f const & x) const {
            return Vec3f(p[0] - x.p[0], p[1] - x.p[1], p[2] - x.p[2]);
        }
        Vec3f operator+(Vec3f const & x) const {
            return Vec3f(p[0] + x.p[0], p[1] + x.p[1], p[2] + x.p[2]);
        }
        Vec3f operator*(float s) const {
            return Vec3f(p[0] * s, p[1] * s, p[2] * s);
        }
        Vec3f Cross(Vec3f const & x) const {
            return Vec3f(p[1]*x.p[2] - p[2]*x.p[1],
                         p[2]*x.p[0] - p[0]*x.p[2],
                         p[0]*x.p[1] - p[1]*x.p[0]);
        }
        float Dot(Vec3f const & x) const {
            return p[0]*x.p[0] + p[1]*x.p[1] + p[2]*x.p[2];
        }
        float Length() const {
            return std::sqrt(this->Dot(*this));
        }

        //  Static method to compute normal vector:
        static
        Vec3f ComputeNormal(Vec3f const & Du, Vec3f const & Dv, float eps = 0) {
            Vec3f N = Du.Cross(Dv);
            float lenSqrd = N.Dot(N);
            if (lenSqrd <= eps) return Vec3f(0.0f, 0.0f, 0.0f);
            return N * (1.0f / std::sqrt(lenSqrd));
        }

        //  Member variables (XYZ coordinates):
        float p[3];
    };

    //
    //  Create a TopologyRefiner from a specified Obj file:
    //
    Far::TopologyRefiner *
    createTopologyRefinerFromObj(std::string const & objFileName,
                                 Sdc::SchemeType schemeType,
                                 std::vector<Vec3f> & posVector,
                                 std::vector<Vec3f> & uvVector) {

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
                fprintf(stderr,
                    "Error:  Cannot create Shape from Obj file '%s'\n", filename);
                return 0;
            }
        } else {
            fprintf(stderr, "Error:  Cannot open Obj file '%s'\n", filename);
            return 0;
        }

        Sdc::SchemeType sdcType    = GetSdcType(*shape);
        Sdc::Options    sdcOptions = GetSdcOptions(*shape);

        Far::TopologyRefiner * refiner = Far::TopologyRefinerFactory<Shape>::Create(
            *shape, Far::TopologyRefinerFactory<Shape>::Options(sdcType, sdcOptions));
        if (refiner == 0) {
            fprintf(stderr,
                "Error:  Unable to construct TopologyRefiner from Obj file '%s'\n",
                filename);
            return 0;
        }

        int numVertices = refiner->GetNumVerticesTotal();
        posVector.resize(numVertices);
        std::memcpy(&posVector[0], &shape->verts[0], numVertices * 3 * sizeof(float));

        uvVector.resize(0);
        if (refiner->GetNumFVarChannels()) {
            int numUVs = refiner->GetNumFVarValuesTotal(0);
            uvVector.resize(numUVs);
            for (int i = 0; i < numUVs; ++i) {
                uvVector[i] = Vec3f(shape->uvs[i*2], shape->uvs[i*2+1], 0.0f);
            }
        }

        delete shape;
        return refiner;
    }
} // end namespace


//
//  Command line arguments parsed to provide run-time options:
//
class Args {
public:
    std::string     inputObjFile;
    std::string     outputObjFile;
    Sdc::SchemeType schemeType;
    int             tessUniformRate;
    bool            tessQuadsFlag;

public:
    Args(int argc, char ** argv) :
        inputObjFile(),
        outputObjFile(),
        schemeType(Sdc::SCHEME_CATMARK),
        tessUniformRate(5),
        tessQuadsFlag(false) {

        for (int i = 1; i < argc; ++i) {
            if (strstr(argv[i], ".obj")) {
                if (inputObjFile.empty()) {
                    inputObjFile = std::string(argv[i]);
                } else {
                    fprintf(stderr, "Warning: Obj file '%s' ignored\n", argv[i]);
                }
            } else if (!strcmp(argv[i], "-o")) {
                if (++i < argc) outputObjFile = std::string(argv[i]);
            } else if (!strcmp(argv[i], "-bilinear")) {
                schemeType = Sdc::SCHEME_BILINEAR;
            } else if (!strcmp(argv[i], "-catmark")) {
                schemeType = Sdc::SCHEME_CATMARK;
            } else if (!strcmp(argv[i], "-loop")) {
                schemeType = Sdc::SCHEME_LOOP;
            } else if (!strcmp(argv[i], "-res")) {
                if (++i < argc) tessUniformRate = atoi(argv[i]);
            } else if (!strcmp(argv[i], "-quads")) {
                tessQuadsFlag = true;
            } else {
                fprintf(stderr, "Warning: Argument '%s' ignored\n", argv[i]);
            }
        }
    }

private:
    Args() { }
};

class ObjWriter {
public:
    ObjWriter(std::string const &filename = 0);
    ~ObjWriter();

    int GetNumVertices() const { return _numVertices; }
    int GetNumFaces()    const { return _numFaces; }

    void writeVertexPositions(std::vector<Vec3f> const & p);
    void writeVertexNormals(std::vector<Vec3f> const & du,
                            std::vector<Vec3f> const & dv);

    void writeFaces(std::vector<int> const & faceVertices, int faceSize,
                    bool writeNormalIndices = false,
                    bool writeUVIndices = false);

    void writeGroupName(char const * prefix, int index);

private:
    std::string _filename;
    FILE *      _fptr;

    int _numVertices;
    int _numNormals;
    int _numFaces;
};

ObjWriter::ObjWriter(std::string const &filename) :
        _fptr(0), _numVertices(0), _numNormals(0), _numFaces(0) {

    if (filename != std::string()) {
        _fptr = fopen(filename.c_str(), "w");
        if (_fptr == 0) {
            fprintf(stderr, "Error:  ObjWriter cannot open Obj file '%s'\n",
                filename.c_str());
        }
    }
    if (_fptr == 0) _fptr = stdout;
}

ObjWriter::~ObjWriter() {

    if (_fptr != stdout) fclose(_fptr);
}

void
ObjWriter::writeVertexPositions(std::vector<Vec3f> const & positions) {

    int numNewVerts = (int)positions.size();

    for (int i = 0; i < numNewVerts; ++i) {
        Vec3f const & P = positions[i];
        fprintf(_fptr, "v %f %f %f\n", P[0], P[1], P[2]);
    }
    _numVertices += numNewVerts;
}

void
ObjWriter::writeVertexNormals(std::vector<Vec3f> const & du,
                              std::vector<Vec3f> const & dv) {

    assert(du.size() == dv.size());
    int numNewNormals = (int)du.size();

    for (int i = 0; i < numNewNormals; ++i) {
        Vec3f N = Vec3f::ComputeNormal(du[i], dv[i]);
        fprintf(_fptr, "vn %f %f %f\n", N[0], N[1], N[2]);
    }
    _numNormals += numNewNormals;
}

void
ObjWriter::writeFaces(std::vector<int> const & faceVertices, int faceSize,
                      bool includeNormalIndices, bool includeUVIndices) {

    int numNewFaces = (int)faceVertices.size() / faceSize;

    int const * v = &faceVertices[0];
    for (int i = 0; i < numNewFaces; ++i, v += faceSize) {
        fprintf(_fptr, "f ");
        for (int j = 0; j < faceSize; ++j) {
            if (v[j] >= 0) {
                //  Remember Obj indices start with 1:
                int vIndex = 1 + v[j];

                if (includeNormalIndices && includeUVIndices) {
                    fprintf(_fptr, " %d/%d/%d", vIndex, vIndex, vIndex);
                } else if (includeNormalIndices) {
                    fprintf(_fptr, " %d//%d", vIndex, vIndex);
                } else if (includeUVIndices) {
                    fprintf(_fptr, " %d/%d", vIndex, vIndex);
                } else {
                    fprintf(_fptr, " %d", vIndex);
                } 
            }
        }
        fprintf(_fptr, "\n");
    }
    _numFaces += numNewFaces;
}

void
ObjWriter::writeGroupName(char const * prefix, int index) {

    fprintf(_fptr, "g %s%d\n", prefix ? prefix : "", index);
}

//  Local helpers for the main tessellation function that follows:
namespace {
    inline bool
    DoEdgeVertexIndicesIncrease(int edgeInFace, ConstIndexArray & faceVerts) {

        int v0InFace = edgeInFace;
        int v1InFace = (v0InFace == (faceVerts.size()-1)) ? 0 : (v0InFace+1);

        return (faceVerts[v0InFace] < faceVerts[v1InFace]);
    }
} // end namespace

//
//  The main tessellation function:  given a mesh and vertex positions,
//  tessellate each face -- writing results in Obj format.
//
//  This tessellation function differs from earlier tutorials in that it
//  computes and used shared points at vertices and edges of the mesh.
//  These are computed and used as encountered by the faces -- rather than
//  computing all shared vertex and edge points at once (which is more
//  amenable to threading).
//
//  This method has the advantage of only constructing face Surfaces once
//  per face, but requires additional book-keeping, and accesses memory
//  less coherently (making threading more difficult).
//
void
tessellateToObj(Far::TopologyRefiner const & baseMesh,
                std::vector<Vec3f> const &   baseMeshVertexXYZs,
                Args const &                 args) {

    //  Initialize an Obj writer locally for this mesh:
    ObjWriter objWriter(args.outputObjFile);

    //  Use simpler type names locally for the Surface and its factory:
    typedef Bfr::RefinerSurfaceFactory   SurfaceFactory;
    typedef Bfr::Surface<float>          Surface;

    //
    //  Initialize specified evaluation options (none explicit here) and
    //  declare buffers required by use of instances of Surface during
    //  evaluation (declared here to reuse memory for each face):
    //
    SurfaceFactory::Options surfaceOptions;

    std::vector<Vec3f> surfaceXYZPoints;

    //
    //  Initialize tessellation options (use 4 indices per facet to
    //  accomodate quads), declare buffers required for evaluation of
    //  Bfr::Tessellation patterns (declared here to reuse memory for
    //  each face):
    //
    int const FacetSize = 3 + args.tessQuadsFlag;

    Bfr::Tessellation::Options tessOptions;
    tessOptions.SetFacetSize(FacetSize);
    tessOptions.PreserveQuads(args.tessQuadsFlag);

    std::vector<float> tessCoordPairs;
    std::vector<int>   tessFacetIndices;
    std::vector<Vec3f> tessXYZ, tessDu, tessDv;

    //
    //  Initialize the SurfaceFactory for the given base mesh (very
    //  low cost in terms of time and space) and tessellate each
    //  face independently (i.e. no shared vertices):
    //
    //  Note that the SurfaceFactory is not thread-safe by default
    //  due to use of an internal cache.  Creating a separate instance
    //  of the SurfaceFactory for each thread is one way to safely
    //  parallelize this loop.  Another (preferred) is to assign a
    //  thread-safe cache to the single instance.
    //
    SurfaceFactory surfaceFactory(baseMesh, surfaceOptions);

    //
    //  Vectors to identify shared tessellation points at vertices and
    //  edges and their indices around the boundary of a face:
    //
    Far::TopologyLevel const & baseLevel = baseMesh.GetLevel(0);

    std::vector<int> sharedVertexPointIndex(baseLevel.GetNumVertices(), -1);
    std::vector<int> sharedEdgePointIndex(baseLevel.GetNumEdges(), -1);

    std::vector<int> tessBoundaryIndices;

    //
    //  Tessellate each face -- writing all points and facets to the Obj
    //  file in a group designated for this face:
    //
    int numMeshPointsEvaluated = 0;

    int numFaces = surfaceFactory.GetNumFaces();
    for (int faceIndex = 0; faceIndex < numFaces; ++faceIndex) {
        //
        //  Create/populate the Surface for this face (if present, i.e.
        //  skipping holes and designated boundary faces) and declare
        //  the simple uniform Tessellation using its Parameterization:
        //
        //  (The position Surface can also first be used to evaluate points
        //  that may then determine non-uniform Tessellation parameters per
        //  edge, e.g. evaluating positions and normals at corners of the
        //  face to assess curvature, etc.  Note that invalid tessellation
        //  parameters can cause Tessellation construction to fail, so use
        //  an assert to catch programming errors.)
        //
        Surface posSurface;

        if (!surfaceFactory.InitVertexSurface(faceIndex, &posSurface)) {
            continue;
        }

        Bfr::Tessellation tessPattern(posSurface.GetParameterization(),
                                      args.tessUniformRate,
                                      tessOptions);
        assert(tessPattern.IsValid());

        //
        //  Identify coordinates of the sample points of the Tessellation
        //  pattern -- in this case, separating boundary and interior points:
        int numTessCoords = tessPattern.GetNumCoords();

        tessCoordPairs.resize(numTessCoords * 2);

        tessPattern.GetCoords(&tessCoordPairs[0]);

        //
        //  Assemble/resize the local buffer of points for the Surface
        //  evaluation and resize buffers for the evaluated points:
        //
        surfaceXYZPoints.resize(posSurface.GetNumPatchPoints());

        posSurface.PreparePatchPointValues(baseMeshVertexXYZs,
                                           surfaceXYZPoints);

        //  Resize these buffers for all points now, but the actual number
        //  of new points evaulated may be less -- remember to trim later:
        tessXYZ.resize(numTessCoords);
        tessDu.resize(numTessCoords);
        tessDv.resize(numTessCoords);

        //
        //  Evaluate the sample points of the Tessellation:
        //
        //  First we traverse the boundary of the face to determine whether
        //  to evaluate or share points on vertices and edges of the face.
        //  Both pre-existing and new boundary points are identified by
        //  index in an index buffer for later use.  The interior points
        //  are trivially computed after the boundary is dealt with.
        //
        //  Identify the boundary and interior coords and initialize the
        //  buffer for the potentially shared boundary points:
        //
        int numBoundaryCoords = tessPattern.GetNumBoundaryCoords();
        int numInteriorCoords = numTessCoords - numBoundaryCoords;

        float const * tessBoundaryCoords = &tessCoordPairs[0];
        float const * tessInteriorCoords = &tessCoordPairs[numBoundaryCoords*2];

        ConstIndexArray fVerts = baseLevel.GetFaceVertices(faceIndex);
        ConstIndexArray fEdges = baseLevel.GetFaceEdges(faceIndex);

        tessBoundaryIndices.resize(numBoundaryCoords);

        //
        //  Walk around the face, inspecting each vertex and outgoing edge,
        //  and populating the boundary index buffer in the process:
        //
        int boundaryIndex = 0;
        int numFacePointsEvaluated = 0;
        for (int i = 0; i < fVerts.size(); ++i) {
            //  Evaluate and/or retrieve the shared point for the vertex:
            {
                int & vertPointIndex = sharedVertexPointIndex[fVerts[i]];
                if (vertPointIndex < 0) {
                    vertPointIndex = numMeshPointsEvaluated ++;

                    float const * uv = &tessBoundaryCoords[boundaryIndex*2];

                    int index = numFacePointsEvaluated ++;
                    posSurface.Evaluate(uv, &surfaceXYZPoints[0],
                            &tessXYZ[index], &tessDu[index], &tessDv[index]);
                }
                tessBoundaryIndices[boundaryIndex++] = vertPointIndex;
            }

            //  Evaluate and/or retrieve all shared points for the edge:
            int N = args.tessUniformRate - 1;
            if (N) {
                //  Be careful to respect ordering of the edge and its
                //  points when both evaluating and identifying indices:
                bool edgeIsNotReversed = DoEdgeVertexIndicesIncrease(i, fVerts);

                int iOffset = edgeIsNotReversed ? 0 : (N - 1);
                int iDelta  = edgeIsNotReversed ? 1 : -1;

                int & edgePointIndex = sharedEdgePointIndex[fEdges[i]];
                if (edgePointIndex < 0) {
                    edgePointIndex = numMeshPointsEvaluated;

                    float const * uv = &tessBoundaryCoords[boundaryIndex*2];

                    int iNext = numFacePointsEvaluated + iOffset;
                    for (int j = 0; j < N; ++j, iNext += iDelta, uv += 2) {
                        posSurface.Evaluate( uv, &surfaceXYZPoints[0],
                            &tessXYZ[iNext], &tessDu[iNext], &tessDv[iNext]);
                    }
                    numFacePointsEvaluated += N;
                    numMeshPointsEvaluated += N;
                }
                int iNext = edgePointIndex + iOffset;
                for (int j = 0; j < N; ++j, iNext += iDelta) {
                    tessBoundaryIndices[boundaryIndex++] = iNext;
                }
            }
        }

        //
        //  Evaluate any interior points unique to this face -- appending
        //  them to those shared points computed above for the boundary:
        //
        if (numInteriorCoords) {
            float const * uv = tessInteriorCoords;

            int iLast = numFacePointsEvaluated + numInteriorCoords;
            for (int i = numFacePointsEvaluated; i < iLast; ++i, uv += 2) {
                posSurface.Evaluate(uv, &surfaceXYZPoints[0],
                                    &tessXYZ[i], &tessDu[i], &tessDv[i]);
            }
            numFacePointsEvaluated += numInteriorCoords;
            numMeshPointsEvaluated += numInteriorCoords;
        }

        //
        //  Remember to trim/resize the buffers storing evaluation results
        //  for new points to reflect the size actually populated.
        //
        tessXYZ.resize(numFacePointsEvaluated);
        tessDu.resize(numFacePointsEvaluated);
        tessDv.resize(numFacePointsEvaluated);

        //
        //  Write the positions and normals of tessellated points to
        //  the Obj file before dealing with the faces:
        //
        objWriter.writeGroupName("baseFace_", faceIndex);

        objWriter.writeVertexPositions(tessXYZ);
        objWriter.writeVertexNormals(tessDu, tessDv);

        //
        //  Identify facets connecting sample points of the Tessellation:
        //
        //  Note that the coordinate indices used by the facets are local
        //  to the face (i.e. they range from [0..N-1], where N is the
        //  number of coordinates in the pattern) and so need to be offset
        //  when writing to Obj format.
        //
        //  For more advanced use, the coordinates associated with the
        //  boundary and interior of the pattern are distinguishable so
        //  that those on the boundary can be easily remapped to refer to
        //  shared edge or corner points, while those in the interior can
        //  be separately offset or similarly remapped.
        //
        //  So transform the indices of the facets here as needed using
        //  the indices of shared boundary points assembled above and a
        //  suitable offset for the new interior points added:
        //
        int tessInteriorOffset = numMeshPointsEvaluated - numTessCoords;

        int numFacets = tessPattern.GetNumFacets();
        tessFacetIndices.resize(numFacets * FacetSize);
        tessPattern.GetFacets(&tessFacetIndices[0]);

        tessPattern.TransformFacetIndices(&tessFacetIndices[0],
                        &tessBoundaryIndices[0], tessInteriorOffset);

        //  Write faces connecting tessellated points to the Obj file:
        objWriter.writeFaces(tessFacetIndices, FacetSize, true, false);
    }
}

int
run(Args const & args) {

    //
    //  Load the topology, positions and UVs from the given Obj file:
    //
    std::vector<Vec3f> meshVtxPositions;
    std::vector<Vec3f> meshFVarUVs;

    Far::TopologyRefiner * baseRefiner = createTopologyRefinerFromObj(
            args.inputObjFile, args.schemeType, meshVtxPositions, meshFVarUVs);

    if (baseRefiner == 0) {
        return EXIT_FAILURE;
    }

    //
    //  Tessellate to Obj format (directed as specified in Args):
    //
    tessellateToObj(*baseRefiner, meshVtxPositions, args);

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
        fprintf(stderr, "Error: Expecting Obj file (.obj) as argument\n");
        return EXIT_FAILURE;
    }

    return run(args);
}
