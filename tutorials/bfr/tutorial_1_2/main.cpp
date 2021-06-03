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
//      This tutorial builds on the previous tutorial that makes use of the
//      Bfr::LimitSurface and Bfr::Tessellation classes for evaluating and
//      tessellating the limit surface of faces of a mesh.  This tutorial
//      adds support for evaluating and tessellating face-varying UVs.  If
//      UVs are present in the mesh, they will be evaluated, tessellated
//      and written to the Obj file.
//

#include "../../../regression/common/far_utils.h"
#include "../../../examples/common/stopwatch.h"

#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/patchTable.h>
#include <opensubdiv/far/patchMap.h>
#include <opensubdiv/far/ptexIndices.h>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/stencilTable.h>
#include <opensubdiv/bfr/refinerLimitSurfaceFactory.h>
#include <opensubdiv/bfr/limitSurface.h>
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
    int             tessUniform;
    bool            noUVFlag;
    bool            triTessFlag;

public:
    Args(int argc, char ** argv) :
        inputObjFile(),
        outputObjFile(),
        schemeType(Sdc::SCHEME_CATMARK),
        tessUniform(5),
        noUVFlag(false),
        triTessFlag(false) {

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
                if (++i < argc) tessUniform = atoi(argv[i]);
            } else if (!strcmp(argv[i], "-nouvs")) {
                noUVFlag = true;
            } else if (!strcmp(argv[i], "-tris")) {
                triTessFlag = true;
            } else {
                fprintf(stderr, "Warning: Argument '%s' ignored\n", argv[i]);
            }
        }
        if (triTessFlag) {
            fprintf(stderr, "Warning: Argument -tris not yet supported.\n");
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
    void writeVertexUVs(std::vector<Vec3f> const & uv);

    void writeFaces(std::vector<Bfr::Facet> const & faces,
                    bool writeNormalIndices = false,
                    bool writeUVIndices = false);

    void writeGroupName(char const * prefix, int index);

private:
    std::string _filename;
    FILE *      _fptr;

    int _numVertices;
    int _numNormals;
    int _numUVs;
    int _numFaces;
};

ObjWriter::ObjWriter(std::string const &filename) :
        _fptr(0), _numVertices(0), _numNormals(0), _numUVs(0), _numFaces(0) {

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
ObjWriter::writeVertexUVs(std::vector<Vec3f> const & uv) {

    int numNewUVs = (int)uv.size();

    for (int i = 0; i < numNewUVs; ++i) {
        fprintf(_fptr, "vt %f %f\n", uv[i][0], uv[i][1]);
    }
    _numUVs += numNewUVs;
}

void
ObjWriter::writeFaces(std::vector<Bfr::Facet> const & faces,
                      bool includeNormalIndices, bool includeUVIndices) {

    int numNewFaces = (int)faces.size();

    for (int i = 0; i < numNewFaces; ++i) {
        int const * v = faces[i].v;
        fprintf(_fptr, "f ");
        for (int j = 0; j < 4; ++j) {
            if (v[j] >= 0) {
                int vIndex = v[j];

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

//
//  The main tessellation function:  given a mesh, vertex positions and
//  UVs, tessellate each face -- writing results in Obj format.
//
void
tessellateToObj(Far::TopologyRefiner const & baseMesh,
                std::vector<Vec3f> const &   baseMeshVertexXYZs,
                std::vector<Vec3f> const &   baseMeshFVarUVs,
                Args const &                 args) {

    //  Initialize an Obj writer locally for this mesh:
    ObjWriter objWriter(args.outputObjFile);

    //
    //  Initialize specified evaluation options and declare buffers
    //  required by use of instances of Bfr::LimitSurface during
    //  evaluation (declared here to reuse memory for each face):
    //
    Bfr::RefinerLimitSurfaceFactory::Options limitFactoryOptions;
    limitFactoryOptions.CreateFVarEvaluators(!args.noUVFlag &&
                                             (baseMeshFVarUVs.size() > 0));

    std::vector<Vec3f> limitSurfaceXYZPoints;
    std::vector<Vec3f> limitSurfaceUVPoints;

    //
    //  Initialize specified tessellation options and declare buffers
    //  required for evaluation of Bfr::Tessellation patterns (declared
    //  here to reuse memory for each face):
    //
    Bfr::Tessellation::Options tessOptions;
    tessOptions.SetTriangulateQuadFacets(args.triTessFlag);

    std::vector<Bfr::Coord> tessCoords;
    std::vector<Bfr::Facet> tessFacets;

    std::vector<Vec3f> tessXYZ, tessDu, tessDv;
    std::vector<Vec3f> tessUV;

    //
    //  Initialize the Bfr::LimitSurfaceFactory for the given base mesh
    //  (very low cost in terms of time and space) and tessellate each
    //  face independently (i.e. no shared vertices):
    //
    //  Note that the LimitSurfaceFactory is not thread-safe by default
    //  due to use of an internal cache.  Creating a separate instance
    //  of the LimitSurfaceFactory for each thread is one way to safely
    //  parallelize this loop.  Another (preferred) is to assign a
    //  thread-safe cache to the single instance.
    //
    Bfr::RefinerLimitSurfaceFactory limitFactory(baseMesh, limitFactoryOptions);

    int numFaces = limitFactory.GetNumFaces();
    for (int faceIndex = 0; faceIndex < numFaces; ++faceIndex) {
        //
        //  Create/populate the LimitSurface for this face (if present, i.e.
        //  skipping holes and designated boundary faces) and declare the
        //  simple uniform Tessellation using its Parameterization:
        //
        //  (The LimitSurface can also first be used to evaluate points
        //  that may then determine non-uniform Tessellation parameters per
        //  edge, e.g. evaluating positions and normals at corners of the
        //  face to assess curvature, etc.)
        //
        if (!limitFactory.FaceHasLimitSurface(faceIndex)) continue;

        Bfr::LimitSurface limitSurface;
        limitFactory.Populate(limitSurface, faceIndex);

        Bfr::Tessellation tessPattern(limitSurface.GetParameterization(),
                                      args.tessUniform,
                                      tessOptions);

        //
        //  Identify coordinates of the sample points of the Tessellation
        //  pattern (more specific inspection methods are available, but
        //  gathering the full set of data is simplest for this example):
        //
        int numTessCoords = tessPattern.GetNumCoords();

        tessCoords.resize(numTessCoords);

        tessPattern.GetCoords(&tessCoords[0]);

        //
        //  For both position and UVs (when present), assemble the local
        //  buffer of points for LimitSurface evaluation (resizing as
        //  needed) and evaluate the sample points of the Tessellation:
        //
        Bfr::Evaluator const * vtxEval = limitSurface.GetVertexEvaluator();
        {
            limitSurfaceXYZPoints.resize(vtxEval->GetNumPatchPoints());

            vtxEval->PreparePatchPointValues(&baseMeshVertexXYZs[0],
                                             &limitSurfaceXYZPoints[0]);

            tessXYZ.resize(numTessCoords);
            tessDu.resize(numTessCoords);
            tessDv.resize(numTessCoords);

            vtxEval->Evaluate(numTessCoords, &tessCoords[0],
                              &limitSurfaceXYZPoints[0],
                              &tessXYZ[0], &tessDu[0], &tessDv[0]);
        }

        Bfr::Evaluator const * fvarEval = limitSurface.GetFaceVaryingEvaluator();
        if (fvarEval) {
            limitSurfaceUVPoints.resize(fvarEval->GetNumPatchPoints());

            fvarEval->PreparePatchPointValues(&baseMeshFVarUVs[0],
                                              &limitSurfaceUVPoints[0]);

            tessUV.resize(numTessCoords);

            fvarEval->Evaluate(numTessCoords, &tessCoords[0],
                               &limitSurfaceUVPoints[0],
                               &tessUV[0]);
        }

        //
        //  Identify facets connecting sample points of the Tessellation:
        //
        //  Note that all Coord indices referenced by the Facets are local
        //  to the face (i.e. they range from [0..N-1], where N is the
        //  number of Coords in the pattern) and so need to be offset when
        //  writing to Obj format.  For more advanced use, the Coords
        //  associated with the boundary and interior of the pattern are
        //  distinguishable so that those on the boundary can be easily
        //  remapped to refer to shared edge or corner points, while those
        //  in the interior can be separately offset or similarly remapped.
        //
        int numTessFaces = tessPattern.GetNumFacets();

        tessFacets.resize(numTessFaces);

        tessPattern.GetFacets(&tessFacets[0]);

        tessPattern.TransformFacetIndices(&tessFacets[0],
                                          1 + objWriter.GetNumVertices());

        //
        //  Write the positions, normals and UVs of tessellated points to
        //  the Obj, along with the faces that connect them:
        //
        bool writeNormals = (tessDu.size() > 0);
        bool writeUVs     = (tessUV.size() > 0);

        objWriter.writeGroupName("baseFace_", faceIndex);

        objWriter.writeVertexPositions(tessXYZ);
        if (writeNormals) {
            objWriter.writeVertexNormals(tessDu, tessDv);
        }
        if (writeUVs) {
            objWriter.writeVertexUVs(tessUV);
        }

        objWriter.writeFaces(tessFacets, writeNormals, writeUVs);
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
    tessellateToObj(*baseRefiner, meshVtxPositions, meshFVarUVs, args);

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
