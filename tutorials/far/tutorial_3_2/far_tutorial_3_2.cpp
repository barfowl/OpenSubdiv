//
//   Copyright 2019 DreamWorks Animation LLC
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


//------------------------------------------------------------------------------
// Tutorial description:
//
// This tutorial expands on 3.1 with a more practical example of a custom
// factory for Far::TopologyRefiner that deals with a complete edge list.
// Since a Far::TopolgyRefiner contains a complete topological description
// of each level refined, we use Far::TopologyLevel as an example mesh type
// and so define Far::TopologyRefinerFactory<Far::TopologyLevel> from which
// a new TopologyRefiner is created.
//
// Construction is verified by ensuring that, given an intial TopologyRefiner
// subdivided twice, a second TopologyRefiner constructed from level 1 of the
// original and then subdivided produces the same result.  This will not be
// true if the edge list is discarded, recreated, or otherwise not preserved.
//

#include "customRefinerFactory.h"

#include "../../../regression/common/far_utils.h"

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/topologyRefinerFactory.h>
#include <opensubdiv/far/primvarRefiner.h>

#include <string>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace OpenSubdiv;

using Far::Index;
using Far::IndexArray;
using Far::ConstIndexArray;


//
//  Vertex container implementation:
//
struct Vertex {

    //  Minimal required interface:
    Vertex() { }

    Vertex(Vertex const & src) {
        _position[0] = src._position[0];
        _position[1] = src._position[1];
        _position[2] = src._position[2];
    }

    void Clear() {
        _position[0] = _position[1] = _position[2] = 0.0f;
    }

    void AddWithWeight(Vertex const & src, float weight) {
        _position[0]+=weight*src._position[0];
        _position[1]+=weight*src._position[1];
        _position[2]+=weight*src._position[2];
    }

    //  Additional methods:
    void SetPosition(float x, float y, float z) {
        _position[0]=x;
        _position[1]=y;
        _position[2]=z;
    }

    void SetPosition(float const xyz[3]) {
        SetPosition(xyz[0], xyz[1], xyz[2]);
    }

    const float * GetPosition() const {
        return _position;
    }

    bool IsEqual(Vertex const & other, float epsilon = 1e-5) {
        return (std::abs(_position[0] - other._position[0]) < epsilon) &&
               (std::abs(_position[1] - other._position[1]) < epsilon) &&
               (std::abs(_position[2] - other._position[2]) < epsilon);
    }

private:
    float _position[3];
};

typedef std::vector<Vertex> VertexVector;


//
//  Utilities to define and load topology and primvar data:
//
namespace {
    //
    //  Create a TopologyRefiner from default/static geometry:
    //
    Far::TopologyRefiner *
    createTopologyRefinerDefault(VertexVector & vertPositions) {

        //  The default/static shape is a cube:
        static int const   _cubeVertCount        = 8;
        static float const _cubeVertValues[8][3] = { { -0.5f, -0.5f, -0.5f },
                                                     { -0.5f,  0.5f, -0.5f },
                                                     { -0.5f,  0.5f,  0.5f },
                                                     { -0.5f, -0.5f,  0.5f },
                                                     {  0.5f, -0.5f, -0.5f },
                                                     {  0.5f,  0.5f, -0.5f },
                                                     {  0.5f,  0.5f,  0.5f },
                                                     {  0.5f, -0.5f,  0.5f } };

        static int const _cubeFaceCount       = 6;
        static int const _cubeFaceSizes[6]    = { 4, 4, 4, 4, 4, 4 };
        static int const _cubeFaceVerts[6][4] = { { 0, 3, 2, 1 },
                                                  { 4, 5, 6, 7 },
                                                  { 0, 4, 7, 3 },
                                                  { 1, 2, 6, 5 },
                                                  { 0, 1, 5, 4 },
                                                  { 3, 7, 6, 2 } };

        //  Assign a TopologyDescriptor with the above static topology:
        typedef Far::TopologyDescriptor Descriptor;

        Descriptor desc;
        desc.numVertices        =  _cubeVertCount;
        desc.numFaces           =  _cubeFaceCount;
        desc.numVertsPerFace    = &_cubeFaceSizes[0];
        desc.vertIndicesPerFace = &_cubeFaceVerts[0][0];

        //  Instantiate a TopologyRefiner from the TopologyDescriptor:
        Sdc::SchemeType sdcType = OpenSubdiv::Sdc::SCHEME_CATMARK;
        Sdc::Options    sdcOptions;
        sdcOptions.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

        Far::TopologyRefiner * refiner =
            Far::TopologyRefinerFactory<Descriptor>::Create(desc,
                Far::TopologyRefinerFactory<Descriptor>::Options(sdcType, sdcOptions));
        assert(refiner);

        //  Load vertex positions:
        vertPositions.resize(_cubeVertCount);
        for (int i = 0; i < _cubeVertCount; ++i) {
            vertPositions[i].SetPosition(_cubeVertValues[i]);
        }
        return refiner;
    }

    //
    //  Create a TopologyRefiner from a specified Obj file:
    //
    Far::TopologyRefiner *
    createTopologyRefinerFromObj(std::string const & objFileName,
                                 Sdc::SchemeType schemeType,
                                 VertexVector & vertPositions) {

        char const *  filename = objFileName.c_str();
        Shape const * shape = 0;

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

        //  Instantiate a TopologyRefiner from the Shape:
        Sdc::SchemeType sdcType    = GetSdcType(*shape);
        Sdc::Options    sdcOptions = GetSdcOptions(*shape);

        Far::TopologyRefiner * refiner = Far::TopologyRefinerFactory<Shape>::Create(*shape,
            Far::TopologyRefinerFactory<Shape>::Options(sdcType, sdcOptions));
        if (refiner == 0) {
            fprintf(stderr,
                "Error:  Unable to construct TopologyRefiner from .obj file '%s'\n", filename);
            return 0;
        }

        //  Load vertex positions:
        int numVertices = refiner->GetNumVerticesTotal();
        vertPositions.resize(numVertices);
        for (int i = 0; i < numVertices; ++i) {
            vertPositions[i].SetPosition(&shape->verts[i * 3]);
        }
        delete shape;
        return refiner;
    }
} // end namespace


//
//  Command line arguments for run-time options:
//
class Args {
public:
    std::string     inputObjFile;
    Sdc::SchemeType schemeType;

public:
    Args(int argc, char ** argv) :
        inputObjFile(),
        schemeType(Sdc::SCHEME_CATMARK) {

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
            } else {
                fprintf(stderr, "Warning: Argument '%s' ignored\n", argv[i]);
            }
        }
    }

private:
    Args() { }
};


//
//  Load command line arguments and geometry, apply uniform refinment,
//  create a new TopologyRefiner from one of its levels then refine that
//  to the same level and compare:
//
int
main(int argc, char **argv) {

    Args args(argc, argv);

    //
    //  Construct topology from default or specified shape:
    //
    std::vector<Vertex> basePositions;

    Far::TopologyRefiner * origRefinerPtr = args.inputObjFile.empty() ?
            createTopologyRefinerDefault(basePositions) :
            createTopologyRefinerFromObj(args.inputObjFile, args.schemeType,
                                         basePositions);
    assert(origRefinerPtr);
    Far::TopologyRefiner & origRefiner = *origRefinerPtr;

    //
    //  Refine the original shape, then construct another refiner from an
    //  intermediate level and refine it to the same level as the original:
    //
    int depthTotal  = 4;
    int depthCloned = 2;
    int depthDelta  = depthTotal - depthCloned;

    origRefiner.RefineUniform(Far::TopologyRefiner::UniformOptions(depthTotal));

    Far::TopologyLevel const & origLevelCloned = origRefiner.GetLevel(depthCloned);

    //  Enable the "validateFullTopology" option while testing a new factory:
    CustomRefinerFactory::Options newRefinerOptions;
    newRefinerOptions.validateFullTopology = true;
    newRefinerOptions.schemeType           = origRefiner.GetSchemeType();
    newRefinerOptions.schemeOptions        = origRefiner.GetSchemeOptions();

    Far::TopologyRefiner * newRefinerPtr =
        CustomRefinerFactory::Create(origLevelCloned, newRefinerOptions);
    assert(newRefinerPtr);
    Far::TopologyRefiner & newRefiner = *newRefinerPtr;

    newRefiner.RefineUniform(Far::TopologyRefiner::UniformOptions(depthDelta));

    //
    //  Compute refined positions for both original and new refined topology
    //  (we allocate them the same size and copy the original results into
    //  the new up to the level that was cloned):
    //
    std::vector<Vertex> origPositions = basePositions;
    origPositions.resize(origRefiner.GetNumVerticesTotal());

    std::vector<Vertex> newPositions = origPositions;

    Vertex * origSrc = &origPositions[0];
    Vertex * newSrc  = &newPositions[0];

    for (int depth = 1; depth <= depthTotal; ++depth) {
        int nSrcVerts = origRefiner.GetLevel(depth-1).GetNumVertices();

        Vertex * origDst = origSrc + nSrcVerts;
        Vertex * newDst  = newSrc  + nSrcVerts;

        Far::PrimvarRefiner(origRefiner).Interpolate(depth, origSrc, origDst);

        if (depth > depthCloned) {
            Far::PrimvarRefiner(newRefiner).Interpolate((depth - depthCloned),
                    newSrc, newDst);
        } else {
            std::memcpy(newDst, origDst, sizeof(Vertex) * 
                    origRefiner.GetLevel(depth).GetNumVertices());
        }

        origSrc = origDst;
        newSrc  = newDst;
    }

    //
    //  Compare refined positions between the last levels of the two refiners:
    //
    Far::TopologyLevel const & origLevelLast = origRefiner.GetLevel(depthTotal);
    Far::TopologyLevel const & newLevelLast = newRefiner.GetLevel(depthDelta);

    assert(newLevelLast.GetNumVertices() == origLevelLast.GetNumVertices());

    int nPositionsLast       = origLevelLast.GetNumVertices();
    int nPositionsBeforeLast = origRefiner.GetNumVerticesTotal() - nPositionsLast;

    origSrc = &origPositions[nPositionsBeforeLast];
    newSrc  = &newPositions[nPositionsBeforeLast];

    for (int i = 0; i < nPositionsLast; ++i) {
        assert(newSrc[i].IsEqual(origSrc[i]));
    }

    delete origRefinerPtr;
    delete newRefinerPtr;
}
