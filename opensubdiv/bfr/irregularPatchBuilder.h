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

#ifndef OPENSUBDIV3_IRREGULAR_PATCH_BUILDER_H
#define OPENSUBDIV3_IRREGULAR_PATCH_BUILDER_H

#include "../version.h"

#include "../bfr/faceSurface.h"
#include "../vtr/stackBuffer.h"

#include <cstdint>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class PatchTree;
}

namespace Bfr {

//
//  IrregularPatchBuilder takes a FaceSurface (that has been flagged as not
//  regular) and builds a representation for the limit surface it defines.
//
//  It is intended to hide the construction details and final representation
//  of the limit surface from its clients, i.e. the SurfaceFactory.  If the
//  preferred representation changes, or more than one is made available, it
//  should have minimal impact on its clients (ideally none).
//
//  In addition to building and providing a representation of an irregular
//  surface, it also deals with the hashing and caching of the instances
//  that it creates.  It computes the hashing keys from the topology and
//  coordinates with a given cache to find or add new instances.
//
//  WIP - the nature of the approximating options needs more work...
//      - we need some way of specifying the options of Far::PatchTree in
//        a way that's more in line with the Factory's public interface
//
class IrregularPatchBuilder {
public:
    //  WIP - see note above
    struct Options {
        Options() : sharpLevel(6), smoothLevel(2),
                    doublePrecision(false), stencilTables(false) { }

        int  sharpLevel;
        int  smoothLevel;
        bool doublePrecision;
        bool stencilTables;
    };

public:
    IrregularPatchBuilder(FaceSurface const & surfaceDescription,
                          Options             options = Options());
    ~IrregularPatchBuilder() { }

    //  Debugging:
    void print() const;

public:
    //  Methods to query the number and indices of control vertices:
    int GetNumControlVertices() const { return _numControlVerts; }

    int GatherControlVertexIndices(Index cvIndices[]) const;

public:
    //  Methods to build irregular patches:
    typedef Far::PatchTree IrregPatchType;

    IrregPatchType const * Build();

public:
    //  Methods to help encode topology for caching:
    typedef std::uint64_t KeyIntType;

    bool GetPackedTopologyKey(KeyIntType * keyValue) const;
    bool GetHashedTopologyKey(KeyIntType * keyValue) const;

private:
    //  Private methods to assemble the topology of the control hull:
    //  WIP - revisit the need for these separate methods (and repeated
    //        iteration) now that we can put results in member buffers
    void initializeControlHullInventory();

    int getNumControlVertices() const { return _numControlVerts; }
    int getNumControlVertices(int corner) const {
        return _cornerCVertStart[corner+1] - _cornerCVertStart[corner];
    }
    int getNextControlVertex(int corner) const {
        return _cornerCVertStart[corner];
    }

    int getNumControlFaces() const { return _numControlFaces; }
    int getNumControlFaces(int corner) const {
        return _cornerCFaceCount[corner];
    }

    int gatherControlFaceSizes(int faceSizes[]) const;
    int gatherControlFaceVertices(int faceVertices[]) const;

    int countSharpControlVertices() const;
    int gatherControlVertexSharpness(int   vertIndices[],
                                     float vertSharpness[]) const;

    int countSharpControlEdges() const;
    int gatherControlEdgeSharpness(int   edgeVertPairs[],
                                   float edgeSharpness[]) const;

    void getControlFaceVertices(int  faceVerts[], int numFaceVerts,
                                int  corner,      int nextPerimeterVert) const;
    void getControlFaceVertices(int  faceVerts[], int numFaceVerts,
                                int  corner,      int nextPerimeterVert,
                                bool lastFace,    int val2IntOverlap = 0) const;

private:
    //  Private members:
    FaceSurface const & _surface;
    Options             _options;

    //  Members defining the control hull of the surface -- some storing
    //  contributions to the control hull for each corner:
    int _numControlVerts;
    int _numControlFaces;

    Vtr::internal::StackBuffer<int,9,true> _cornerCVertStart;
    Vtr::internal::StackBuffer<int,8,true> _cornerCFaceCount;

    bool _hasVal2IntCorners;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_IRREGULAR_PATCH_BUILDER_H */
