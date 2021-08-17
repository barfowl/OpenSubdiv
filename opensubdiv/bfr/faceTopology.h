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

#ifndef OPENSUBDIV3_BFR_FACE_TOPOLOGY_H
#define OPENSUBDIV3_BFR_FACE_TOPOLOGY_H

#include "../version.h"

#include "../bfr/vertexTopology.h"
#include "../bfr/topologyCache.h"

#include "../sdc/types.h"
#include "../sdc/options.h"
#include "../vtr/types.h"
#include "../vtr/stackBuffer.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  The FaceTopology class describes the full topological neighborhood
//  of a base face that includes everything necessary to define the
//  limit surface for that face.
//
//  It is used solely by the base LimitSurfaceFactory -- partially
//  populated by subclasses and then analyzed to assemble the limit
//  surface for the face.
//
//
//  WIP - both the FaceTopology and VertexTopology classes are about
//  to get a facelift...
//
//  FaceTopology encapsulates the full topology around a particular
//  face independent of vertex or face-varying data, but the limit
//  surface is often defined by a subset (delimited by inf-sharp
//  creases, a manifold subset of a non-manifold set, a face-varying
//  subset, etc.)
//
//  The full definition of the limit surface -- both its topology and
//  control point indices -- currently requires the following:
//
//      - an instance of FaceTopology provide all topological information
//      - an array of CornerSubsets describing the topological subset
//      - an array of control point indices:
//          - these are provided relative to the full FaceTopology
//          - from which the relevant subset of points is identified
//
//  These three components are now bundled into a SurfaceDescriptor,
//  which is used to define both vertex and face-varying surfaces. So
//  the Factory now assembles these and passes them on to be assembled
//  into surfaces and assigned.
//
//  SurfaceDescriptor is one addition to a small "ecosystem" of classes
//  in the works...
//
//  VertexTopology:
//      - this is our public-facing class populated by Factory subclasses
//      - it will remain so but stripped down to serve only those needs
//
//  CornerTopology:
//      - will either derive from or contain a VertexTopology to access
//      - will include the existing "face-in-vertex" member for the corner
//      - will contain tags reflecting collective properties of the vertex:
//          - some may move here from VertexTopology, others will be new
//      - will include additional members that correlate to the faces of
//        VertexTopology for additional processing:
//          - an array of ints for "unordered face neighbors" is planned
//            to deal with unordered faces in VertexTopology (which will
//            allow support of non-manifold topology)
//
//  CornerSubset:
//      - will remain unchanged
//      - should remain a very simple struct (no more than a few ints)
//
//  FaceTopology:
//      - will contain an array of CornerTopology (rather than VertexTopology)
//      - also to contain tags reflecting collective properties of the array
//      * this is where unordered and/or non-manifold topology will need to
//        be dealt with but:
//          - requires access to vertex indices to determine connectivity
//          - ideally wants to define a subset when ordering
//
//  SurfaceDescriptor:
//      - will contain a reference to an instance of FaceTopology
//      - will contain an array of CornerSubsets
//      - will contain a reference to an external array of indices
//      - also to contain tags reflecting collective properties of subsets
//
//  All of these should remain light-weight and avoid any memory allocation
//  from the heap (except for high-valence cases).  Simple methods --
//  particularly accessors -- should be inline to additionally keep these
//  efficient in terms of space and time.
//
struct CornerSubset {
    unsigned short _isBoundary;
    unsigned short _isSharp;

    short _numFacesTotal;
    short _numFacesBefore;
    short _numFacesAfter;

    //  These members place the corner relative to a particular collection
    //  of subsets and so may be better off somewhere else...
    short _numOuterVerts;
    short _numOuterFaces;
};

class FaceTopology {
public:
    FaceTopology(Sdc::SchemeType schemeType,
                 Sdc::Options    schemeOptions);
    ~FaceTopology() { }

    void Initialize(int faceSize);
    void Finalize();

    int GetFaceSize() const { return _faceSize; }

    //  WIP - will need some kind of public method to resolve unordered
    //  (non-manifold) vertices using the indices for all vertices.

    //  Debugging...
    void print(Index const faceVertIndices[]) const;

public:
    //  Methods likely to be removed or replaced...

    //  WIP - to be removed once all features supported
    //      - REMEMBER that Loop patches NOT fully supported:
    //          - regular patches also not complete for Loop
    bool IsUnsupported() const {
        if (_hasVal2IntVerts || _hasUnorderedVerts) {
            return true;
        }
        return false;
    }

public:
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;

    int _faceSize;
    int _regFaceSize;

    unsigned int _hasBoundaryVerts  : 1;
    unsigned int _hasInfSharpVerts  : 1;
    unsigned int _hasSemiSharpVerts : 1;
    unsigned int _hasSharpEdges     : 1;
    unsigned int _hasUnSharpBound   : 1;
    unsigned int _hasIncIrregFaces  : 1;
    unsigned int _hasUnorderedVerts : 1;
    unsigned int _hasVal2IntVerts   : 1;

    unsigned int _isInitialized : 1;
    unsigned int _isFinalized   : 1;

    int _numFaceVertsTotal;

    Vtr::internal::StackBuffer<VertexTopology,8> _vertexTopology;
    Vtr::internal::StackBuffer<int,8,true>       _faceInVertex;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_FACE_TOPOLOGY_H */
