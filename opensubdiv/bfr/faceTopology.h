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
//  of a base face that defines the limit surface for that face.
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
//  The Factory currently identifies and manages these components, but
//  its a bit awkward and messy in its present form, and FaceTopology
//  has a lot more functionality than is probably warranted -- given it
//  is the only class involved here other than the Factory.
//
//  So a small "ecosystem" of classes are planned to improve this...
//
//  VertexTopology:
//      - this is our public-face class populated by Factory subclasses
//      - it will remain so but stripped down to serve only those needs
//
//  CornerTopology:
//      - will either derive from or contain a VertexTopology to access
//      - will contain tags reflecting collective properties of the vertex:
//          - some may move here from VertexTopology, others will be new
//      - will include additional members that correlate to the faces of
//        VertexTopology for additional processing:
//          - an array of ints for "unordered face neighbors" is planned
//            to deal with unordered faces in VertexTopology (which will
//            allow support of non-manifold topology)
//      - will include the "face-in-vertex" member for the corner
//
//  CornerSubset:
//      - will remain unchanged -- still a very simple struct
//
//  FaceSubset:
//      - will contain an array of CornerSubsets
//      - also to contain tags reflecting collective properties of the array
//      - instances of these collections will be managed by the Factory
//      - may also include a reference to the FaceTopology which contains it
//      - may also include (optionally) an assigned set of indices
//
//  FaceTopology:
//      - will contain an array of CornerTopology (rather than VertexTopology)
//      - also to contain tags reflecting collective properties of the array
//      - currently debating if a FaceSubset should be included:
//          - this would replace the current array of CornerSubsets that
//            reflect the vertex topology
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
protected:
    friend class LimitSurfaceFactory;

protected:
    FaceTopology(Sdc::SchemeType schemeType,
                 Sdc::Options    schemeOptions);
    ~FaceTopology() { }

    void Initialize(int faceSize);
    void Finalize();

    int GetFaceSize() const { return _faceSize; }

    bool HasLimit() const;
    bool IsRegular(CornerSubset const cornerSubsets[] = 0) const;

    //  WIP - will be removed once all features supported
    //      - REMEMBER that Loop patches NOT fully supported:
    //          - regular patches also not complete for Loop
    bool IsUnsupported() const {
        if (_hasVal2IntVerts || _hasNonManCorners) {
            return true;
        }
        return false;
    }

protected:
    //
    //  More involved inspection and construction methods:
    //
    //  Initializing corner subsets for vertex and face-varying:
    //
    void initializeSubsetInventory(CornerSubset cornerSubsets[]) const;

    void InitializeVertexSubsets(Index const fvtxIndices[] = 0);

    bool IdentifyFaceVaryingSubsets(Index const  fvarIndices[],
                                    CornerSubset fvarSubsets[]) const;

    void findFaceVaryingSubset(int corner,
                               Index const    fvarIndices[],
                               CornerSubset & fvarSubset) const;

    void sharpenFaceVaryingSubset(int corner,
                                  Index const    fvarIndices[],
                                  CornerSubset & fvarSubset) const;

    //
    //  Inspecting the overall topology:
    //
    int GetNumControlVertices(CornerSubset const faceSubsets[]) const;
    int GetNumControlFaces(   CornerSubset const faceSubsets[]) const;

    //
    //  Gathering complete topology information for external use:
    //
    int GatherControlVertexIndices(CornerSubset const faceSubsets[],
                                   Index        const faceIndices[],
                                   Index              cvIndices[]) const;
    int GatherControlFaceSizes(CornerSubset const faceSubsets[],
                               int                faceSizes[]) const;
    int GatherControlFaceVertices(CornerSubset const faceSubsets[],
                                  int                numControlVertices,
                                  int                faceVertices[]) const;
    int GatherControlVertexSharpness(CornerSubset const faceSubsets[],
                                     int                cornerVerts[],
                                     float              vertSharpness[]) const;
    int GatherControlEdgeSharpness(CornerSubset const faceSubsets[],
                                   int                edgeVertPairs[],
                                   float              edgeSharpness[]) const;

    //
    //  Gathering control vertex indices for regular patches:
    //
    void GatherRegularPatchPoints4(CornerSubset const faceSubsets[],
                                   Index        const faceIndices[],
                                   Index              patchPoints[]) const;
    void GatherRegularPatchPoints3(CornerSubset const faceSubsets[],
                                   Index        const faceIndices[],
                                   Index              patchPoints[]) const;

    //
    //  Computing the hashing key for the TopologyCache:
    //
    TopologyCache::Key ComputeTopologyKey(CornerSubset const subsets[]) const;

    //
    //  Debugging...
    //
    void print(Index const faceIndices[], bool printVertInfo) const;
    void printControlTopology(Index const faceIndices[]) const;
    void printSubsets(CornerSubset const faceSubsets[]) const;

private:
    Sdc::SchemeType _schemeType;
    Sdc::Options    _schemeOptions;

    int _faceSize;
    int _regFaceSize;

    unsigned int _hasBoundaryVerts : 1;
    unsigned int _hasSharpVerts    : 1;
    unsigned int _hasSharpEdges    : 1;
    unsigned int _hasIncIrregFaces : 1;
    unsigned int _hasNonManCorners : 1;
    unsigned int _hasVal2IntVerts  : 1;

    unsigned int _isInitialized : 1;
    unsigned int _isFinalized   : 1;

    int _numFaceVertsTotal;

    Vtr::internal::StackBuffer<VertexTopology,8>    _vertexTopology;
    Vtr::internal::StackBuffer<int,8,true>          _faceInVertex;
    Vtr::internal::StackBuffer<CornerSubset,8,true> _cornerSubsets;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_FACE_TOPOLOGY_H */
