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

#ifndef OPENSUBDIV3_BFR_FACE_DESCRIPTORS_H
#define OPENSUBDIV3_BFR_FACE_DESCRIPTORS_H

#include <vector>

#include "../version.h"

#include "../vtr/stackBuffer.h"
#include "../far/topologyDescriptor.h"
#include "../bfr/topologyCache.h"
#include "../bfr/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  WORK IN PROGRESS...  Major changes are planned here.  In particular,
//  the separate public structs for dealing with manifold and non-manifold
//  topology will be combined into one.  Whether a special case to deal
//  with regular topology is also being reconsidered.
//
//  This header includes a number of simple structs that allow clients to
//  define the complete topology around the face of a mesh -- from which
//  the limit surface for the face is then determined.
//
//  More than one struct is available to allow common cases to be specified
//  and processed quickly, while more complicated cases (e.g. the presence
//  of an extremely high valence vertex, or non-manifold topology) will
//  require more effort.
//
//  Cases to consider include:
//      - "regular" for purely regular topology
//      - "simple" for low-valence, manifold and no irregular faces
//      - "general" for all others -- irregular faces, non-manifold, etc.
//
//  Consider including "hints" as members of Descriptors that clients can
//  optionally set to accelerate topological determination -- particularly
//  for the regular case -- as their meshes often have such information
//  already available.
//
//  Also consider providing some kind of public Validate() methods that
//  clients can use prior to Finalize() when debugging to help identify
//  problems with their specification.
//


//
//  WIP - the RegularFaceDescriptor may be useful to clients that have a
//  mesh representation that is subdivision-aware, and which has already
//  determined where the topology is regular for other purposes.  In such
//  cases, the RegularFaceDescriptor simply allows the client to identify
//  the mesh vertices corresponding to those of a regular patch.
//
class RegularFaceDescriptor {
public:
    RegularFaceDescriptor() : _patchSize(0), _boundaryMaskSet(0) { }
    ~RegularFaceDescriptor() { }

    //  Primary methods to initialize and finalize:
    void Initialize(int patchSize);
    bool Finalize();
    bool IsFinalized() const { return _finalized; }

    //  Methods to access arrays of indices associated with patch points:
    int * AccessPatchVertexIndices();
    int * AccessPatchFVarValueIndices();

    //  The boundary mask can be set explicitly if known, otherwise it
    //  will be determined from the assigned point indices:
    void SetBoundaryMask(int boundaryMask);

protected:
    //  Protected/private members:
    unsigned int _faceSize        : 3;
    unsigned int _patchSize       : 5;
    unsigned int _boundaryMask    : 6;
    unsigned int _boundaryMaskSet : 1;
    unsigned int _fvarPatchSet    : 1;
    unsigned int _finalized       : 1;

    int _numCVs;

    int _patchPoints[16];
    int _fvarPatchPoints[16];
};

inline void
RegularFaceDescriptor::SetBoundaryMask(int mask) {
    _boundaryMask    = mask;
    _boundaryMaskSet = true;
}
inline int *
RegularFaceDescriptor::AccessPatchVertexIndices() {
    return &_patchPoints[0];
}
inline int *
RegularFaceDescriptor::AccessPatchFVarValueIndices() {
    _fvarPatchSet = true;
    return &_fvarPatchPoints[0];
}


//
//  WIP - plans are to combine the ManifoldFaceDescriptor described here
//  with the NonManifoldFaceDescriptor that follows.  The ability to
//  simplify manifold rings (or half-rings) for the corners of a face
//  will be preserved, while including interface extensions to specify a
//  non-manifold vertex for only those corners that require it.
//
//  The ManifoldFaceDescriptor is likely going to be most preferred
//  by clients that do not have to deal with non-manifold meshes.  The
//  specification and required inspection will both be more efficient
//  than the more general descriptor that supports non-manifold cases.
//
//  WIP - a couple more things to consider here...
//      - consider exposing a VertexDescriptor to use for each corner
//      - revisit Access() methods returning points:
//          - a Set() and internal memcpy() would be clearer
//      - make "constFaceSize" a per-corner property:
//          - to be specified when setting num incident faces
//      - consider a public Validate() separate from Finalize():
//          - removes ambiguity about bool return values
//
class ManifoldFaceDescriptor {
public:
    ManifoldFaceDescriptor() : _faceSize(0), _constFaceSize(false) { }
    virtual ~ManifoldFaceDescriptor() { }

    //  Primary methods to initialize and finalize:
    void Initialize(int faceSize, bool constFaceSize = false);
    bool Finalize();
    bool IsFinalized() const { return _finalized; }

    //  Methods to assign one-ring topology for each corner:
    void SetCornerNumIncidentFaces(int corner, int numFaces);
    void SetCornerBoundary(int corner, int faceInBoundaryRing);
    void SetCornerVertexIndex(int corner, Index vtxIndex);
    void SetCornerFVarValueIndex(int corner, Index fvarIndex);

    int * AccessCornerIncidentFaceSizes(int corner);
    int * AccessCornerRingVertexIndices(int corner);

protected:
    //  Protected/private members:
    struct Corner {
        int        _vertexIndex;
        int        _fvarValueIndex;
        bool       _isBoundary;
        LocalIndex _numIncFaces;
        LocalIndex _faceInRing;

        Vtr::internal::StackBuffer<int, 8,true> _incFaceSizes;
        Vtr::internal::StackBuffer<int,16,true> _ringVertIndices;

        int _numLocalVerts;
        int _numLocalFaces;
    };
    Vtr::internal::StackBuffer<Corner,8,false> _corners;

    unsigned int _faceSize       : 16;
    unsigned int _constFaceSize  :  3;
    unsigned int _fvarIndicesSet :  1;
    unsigned int _finalized      :  1;

    int _numVerts;
    int _numFaces;
    int _numFaceVerts;
};

//  Methods to assign one-ring topology for corners of ManifoldFaceDescriptor:
inline void
ManifoldFaceDescriptor::SetCornerVertexIndex(int corner, Index vtxIndex) {
    _corners[corner]._vertexIndex = vtxIndex;
}
inline void
ManifoldFaceDescriptor::SetCornerFVarValueIndex(int corner, Index fvarIndex) {
    _corners[corner]._fvarValueIndex = fvarIndex;
    _fvarIndicesSet = true;
}
inline void
ManifoldFaceDescriptor::SetCornerNumIncidentFaces(int corner, int numFaces) {
    _corners[corner]._numIncFaces = numFaces;

    //  Allocate inc face sizes -- ring size must be deferred until
    //  inc face sizes are known, so set to 0 to trigger resizing:
    _corners[corner]._incFaceSizes.SetSize(numFaces);
    _corners[corner]._ringVertIndices.SetSize(0);
}
inline void
ManifoldFaceDescriptor::SetCornerBoundary(int corner, int faceInBoundaryRing) {
    _corners[corner]._isBoundary = true;
    _corners[corner]._faceInRing = faceInBoundaryRing;
}
inline int *
ManifoldFaceDescriptor::AccessCornerIncidentFaceSizes(int corner) {
    return &_corners[corner]._incFaceSizes[0];
}
inline int *
ManifoldFaceDescriptor::AccessCornerRingVertexIndices(int corner) {

    //  If the face sizes vary and were explicitly specified, make sure
    //  space for the ring has been allocated before returning it:
    Corner & C = _corners[corner];
    if (C._ringVertIndices.GetSize() == 0) {
        int ringSize = C._isBoundary;
        if (_constFaceSize) {
            ringSize += C._numIncFaces * (_constFaceSize-2);
        } else {
            for (int i = 0; i < C._numIncFaces; ++i) {
                ringSize += (C._incFaceSizes[i] - 2);
            }
        }
        C._ringVertIndices.SetSize(ringSize);
    }
    return &C._ringVertIndices[0];
}


//
//  Note -- this NonManifoldFaceDescriptor, which makes use of the
//  Far::TopologyDescriptor, is only a preliminary implementation to
//  support more complete coverage of topology for early testing.  A
//  simpler mechanism is eventually needed here and will be provided
//  as extensions to the ManifoldFaceDescriptor above (thus requiring
//  the "Manifold" name to be dropped).
//
class NonManifoldFaceDescriptor {
public:
    NonManifoldFaceDescriptor() : _finalized(false) { }
    virtual ~NonManifoldFaceDescriptor() { }

    void Initialize();
    void Finalize();
    bool IsFinalized() const { return _finalized; }

public:
    //  Members explicitly assigned and bound to the TopologyDescriptor:
    struct Data {
        std::vector<int>   faceSizes;
        std::vector<Index> faceVerts;
        std::vector<Index> faceFVarValues;
        std::vector<Index> creaseIndices;
        std::vector<float> creaseSharpness;
        std::vector<Index> cornerIndices;
        std::vector<float> cornerSharpness;
        std::vector<Index> faceHoles;
    };
    Data _data;

    Far::TopologyDescriptor _topology;

    std::vector<int> _controlVertices;
    bool             _finalized;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_FACE_DESCRIPTORS_H */
