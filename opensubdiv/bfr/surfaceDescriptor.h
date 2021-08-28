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

#ifndef OPENSUBDIV3_BFR_SURFACE_DESCRIPTOR_H
#define OPENSUBDIV3_BFR_SURFACE_DESCRIPTOR_H

#include "../version.h"

#include "../bfr/cornerTopology.h"
#include "../bfr/faceTopology.h"

#include "../vtr/types.h"
#include "../vtr/stackBuffer.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  CornerSubset is a simple struct that identifies a topological subset
//  around a vertex (i.e. CornerTopology).  Such subsets are what really
//  define the surface around a face and so are used by SurfaceDescriptor.
//
struct CornerSubset {
    //  Members defining the extent of the subset:
    short _numFacesBefore;
    short _numFacesAfter;
    short _numFacesTotal;

    //  The definition is completed with the boundary and sharp bits --
    //  now part of the CornerTag (and later combined).  Simple get/set
    //  methods are provided to avoid the tedious syntax of the tag:
    CornerTag _tag;

    bool IsBoundary() const { return _tag._boundaryVerts; }
    bool IsSharp()    const { return _tag._infSharpVerts; }

    void SetBoundary(bool on) { _tag._boundaryVerts = on; }
    void SetSharp(bool on)    { _tag._infSharpVerts = on; }
};


//
//  SurfaceDescriptor combines references to several other classes and
//  data to provide a complete description of the limit surface of a face.
//
//  It is a simple aggregate of three sets of data:
//      - an instance of FaceTopology with all topological information
//      - a set of CornerSubsets for topological extent of each corner
//      - a set of indices associate with all vertices of FaceTopology
//  with a few additional members summarizing features of these.
//
//  SurfaceDescriptors are constructed/initialized in two ways:
//      - for the vertex topology of a face:
//          - requiring FaceTopology and associated vertex indices
//      - for the face-varying topology of a face:
//          - requiring FaceTopology and associated face-varying indices
//          - and additionally a SurfaceDescriptor with vertex topology,
//            from which face-varying subsets are determined
//
//  Once initialized, other than a few simple queries, it serves solely
//  as a container to be passed to other classes to assemble into regular
//  or irregular surfaces.
//
class SurfaceDescriptor {
public:
    SurfaceDescriptor(FaceTopology const & topology) :
                      _topology(topology), _isInitialized(false) { }
    ~SurfaceDescriptor() { }

    //  Requires initialization for vertex or face-varying topology:
    //  WIP - consider making these constructors instead
    void InitializeVertex(Index const vtxIndices[]);

    void InitializeFaceVarying(Index const fvarIndices[],
                               SurfaceDescriptor const & vtxSurface);

    //   Main public methods to distinquish surface and topology:
    bool IsRegular() const { return _isRegular; }

    bool MatchesVertexTopology() const { return _matchesVertex; }

    //  Debugging:
    void print(bool printVerts = false) const;

public:
    //  Public access to the main members:
    FaceTopology const & GetTopology() const { return _topology; }
    CornerSubset const * GetSubsets()  const { return _corners; }
    Index        const * GetIndices()  const { return _indices; }
    CombinedTag          GetTag()      const { return _combinedTag; }

public:
    //  Additional public access to date used by builder classes:
    int GetFaceSize() const;
    int GetRegFaceSize() const;

    Sdc::SchemeType GetSchemeType() const;
    Sdc::Options    GetSchemeOptions() const;

    CornerTopology const & GetCornerTopology(int corner) const;
    CornerSubset   const & GetCornerSubset(int corner) const;

    int GetNumIndices() const;

private:
    //  Internal methods for supporting face-varying initialization:
    void initialize(int faceSize, Index const indices[]);

    void extendFVarSubset(CornerSubset         & fvarSubset,
                          CornerSubset const   & vtxSubset,
                          CornerTopology const & cornerTopology,
                          Index const            fvarIndices[]);

    void sharpenFVarSubset(CornerSubset         & fvarSubset,
                           CornerSubset const   & vtxSubset,
                           CornerTopology const & cornerTopology,
                           Index const            fvarIndices[]);

    bool isRegular() const;

private:
    typedef Vtr::internal::StackBuffer<CornerSubset,8,true> CornerArray;

    FaceTopology const & _topology;
    Index        const * _indices;
    CornerArray          _corners;
    CombinedTag          _combinedTag;

    //  Members here reflecting collective properties of the corners:
    unsigned int _isInitialized : 1;
    unsigned int _isRegular     : 1;
    unsigned int _isFaceVarying : 1;
    unsigned int _matchesVertex : 1;
};

//
//  Inline accessors:
//
inline int
SurfaceDescriptor::GetFaceSize() const {
    return _topology.GetFaceSize();
}
inline int
SurfaceDescriptor::GetRegFaceSize() const {
    return _topology.GetRegFaceSize();
}

inline Sdc::SchemeType
SurfaceDescriptor::GetSchemeType() const {
    return _topology._schemeType;
}
inline Sdc::Options
SurfaceDescriptor::GetSchemeOptions() const {
    return _topology._schemeOptions;
}

inline CornerTopology const &
SurfaceDescriptor::GetCornerTopology(int corner) const {
    return _topology.GetTopology(corner);
}

inline CornerSubset const & 
SurfaceDescriptor::GetCornerSubset(int corner) const {
    return _corners[corner];
}

inline int
SurfaceDescriptor::GetNumIndices() const {
    return _topology.GetNumFaceVertices();
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE_DESCRIPTOR_H */
