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
//  SurfaceDescriptor is a simple class aggregating the data that
//  defines a limit surface:  the entire topology around the base
//  face, the control vertex indices of the same neighborhood, and
//  the subsets of each corner of the face that contribute to the
//  limit surface.
//
class SurfaceDescriptor {
public:
    SurfaceDescriptor(FaceTopology const & topology) :
                      _topology(topology), _isInitialized(false) { }
    ~SurfaceDescriptor() { }

    //  Requires initialization for vertex or face-varying topology:
    void Initialize(Index const vtxIndices[]);

    void InitializeFaceVarying(SurfaceDescriptor const & vtxSurface,
                               Index const fvarIndices[]);

    //  Debugging:
    void print(bool printVerts = false) const;

public:
    //   Main public methods to distinquish surface and topology:
    bool IsRegular() const;

    bool MatchesVertexTopology() const { return _matchesVertex; }

public:
    //  Public access to the main members:
    FaceTopology const & GetTopology() const { return _topology; }
    CornerSubset const * GetSubsets()  const { return _corners; }
    Index        const * GetIndices()  const { return _indices; }
    CornerTags           GetTags()     const { return _combinedTags; }

public:
    //  Additional public access to FaceTopology members:
    int GetFaceSize() const;
    int GetRegFaceSize() const;

    Sdc::SchemeType GetSchemeType() const;
    Sdc::Options    GetSchemeOptions() const;

    CornerTopology const & GetCornerTopology(int corner) const;
    CornerSubset   const & GetCornerSubset(int corner) const;

    int GetNumIndices() const;

private:
    //  Internal methods for dealing with corner topology arrays:
    void initializeFVarSubset(SurfaceDescriptor const & vtxSurface,
                              int corner, Index const fvarIndices[]);

    void sharpenFVarSubset(SurfaceDescriptor const & vtxSurface,
                           int corner, Index const fvarIndices[]);

    void initializeSubsetInventory();

private:
    typedef Vtr::internal::StackBuffer<CornerSubset,8,true> CornerArray;

    FaceTopology const & _topology;
    Index        const * _indices;
    CornerArray          _corners;
    CornerTags           _combinedTags;

    //  Members here reflecting collective properties of the corners:
    unsigned int _isInitialized : 1;
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
