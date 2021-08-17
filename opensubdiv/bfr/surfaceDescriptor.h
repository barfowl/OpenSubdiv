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

    //  Debugging:
    void print(bool printVerts = false) const;

public:
    void Initialize(Index const vtxIndices[]);

    void InitializeFaceVarying(SurfaceDescriptor const & vtxSurface,
                               Index const fvarIndices[]);

    int GetFaceSize() const { return _topology.GetFaceSize(); }

    bool IsRegular() const;

    bool MatchesVertexTopology() const { return _matchesVertex; }

private:
    //  Internal methods for dealing with corner topology arrays:
    void initializeCornerInventory();

    void initializeFVarCorner(int corner, CornerSubset const & vtxCorner,
                                          Index        const   fvarIndices[]);

    void sharpenFVarCorner(int corner, CornerSubset const & vtxCorner,
                                       Index        const   fvarIndices[]);

public:
    typedef Vtr::internal::StackBuffer<CornerSubset,8,true> CornerArray;

    FaceTopology const & _topology;
    Index        const * _indices;
    CornerArray          _corners;

    //  Members here reflecting collective properties of the corners:
    unsigned int _isInitialized : 1;
    unsigned int _isFaceVarying : 1;
    unsigned int _matchesVertex : 1;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE_DESCRIPTOR_H */
