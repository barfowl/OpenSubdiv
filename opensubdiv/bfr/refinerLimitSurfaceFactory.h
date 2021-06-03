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

#ifndef OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H
#define OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H

#include "../version.h"

#include "../bfr/limitSurfaceFactory.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class TopologyRefiner;
    class PatchBuilder;
}

namespace Bfr {

//
//  Subclass of LimitSurfaceFactory using Far::TopologyRefiner as the mesh
//  class:
//
//  WIP - The virtual interface is going to be changing here.  What exists
//  now allows this subclass to support all possible topological cases, but
//  not as easily as we would like others to do in future.
//
class RefinerLimitSurfaceFactory : public LimitSurfaceFactory {
public:
    //
    //  Subclass-specific constructor/destructor and queries:
    //
    RefinerLimitSurfaceFactory(Far::TopologyRefiner const & mesh,
                               Options options = Options());
    virtual ~RefinerLimitSurfaceFactory();

    Far::TopologyRefiner const & GetMesh() const { return _mesh; }

protected:
    //
    //  Virtual methods required by the base class:
    //
    bool isFaceHole(Index baseFace) const;

    bool populateDescriptor(Index baseFace, RegularFaceDescriptor &) const;
    bool populateDescriptor(Index baseFace, ManifoldFaceDescriptor &) const;
    bool populateDescriptor(Index baseFace, NonManifoldFaceDescriptor &) const;

protected:
    //  Internal support (should be able to hide these from public header):
    bool isFaceLimitRegular(Index baseFace) const;

private:
    //  Additional members for the subclass:
    Far::TopologyRefiner const & _mesh;

    bool _populateFVarTopology;

    Far::PatchBuilder const * _patchBuilder;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H */
