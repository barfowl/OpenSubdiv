//
//   Copyright 2018 DreamWorks Animation LLC
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

#ifndef OPENSUBDIV3_FAR_BASE_PATCH_FACTORY_H
#define OPENSUBDIV3_FAR_BASE_PATCH_FACTORY_H

#include "../version.h"

#include "../far/basePatch.h"
#include "../far/patchTreeFactory.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

class PatchTreeCache;

//
//  BasePatchFactory for constructing a BasePatch from the face of a mesh.
//
//  Eventually prefer this to be a template in terms of <MESH> so that clients
//  can deal with their own mesh classes and not be forced to create an entire
//  TopologyRefiner.
//
//  Currently using the static factory method convention here, but want to
//  eventually consider a BasePatchFactory<MESH> that is instiated with both
//  an instance of <MESH> and a set of Options for which BasePatches for all
//  faces will then be consistently generated.  An instance of such a Factory
//  could manage its own PatchTreeCache internally for all faces of the mesh.
//
class BasePatchFactory {
public:

    //  Prefer to avoid -- or at least minimize -- the number of shape
    //  approximating options here...
    //
    struct Options {
        Options() : patchTreeOptions(4),
                    updatePatchTreeCache(false),
                    patchTreeCache(0) { }

        PatchTreeFactory::Options patchTreeOptions;

        unsigned int updatePatchTreeCache : 1;

        PatchTreeCache * patchTreeCache;
    };

    //  A BasePatch is constructed from a mesh and a specified base face:
    //
    static BasePatch * Create(TopologyRefiner const & meshTopology,
                              Index                   baseFace,
                              Options                 options = Options());
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_BASE_PATCH_FACTORY_H */
