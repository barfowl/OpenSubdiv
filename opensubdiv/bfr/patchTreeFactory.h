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

#ifndef OPENSUBDIV3_BFR_PATCH_TREE_FACTORY_H
#define OPENSUBDIV3_BFR_PATCH_TREE_FACTORY_H

#include "../version.h"

#include "../bfr/patchTree.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class TopologyRefiner;
}

namespace Bfr {

//
//  WIP - this stateless Factory with static Create() methods reflects
//        the origin of PatchTree within Far
//      - this Factory class will be replaced with some kind of Builder
//
//  PatchTreeFactory with static method constructing PatchTrees:
//
class PatchTreeFactory {
public:

    //
    //  Minimize the number of shape approximating Options here (compared
    //  to the Far classes):
    //
    //  Note that the "interior patches" capability of PatchTree is not
    //  used in Bfr and so is never enabled. It is left available as a
    //  reminder of that ability for future use.
    //
    struct Options {
        enum BasisType { REGULAR, GREGORY, LINEAR };

        Options(int depth = 4) : irregularBasis(GREGORY),
                                 maxPatchDepthSharp(depth),
                                 maxPatchDepthSmooth(15),
                                 includeInteriorPatches(false),
                                 useDoublePrecision(false) { }

        unsigned int irregularBasis         : 4;
        unsigned int maxPatchDepthSharp     : 4;
        unsigned int maxPatchDepthSmooth    : 4;
        unsigned int includeInteriorPatches : 1;
        unsigned int useDoublePrecision     : 1;
    };

    //
    //  Create a PatchTree from the first face of a TopologyRefiner
    //  representing a small local neighborhood of that face:
    //
    static PatchTree * Create(Far::TopologyRefiner & faceRefiner,
                              Options options = Options());
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_PATCH_TREE_FACTORY_H */
