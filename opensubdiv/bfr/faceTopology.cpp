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

#include <cstring>
#include <cstdio>

#include "../bfr/faceTopology.h"
#include "../sdc/crease.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {


//
//  Constructor needs the same Sdc scheme/options as the Factory to
//  support internal work -- may need to figure another way to assign
//  these if we later need a default constructor for some purpose...
//
FaceTopology::FaceTopology(Sdc::SchemeType schemeType,
                           Sdc::Options schemeOptions) :
    _schemeType(schemeType),
    _schemeOptions(schemeOptions),
    _regFaceSize(Sdc::SchemeTypeTraits::GetRegularFaceSize(schemeType)),
    _isInitialized(false) {

}

//
//  Main initialize/finalize used by base factory to delimit assignment:
//
void
FaceTopology::Initialize(int faceSize) {

    _faceSize = faceSize;
    _numFaceVertsTotal = 0;

    _combinedTags.Clear();

    _isInitialized = true;
    _isFinalized   = false;

    _corner.SetSize(faceSize);
}

void
FaceTopology::Finalize() {

    //
    //  Inspect all corner vertex topologies -- accumulating the presence
    //  of irregular features for the face and assigning other internal
    //  members used to assemble the limit surface:
    //
    //  WIP - potentially want to identify presence of degenerate faces
    //  below too, i.e. face size < 3.  A subclass may specify these in
    //  an ordered set and that would mess up some of the topological
    //  traversals.  In such case, we can initialize the vertex subset
    //  to excludes such faces -- treating their edges as non-manifold.
    //
    //  Probably need to add yet another bit per vertex here to know when
    //  to process an otherwise simple manifold ring, i.e. hasDegenFaces
    //
    assert(_isInitialized);

    for (int i = 0; i < _faceSize; ++i) {
        CornerTopology & cTop  = GetTopology(i);
        CornerTags       cTags = cTop.GetTags();

        _combinedTags.BitwiseOr(cTags);

        _numFaceVertsTotal += cTop.GetNumFaceVertices();
    }

    _isFinalized = true;
}

void
FaceTopology::print(Index const faceVertIndices[]) const {

    printf("FaceTopology:\n");
    printf("    face size      = %d\n", _faceSize);
    printf("    num-face-verts = %d\n", _numFaceVertsTotal);
    printf("  Tags:\n");
    printf("    inf-sharp verts  = %d\n", _combinedTags._infSharpVerts);
    printf("    semi-sharp verts = %d\n", _combinedTags._semiSharpVerts);
    printf("    any sharp edges  = %d\n", _combinedTags._anySharpEdges);
    printf("    unsharp boundary = %d\n", _combinedTags._boundaryNonSharp);
    printf("    irregular faces  = %d\n", _combinedTags._irregularFaceSizes);
    printf("    unordered verts  = %d\n", _combinedTags._unOrderedFaces);
    printf("    val-2 int verts  = %d\n", _combinedTags._interiorVal2Verts);

    if (faceVertIndices) {
        Index const * cornerFaceVertIndices = faceVertIndices;

        for (int i = 0; i < _faceSize; ++i) {
            printf("    corner %d:\n", i);

            CornerTopology const & cTop = GetTopology(i);
            printf("        topology:  num faces  = %d, boundary = %d\n",
                    cTop.GetNumFaces(), cTop.IsBoundary());

            printf("        face-vert indices:\n");

            for (int j = 0, n = 0; j < cTop.GetNumFaces(); ++j) {
                printf("        face %d:  ", j);
                int S = cTop.GetFaceSize(j);
                for (int k = 0; k < S; ++k, ++n) {
                    printf("%3d", cornerFaceVertIndices[n]);
                }
                printf("\n");
            }
            cornerFaceVertIndices += cTop.GetNumFaceVertices();
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
