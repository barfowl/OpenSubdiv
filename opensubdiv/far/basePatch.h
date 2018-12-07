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

#ifndef OPENSUBDIV3_FAR_BASE_PATCH_H
#define OPENSUBDIV3_FAR_BASE_PATCH_H

#include "../version.h"

#include "../far/patchTree.h"
#include "../far/stencilTable.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  BasePatch is the main client-facing class that provides a potentially
//  piecewise piece of limit surface associated with the base face of a
//  mesh.
//
//  Depending on how we want to deal with irregular (non-quad or non-tri)
//  faces and face-varying channels, the BasePatch may manage one or more
//  PatchTrees.  For now, we limit support to regular base faces whose
//  limit surface is represented by a single PatchTree.
//
//  Since PatchTrees are shared topological structures with a local set
//  of control vertices, the BasePatch needs to identify the vertices in
//  its base mesh that correspond to control points of a PatchTree.
//
//  Ideally we want the common cases of regular patches to be as efficient
//  as possible -- which may warrant a simple patch as an alternative to
//  the more general PatchTree in cases where no "tree" is necessary.
//
class BasePatch {
public:
    //  Constructors are protected
    ~BasePatch();

    //
    //  Methods related to control points:
    //
    int GetNumControlPoints() const { return _patchTree->GetNumControlPoints(); }

    ConstIndexArray GetControlPointIndices() const {
        return ConstIndexArray(&_controlPoints[0], (int)_controlPoints.size());
    }

    template <class T, class U>
    void GetControlPointValues(T const & src, U & dst) const {
        for (int i = 0; i < (int)_controlPoints.size(); ++i) {
            //  Cannot guarantee T copy-constructible
            dst[i].Clear();
            dst[i].AddWithWeight(src[_controlPoints[i]], 1.0f);
        }
    }

    //
    //  Methods related to additional points (of sub-patches):
    //
    int GetNumSubPatchPoints() const{ return _patchTree->GetNumSubPatchPoints(); }
    int GetNumPointsTotal() const   { return _patchTree->GetNumPointsTotal(); }

    //  Dealing directly with the StencilTable provides more flexibility than a
    //  single template method that applies it (like the PatchTable):
    StencilTableReal<float> const * GetSubPatchPointStencilTable() const {
        return _patchTree->GetStencilTable();
    }

    //
    //  Evaluation methods:
    //
    //  Currently this matches the PatchTree interfaces -- prefer to avoid the
    //  explicit "sub patch" requirement somehow...
    //
    int FindSubPatch(float u, float v, int maxDepth = -1) const {
        return _patchTree->FindSubPatch(u, v, maxDepth);
    }

    ConstIndexArray GetSubPatchPoints(int subPatch) const {
        return _patchTree->GetSubPatchPoints(subPatch);
    }

    int EvalSubPatchBasis(int subPatch, float u, float v, float wP[], float wDu[], float wDv[]) const {
        return _patchTree->EvalSubPatchBasis(subPatch, u, v, wP, wDu, wDv);
    }

protected:
    BasePatch();
    friend class BasePatchFactory;

private:
    //  Private members:

    //  Vector of control point indices defining the patch for a face:
    std::vector<Index> _controlPoints;

    //  The PatchTree storing the hierarchy of sub-patches, stencils, etc.
    PatchTree const * _patchTree;
    bool              _patchTreeIsExternal;
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_BASE_PATCH */
