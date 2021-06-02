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

#ifndef OPENSUBDIV3_FAR_PATCH_TREE_H
#define OPENSUBDIV3_FAR_PATCH_TREE_H

#include "../version.h"

#include "../far/patchDescriptor.h"
#include "../far/patchParam.h"
#include "../far/stencilTable.h"

#include "../sdc/options.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  WORK IN PROGRESS...
//
//  A PatchTree is a hierarchical collection of sub-patches that form a
//  piecewise representation of the limit surface for a single face of a
//  mesh.  Using the same patch representations as other Far classes, it
//  combines stripped down versions of the PatchTable, PatchMap and
//  StencilTable into a more compact representation suited to evaluating
//  a single face. These are constructed based on adaptive refinement of
//  a single face of a TopologyRefiner.
//
//  Note that PatchTree is not publicly exposed. As with other Far classes
//  that are for internal use but not publicly exposed, it is not added
//  to the "internal" namespace.
//
//  The PatchTree is intended for internal use by other classes that will
//  provide a simpler interface to the limit surface for the base faces
//  of a mesh.  It is kept in Far as it's use of other Far classes, e.g.
//  TopologyRefiner and StencilTable, currently requires protected access
//  (which is not typically granted to classes outside Far).
//
class PatchTree {
public:
    //  Constructors are protected
    ~PatchTree();

    //  Simple public accessors:
    int GetNumControlPoints() const  { return _numControlPoints; }
    int GetNumSubPatchPoints() const { return _numSubPatchPoints; }
    int GetNumPointsTotal() const    { return _numControlPoints + _numSubPatchPoints; }

    //  These may not be necessary...
    int GetDepth() const      { return _treeDepth; }
    int GetNumPatches() const { return (int)_patchParams.size(); }

    StencilTableReal<float> const * GetStencilTable() const { return _stencilTable; }

    //  Methods supporting evaluation:
    int HasSubFaces() const    { return _numSubFaces > 0; }
    int GetNumSubFaces() const { return _numSubFaces; }

    int FindSubPatch(float u, float v, int subFace = 0, int maxDepth = -1) const {
        return searchQuadtree(u, v, subFace, maxDepth);
    }

    ConstIndexArray GetSubPatchPoints(int subPatch) const;
    PatchParam      GetSubPatchParam( int subPatch) const { return _patchParams[subPatch]; }

    int EvalSubPatchBasis(int subPatch, float u, float v, float w[], float wDu[], float wDv[]) const;

protected:
    PatchTree();
    friend class PatchTreeBuilder;

protected:
    //  Internal quad-tree node type and assembly and search methods:
    struct TreeNode {
        struct Child {
            unsigned int isSet  :  1;
            unsigned int isLeaf :  1;
            unsigned int index  : 30;
        };

        TreeNode() { patchIndex = -1; std::memset(children, 0, sizeof(children)); }

        void SetChildren(int index);
        void SetChild(int quadrant, int index, bool isLeaf);

        int   patchIndex;
        Child children[4];
    };

    int  searchQuadtree(float u, float v, int subFace = 0, int depth = -1) const;
    void buildQuadtree();

    TreeNode * assignLeafOrChildNode(TreeNode * node, bool isLeaf, int quadrant, int index);

private:
    //  Private members:
    typedef PatchDescriptor::Type PatchType;

    //  Simple inventory and topology members:
    int       _numSubFaces;
    int       _numControlPoints;
    int       _numSubPatchPoints;
    bool      _patchesAreTriangular;
    bool      _patchesIncludeNonLeaf;
    PatchType _regPatchType;
    int       _regPatchSize;
    PatchType _irregPatchType;
    int       _irregPatchSize;
    int       _patchPointStride;

    //  Vectors for points and PatchParams of all patches:
    //
    //  Note we store both regular and irregular patch point indices in the same
    //  vector (using a common stride for each patch) and the patch type determined
    //  by the PatchParam -- in the same way that face-varying patches are stored in
    //  the PatchTable.  Could also be stored in separate "patch arrays" or separated
    //  in other ways and manged with a bit more book-keeping.
    //
    std::vector<Index>      _patchPoints;
    std::vector<PatchParam> _patchParams;

    //  The quadtree organizing the patches:
    std::vector<TreeNode>  _treeNodes;
    int                    _treeDepth;

    //  Stencils for computing points of all patches:
    StencilTableReal<float> const * _stencilTable;
};

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_PATCH_TREE */
