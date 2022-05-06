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

#include "../sdc/options.h"

#include <vector>
#include <cstring>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  WIP ...
//
//  A PatchTree is a hierarchical collection of sub-patches that form a
//  piecewise representation of the limit surface for a single face of a
//  mesh.  Using the same patch representations as other Far classes, it
//  combines stripped down versions of the PatchTable, PatchMap and a
//  raw representation of stencils into a more compact representation
//  suited to evaluating a single face. These are constructed based on
//  adaptive refinement of a single face of a TopologyRefiner.
//
//  Note that PatchTree is not publicly exposed. As with other Far classes
//  that are for internal use but not publicly exposed, it is not added
//  to the "internal" namespace.
//
//  The PatchTree was initially intended for internal use by other classes
//  to provide a simpler interface to the limit surface for the base faces
//  of a mesh.  It was kept in Far as previous use of other Far classes
//  required protected access, but that is no longer the case -- so it may
//  be moved elsewhere.
//
class PatchTree {
public:
    //  Constructors are protected
    ~PatchTree();

    //  Simple public accessors:
    int GetNumControlPoints() const  { return _numControlPoints; }
    int GetNumSubPatchPoints() const { return _numSubPatchPoints; }
    int GetNumPointsTotal() const    { return _numControlPoints +
                                              _numSubPatchPoints; }

    //  These queries may not be necessary...
    int GetDepth() const      { return _treeDepth; }
    int GetNumPatches() const { return (int)_patchParams.size(); }

    //  Methods to access stencils to compute patch points:
    template <typename REAL>
    REAL const * GetStencilMatrix() const;

    bool UsesDoublePrecision() const { return _useDoublePrecision; }

    //  Methods supporting evaluation:
    int HasSubFaces() const    { return _numSubFaces > 0; }
    int GetNumSubFaces() const { return _numSubFaces; }

    int FindSubPatch(double u, double v, int subFace=0, int maxDep=-1) const;

    PatchParam      GetSubPatchParam( int subPatch) const;
    ConstIndexArray GetSubPatchPoints(int subPatch) const;

    //  Main evaluation methods - basis weights or limit stencils:
    template <typename REAL>
    int EvalSubPatchBasis(int subPatch, REAL u, REAL v, REAL w[],
                          REAL wDu[],  REAL wDv[],
                          REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;

    template <typename REAL>
    int EvalSubPatchStencils(int subPatch, REAL u, REAL v, REAL s[],
                             REAL sDu[],  REAL sDv[],
                             REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

protected:
    PatchTree();
    friend class PatchTreeBuilder;

    //
    //  Internal utilities to support the stencil matrix of variable precision
    //
    template <typename REAL> std::vector<REAL> const & getStencilMatrix() const;
    template <typename REAL> std::vector<REAL>       & getStencilMatrix();

    template <typename REAL_MATRIX, typename REAL>
    int evalSubPatchStencils(int subPatch, REAL u, REAL v, REAL s[],
                             REAL sDu[],  REAL sDv[],
                             REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

protected:
    //  Internal quad-tree node type and assembly and search methods:
    struct TreeNode {
        struct Child {
            unsigned int isSet  :  1;
            unsigned int isLeaf :  1;
            unsigned int index  : 30;
        };

        TreeNode() : patchIndex(-1) {
            std::memset(children, 0, sizeof(children));
        }

        void SetChildren(int index);
        void SetChild(int quadrant, int index, bool isLeaf);

        int   patchIndex;
        Child children[4];
    };

    int searchQuadtree(double u, double v, int subFace=0, int depth=-1) const;
    void buildQuadtree();

    TreeNode * assignLeafOrChildNode(TreeNode * node,
                                     bool isLeaf, int quadrant, int index);

private:
    //  Private members:
    typedef PatchDescriptor::Type PatchType;

    //  Simple configuration members:
    unsigned int _useDoublePrecision    : 1;
    unsigned int _patchesIncludeNonLeaf : 1;
    unsigned int _patchesAreTriangular  : 1;

    PatchType _regPatchType;
    PatchType _irregPatchType;
    int       _regPatchSize;
    int       _irregPatchSize;
    int       _patchPointStride;

    //  Simple topology inventory members:
    int _numSubFaces;
    int _numControlPoints;
    int _numRefinedPoints;
    int _numSubPatchPoints;
    int _numIrregPatches;

    //  Vectors for points and PatchParams of all patches:
    //
    //  Note we store both regular and irregular patch point indices in the
    //  same vector (using a common stride for each patch) and the patch type
    //  determined by the PatchParam -- in the same way that face-varying
    //  patches are stored in the PatchTable.  Could also be stored in
    //  separate "patch arrays" or separated in other ways and manged with
    //  a bit more book-keeping.
    //
    std::vector<Index>      _patchPoints;
    std::vector<PatchParam> _patchParams;

    //  The quadtree organizing the patches:
    std::vector<TreeNode>  _treeNodes;
    int                    _treeDepth;

    //  Array of stencils for computing patch points from control points
    //  (single or double to be used as specified on construction):
    std::vector<float>  _stencilMatrixFloat;
    std::vector<double> _stencilMatrixDouble;
};

//
//  Internal specializations to support the stencil matrix of variable
//  precision -- const access presumes it is non-empty (and so assert)
//  while non-const access may be used to populate it.
//
template <>
inline std::vector<float> const &
PatchTree::getStencilMatrix<float>() const {
    assert(!_stencilMatrixFloat.empty());
    return _stencilMatrixFloat;
}
template <>
inline std::vector<double> const &
PatchTree::getStencilMatrix<double>() const {
    assert(!_stencilMatrixDouble.empty());
    return _stencilMatrixDouble;
}

template <>
inline std::vector<float> &
PatchTree::getStencilMatrix<float>() {
    return _stencilMatrixFloat;
}
template <>
inline std::vector<double> &
PatchTree::getStencilMatrix<double>() {
    return _stencilMatrixDouble;
}

//
//  Inline methods:
//
inline PatchParam
PatchTree::GetSubPatchParam(int subPatch) const {
    return _patchParams[subPatch];
}

inline int
PatchTree::FindSubPatch(double u, double v, int subFace, int maxDep) const {
    return searchQuadtree(u, v, subFace, maxDep);
}

template <typename REAL>
inline REAL const *
PatchTree::GetStencilMatrix() const {
    return &getStencilMatrix<REAL>()[0];
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_PATCH_TREE */
