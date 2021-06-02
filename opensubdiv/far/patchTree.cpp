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

#include "../far/patchTree.h"
#include "../far/patchBasis.h"

#include <algorithm>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  Simple inline methods for the PatchTree::TreeNode:
//
// sets all the children to point to the patch of given index
inline void
PatchTree::TreeNode::SetChildren(int index) {

    for (int i=0; i<4; ++i) {
        children[i].isSet  = true;
        children[i].isLeaf = true;
        children[i].index  = index;
    }
}

// sets the child in "quadrant" to point to the node or patch of the given index
inline void
PatchTree::TreeNode::SetChild(int quadrant, int index, bool isLeaf) {

    assert(!children[quadrant].isSet);
    children[quadrant].isSet  = true;
    children[quadrant].isLeaf = isLeaf;
    children[quadrant].index  = index;
}


//
//  PatchTree constructor and destructor:
//
PatchTree::PatchTree() :
    _numSubFaces(0),
    _numControlPoints(0),
    _numSubPatchPoints(0),
    _patchesAreTriangular(false),
    _patchesIncludeNonLeaf(false),
    _regPatchType(PatchDescriptor::NON_PATCH),
    _regPatchSize(0),
    _irregPatchType(PatchDescriptor::NON_PATCH),
    _irregPatchSize(0),
    _patchPointStride(0),
    _treeDepth(-1),
    _stencilTable(0) {
}

PatchTree::~PatchTree() {
    delete _stencilTable;
}


//
//  Class methods supporing access to patches:
//
ConstIndexArray
PatchTree::GetSubPatchPoints(int patchIndex) const {

    return ConstIndexArray(
            &_patchPoints[patchIndex * _patchPointStride],
            _patchParams[patchIndex].IsRegular() ? _regPatchSize : _irregPatchSize);
}

int
PatchTree::EvalSubPatchBasis(int patchIndex, float u, float v,
                             float wP[], float wDu[], float wDv[]) const {

    PatchParam const & param = _patchParams[patchIndex];

    return internal::EvaluatePatchBasis(
            param.IsRegular() ? _regPatchType : _irregPatchType,
            param, u, v, wP, wDu, wDv);
}

//
//  Local functions and class methods supporting tree searches and construction:
//
namespace {
    template <typename T>
    inline int
    transformUVToQuadQuadrant(T const & median, T & u, T & v) {

        int uHalf = (u >= median);
        if (uHalf) u -= median;

        int vHalf = (v >= median);
        if (vHalf) v -= median;

        return (vHalf << 1) | uHalf;
    }

    template <typename T>
    int inline
    transformUVToTriQuadrant(T const & median, T & u, T & v, bool & rotated) {

        if (!rotated) {
            if (u >= median) {
                u -= median;
                return 1;
            }
            if (v >= median) {
                v -= median;
                return 2;
            }
            if ((u + v) >= median) {
                rotated = true;
                return 3;
            }
            return 0;
        } else {
            if (u < median) {
                v -= median;
                return 1;
            }
            if (v < median) {
                u -= median;
                return 2;
            }
            u -= median;
            v -= median;
            if ((u + v) < median) {
                rotated = true;
                return 3;
            }
            return 0;
        }
    }
} // end namespace

inline PatchTree::TreeNode *
PatchTree::assignLeafOrChildNode(TreeNode * node, bool isLeaf, int quadrant, int patchIndex) {

    //  This is getting far enough away from Far::PatchMap's original
    //  structure and implementation that it warrants a face lift...

    if (!node->children[quadrant].isSet) {
        if (isLeaf) {
            node->SetChild(quadrant, patchIndex, true);
            return node;
        } else {
            int newNodeIndex = (int)_treeNodes.size();
            _treeNodes.push_back(TreeNode());
            node->SetChild(quadrant, newNodeIndex, false);
            return &_treeNodes[newNodeIndex];
        }
    }

    if (isLeaf || node->children[quadrant].isLeaf) {
        //  Need to replace the leaf index with new node and index:
        int newNodeIndex = (int)_treeNodes.size();
        _treeNodes.push_back(TreeNode());
        TreeNode * newNode = &_treeNodes[newNodeIndex];

        //  Move existing patch index from child to new child node:
        newNode->patchIndex = node->children[quadrant].index;

        node->children[quadrant].index  = newNodeIndex;
        node->children[quadrant].isLeaf = false;
        if (isLeaf) {
            newNode->SetChild(quadrant, patchIndex, true);
        }
        return newNode;
    } else {
        //  Simply return the existing interior node:
        return &_treeNodes[node->children[quadrant].index];
    }
}

void
PatchTree::buildQuadtree() {

    int numPatches = (int) _patchParams.size();

//printf("\nPatchTree::buildQuadtree():\n");

    _treeNodes.reserve(numPatches);
    _treeNodes.resize(_numSubFaces ? _numSubFaces : 1);
    _treeDepth = 0;

//printf("Traversing %d patches...\n", numPatches);
    for (int patchIndex = 0; patchIndex < numPatches; ++patchIndex) {

        PatchParam const & param = _patchParams[patchIndex];

        int depth     = param.GetDepth();
        int rootDepth = param.NonQuadRoot();
        int subFace   = param.GetFaceId();

        TreeNode * node = &_treeNodes[subFace];

        _treeDepth = std::max(depth, _treeDepth);

//printf("    patch %2d:  depth = %d, faceId = %d\n", patchIndex, depth, subFace);

        if (depth == rootDepth) {
            node->patchIndex = patchIndex;
//printf("        assigning to node[%d]\n", subFace);
            continue;
        }
            
        if (!_patchesAreTriangular) {
            //  Use the UV bits of the PatchParam directly for quad patches:
            int u = param.GetU();
            int v = param.GetV();

            for (int j = rootDepth + 1; j <= depth; ++j) {
                int uBit = (u >> (depth - j)) & 1;
                int vBit = (v >> (depth - j)) & 1;

                int quadrant = (vBit << 1) | uBit;

                node = assignLeafOrChildNode(node, (j == depth), quadrant, patchIndex);
            }
        } else {
            //  Use an interior UV point of triangles to identify quadrants:
            float u = 0.25f;
            float v = 0.25f;
            param.UnnormalizeTriangle(u, v);

            float median = 0.5f;
            bool triRotated = false;

            for (int j = rootDepth + 1; j <= depth; ++j, median *= 0.5f) {
                int quadrant = transformUVToTriQuadrant(median, u, v, triRotated);

                node = assignLeafOrChildNode(node, (j == depth), quadrant, patchIndex);
            }
        }
    }
/*
printf("Resulting %d tree nodes...\n", (int)_treeNodes.size());
for (int i = 0; i < (int)_treeNodes.size(); ++i) {
    TreeNode const & node = _treeNodes[i];
    printf("    node %d:  patch index = %d\n", i, node.patchIndex);
    for (int j = 0; j < 4; ++j) {
        TreeNode::Child const & child = node.children[j];
        printf("        child %d:  leaf = %d, set = %d, %s = %d\n", j,
            child.isLeaf, child.isSet,
            child.isLeaf ? "patch" : "node ",
            child.isSet ? child.index : -1);
    }
}
*/
}

int
PatchTree::searchQuadtree(float u, float v, int subFace, int searchDepth) const {

    //
    //  These details warrant closer inspection and possible tweaking
    //  since non-leaf patches were optionally added to the tree...
    //

    //
    //  Identify the root patch and make a quick exit when seeking it.  If
    //  there is no patch at level 0 but subpatches present (possible e.g.
    //  if adjacent to an irregular face) force the search to level 1:
    //
    TreeNode const * node = &_treeNodes[subFace];

    if (_treeDepth == 0) {
        assert(node->patchIndex >= 0);
        return node->patchIndex;
    }

    int maxDepth = ((searchDepth >= 0) && _patchesIncludeNonLeaf) ? searchDepth : _treeDepth;
    if (maxDepth == (_numSubFaces > 0)) {
        if (node->patchIndex >= 0) {
            return node->patchIndex;
        }
        maxDepth = 1;
    }

    //
    //  Search the tree for the sub-patch containing the given (u,v)
    //
    float median = 0.5f;
    bool triRotated = false;

    for (int depth = 1; depth <= maxDepth; ++depth, median *= 0.5f) {

        int quadrant = _patchesAreTriangular
                     ? transformUVToTriQuadrant(median, u, v, triRotated)
                     : transformUVToQuadQuadrant(median, u, v);

        //  Identify child patch if leaf, otherwise child node:
        if (node->children[quadrant].isLeaf) {
            return node->children[quadrant].index;
        } else if (node->children[quadrant].isSet) {
            node = &_treeNodes[node->children[quadrant].index];
        }
    }
    assert(node->patchIndex >= 0);
    return node->patchIndex;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
