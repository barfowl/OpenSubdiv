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

#ifndef OPENSUBDIV3_FAR_PATCH_TREE_H
#define OPENSUBDIV3_FAR_PATCH_TREE_H

#include "../version.h"

#include "../far/patchDescriptor.h"
#include "../far/patchParam.h"
#include "../far/topologyDescriptor.h"
#include "../far/stencilTable.h"

#include "../sdc/options.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  A PatchTree is a hierarchical collection of sub-patches associated with a
//  face and its particular local neighborhood that defines the limit surface.
//
class PatchTree {
public:
    //
    //  Place-holders for the topology Descriptor and Key are used to construct
    //  and cache instances of patch trees (these may be better of in
    //  PatchTreeFactory).
    //
    //  Use of Far::TopologyDescriptor here is only a shortcut to support more
    //  complete coverage of topology for testing.  A much simpler mechanism is
    //  needed for clients to describe the topological neighborhood around a
    //  single face that is efficient and unambigous in terms of control point
    //  ordering.  Details will all be replaced here.
    //
    //  Eventually consider including "hints" as members of the Descriptor that
    //  clients can optionally set to accelerate topological determination --
    //  particularly for the regular case -- as their meshes often have such
    //  information already available.
    //
    struct Descriptor {
        Sdc::SchemeType         schemeType;
        Sdc::Options            schemeOptions;
        Far::TopologyDescriptor topology;

        void FinalizeTopologyDescriptor(int nVertices, int nFaces);

        //  Data member vectors that the TopologyDescriptor will reference:
        struct Data {
            std::vector<int>   faceSizes;
            std::vector<Index> faceVerts;
            std::vector<Index> creaseIndices;
            std::vector<float> creaseSharpness;
            std::vector<Index> cornerIndices;
            std::vector<float> cornerSharpness;
            std::vector<Index> faceHoles;
        };
        Data data;
    };

    struct Key {
        Key() : hashBits(0) { }
        Key(Descriptor const & ) : hashBits(0) { }

        unsigned long hashBits;
    };

public:
    //  Constructors are protected
    ~PatchTree();

    //  Simple public accessors:
    int GetNumControlPoints() const { return _numControlPoints; }
    int GetNumSubPatchPoints() const{ return _numSubPatchPoints; }
    int GetNumPointsTotal() const   { return _numControlPoints + _numSubPatchPoints; }

    StencilTableReal<float> const * GetStencilTable() const { return _stencilTable; }

    //  Methods supporting evaluation:
    int FindSubPatch(float u, float v, int maxDepth = -1) const {
        return searchQuadtree(u, v, maxDepth);
    }

    ConstIndexArray GetSubPatchPoints(int subPatch) const;

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

    int  searchQuadtree(float u, float v, int depth = -1) const;
    void buildQuadtree();

    TreeNode * assignLeafOrChildNode(TreeNode * node, bool isLeaf, int quadrant, int index);

private:
    //  Private members:
    typedef PatchDescriptor::Type PatchType;

    //  Simple inventory and topology members:
    int       _numControlPoints;
    int       _numSubPatchPoints;
    bool      _patchesAreTriangular;
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
