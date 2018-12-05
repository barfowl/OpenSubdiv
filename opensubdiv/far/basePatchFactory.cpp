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

#include "../far/topologyRefiner.h"
#include "../far/topologyDescriptor.h"
#include "../far/basePatchFactory.h"
#include "../far/patchTreeFactory.h"
#include "../far/patchTreeCache.h"

#include <map>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Far {


namespace {
    //
    //  Build a PatchTree::Descriptor from a given base face of a TopologyRefiner:
    //
    void
    baseFaceToPatchTreeDescriptor(Far::TopologyRefiner const & baseRefiner, int baseFace,
                                  Far::PatchTree::Descriptor & patchTreeDescriptor,
                                  std::vector<int>           & baseControlPoints) {

        //
        //  NOTE -- this method of identifying the faces and vertices contributing
        //  to the patches for a face is general but ultimately unsuitable for our
        //  needs...
        //
        //  For common manifold cases we want a consistent vertex ordering and
        //  numbering that is unique to that topology (subject to rotations of the
        //  face).  The assembly here can result in many orderings for the vertices
        //  since the face-vertices are traversed in assigned order -- so rotations
        //  of all incident faces affect the ordering.  This method of gathering
        //  unique components in std::maps is also not very efficient compared to
        //  a manifold traversal.
        //
        //  We may end up using something like this for non-manifold cases, but 
        //  will want an alternative for manifold.  
        //
        typedef std::map<Index, Index> IndexMap;

        IndexMap faceMap;
        IndexMap vertMap;

        Far::TopologyLevel const & baseLevel = baseRefiner.GetLevel(0);

        std::vector<Index> baseFaces;
        baseControlPoints.resize(0);

        //  Identify the unique list of faces involved (base face first):
        faceMap[baseFace] = 0;
        baseFaces.push_back(baseFace);

        Far::ConstIndexArray baseFaceVerts = baseLevel.GetFaceVertices(baseFace);
        for (int i = 0; i < baseFaceVerts.size(); ++i) {
            Far::ConstIndexArray vFaces = baseLevel.GetVertexFaces(baseFaceVerts[i]);

            for (int j = 0; j < vFaces.size(); ++j) {
                Index fIndex = vFaces[j];

                if (faceMap.find(fIndex) == faceMap.end()) {
                    faceMap[fIndex] = (Index) faceMap.size();
                    baseFaces.push_back(fIndex);
                }
            }
        }

        //  Traverse all faces, identify unique vertices while appending face-verts:
        std::vector<Index> & faceVerts = patchTreeDescriptor.data.faceVerts;
        std::vector<int>   & faceSizes = patchTreeDescriptor.data.faceSizes;

        for (int i = 0; i < (int) baseFaces.size(); ++i) {
            Index                fIndex = baseFaces[i];
            Far::ConstIndexArray fVerts = baseLevel.GetFaceVertices(fIndex);

            for (int j = 0; j < fVerts.size(); ++j) {
                Index vIndex = fVerts[j];

                IndexMap::iterator vFound = vertMap.find(vIndex);
                if (vFound == vertMap.end()) {
                    Index vNew = (Index) vertMap.size();

                    vertMap[vIndex] = vNew;
                    faceVerts.push_back(vNew);
                    baseControlPoints.push_back(vIndex);
                } else {
                    faceVerts.push_back(vFound->second);
                }
            }
            faceSizes.push_back(fVerts.size());
        }

        //  Assign any sharp vertices:
        std::vector<float> & cornerSharpness = patchTreeDescriptor.data.cornerSharpness;
        std::vector<Index> & cornerIndices   = patchTreeDescriptor.data.cornerIndices;

        for (IndexMap::iterator vIt = vertMap.begin(); vIt != vertMap.end(); ++vIt) {
            Index vIndex = vIt->first;
            float vSharpness = baseLevel.GetVertexSharpness(vIndex);
            if (vSharpness > 0.0f) {
                cornerIndices.push_back(vIt->second);
                cornerSharpness.push_back(vSharpness);
            }
        }

        //  Assign any sharp edges (vertex pairs):
        std::vector<float> & creaseSharpness = patchTreeDescriptor.data.creaseSharpness;
        std::vector<Index> & creaseIndices   = patchTreeDescriptor.data.creaseIndices;

        for (int i = 0; i < baseFaceVerts.size(); ++i) {
            Far::ConstIndexArray vEdges = baseLevel.GetVertexEdges(baseFaceVerts[i]);

            for (int j = 0; j < vEdges.size(); ++j) {
                Index eIndex = vEdges[j];
                float eSharpness = baseLevel.GetEdgeSharpness(eIndex);
                if ((eSharpness > 0.0f) && baseLevel.GetEdgeFaces(eIndex).size()) {
                    Far::ConstIndexArray eVerts = baseLevel.GetEdgeVertices(eIndex);

                    creaseIndices.push_back(vertMap.find(eVerts[0])->second);
                    creaseIndices.push_back(vertMap.find(eVerts[1])->second);
                    creaseSharpness.push_back(eSharpness);
                }
            }
        }

        //  Assign the PatchTree::Descriptor fields:
        int nVerts = (int) vertMap.size();
        int nFaces = (int) faceMap.size();

        patchTreeDescriptor.FinalizeTopologyDescriptor(nVerts, nFaces);

        patchTreeDescriptor.schemeType    = baseRefiner.GetSchemeType();
        patchTreeDescriptor.schemeOptions = baseRefiner.GetSchemeOptions();
    }
} // end namespace


//
//  The public creation method:
//
BasePatch *
BasePatchFactory::Create(TopologyRefiner const & meshTopology,
                         Index                   baseFace,
                         Options                 options) {

    BasePatch * basePatch = new BasePatch();

    PatchTree::Descriptor topologyDescriptor;
    baseFaceToPatchTreeDescriptor(meshTopology, baseFace, topologyDescriptor,
                                  basePatch->_controlPoints);
    PatchTree::Key topologyKey(topologyDescriptor);

    //  We are going to need a better mapping between base level control
    //  points and those in the PatchTree, as any PatchTree retrieved from
    //  the cache may involve a permutation and/or subset of the control
    //  points for the full topology descriptor.  Additional information
    //  within or associated with the PatchTree seems warranted.

    if (options.patchTreeCache) {
        basePatch->_patchTree = options.patchTreeCache->Find(topologyKey);
        basePatch->_patchTreeIsExternal = true;
    }

    if (basePatch->_patchTree == 0) {
        basePatch->_patchTree = PatchTreeFactory::Create(topologyDescriptor,
                        PatchTreeFactory::Options(options.maxPatchDepth));
        basePatch->_patchTreeIsExternal = false;

        if (options.updatePatchTreeCache) {
            options.patchTreeCache->Add(topologyKey, basePatch->_patchTree);
            basePatch->_patchTreeIsExternal = true;
        }
    }
    return basePatch;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
