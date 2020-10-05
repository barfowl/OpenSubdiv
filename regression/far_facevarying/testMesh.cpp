//
//   Copyright 2016 Dreamworks
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
#include <set>
#include <cassert>
#include <cstdio>

#include <opensubdiv/far/topologyDescriptor.h>
#include <opensubdiv/far/topologyRefiner.h>
#include <opensubdiv/far/topologyRefinerFactory.h>
#include <opensubdiv/far/topologyLevel.h>
#include <opensubdiv/far/patchTable.h>
#include <opensubdiv/far/patchTableFactory.h>
#include <opensubdiv/far/patchMap.h>
#include <opensubdiv/far/ptexIndices.h>

#include "../../regression/common/far_utils.h"

#include "testMesh.h"

using namespace OpenSubdiv;
using namespace OpenSubdiv::OPENSUBDIV_VERSION;

using Far::Index;
using Far::LocalIndex;
using Far::ConstIndexArray;
using Far::ConstLocalIndexArray;


//
//  TopologyContainer methods:
//
TopologyContainer::TopologyContainer(TopologyContainer const & source) :
    vertexCount(source.vertexCount),
    faceCount(source.faceCount),
    fvarValueCount(source.fvarValueCount),
    faceVertexCounts(source.faceVertexCounts),
    faceVertexOffsets(source.faceVertexOffsets),
    faceVertexIndices(source.faceVertexIndices),
    creaseVertexPairs(source.creaseVertexPairs),
    creaseWeights(source.creaseWeights),
    cornerVertexIndices(source.cornerVertexIndices),
    cornerWeights(source.cornerWeights),
    holeFaceIndices(source.holeFaceIndices),
    fvarValueIndices(source.fvarValueIndices) {

}

void
TopologyContainer::GetTopologyDescriptor(Far::TopologyDescriptor & descriptor,
        Far::TopologyDescriptor::FVarChannel & descFVarChannel) const {

    TopologyContainer const & container = *this;

    std::memset(&descriptor, 0, sizeof(Far::TopologyDescriptor));

    descriptor.numVertices = container.vertexCount;

    descriptor.numFaces = container.faceCount;
    if (descriptor.numFaces) {
        descriptor.numVertsPerFace    = &container.faceVertexCounts[0];
        descriptor.vertIndicesPerFace = &container.faceVertexIndices[0];
    }

    descriptor.numCreases = (int)container.creaseWeights.size();
    if (descriptor.numCreases) {
        descriptor.creaseVertexIndexPairs = &container.creaseVertexPairs[0];
        descriptor.creaseWeights          = &container.creaseWeights[0];
    }

    descriptor.numCorners = (int)container.cornerWeights.size();
    if (descriptor.numCorners) {
        descriptor.cornerVertexIndices = &container.cornerVertexIndices[0];
        descriptor.cornerWeights       = &container.cornerWeights[0];
    }

    descriptor.numHoles = (int)container.holeFaceIndices.size();
    if (descriptor.numHoles) {
        descriptor.holeIndices = &container.holeFaceIndices[0];
    }

    if (container.fvarValueCount) {
        descriptor.numFVarChannels = 1;
        descriptor.fvarChannels    = &descFVarChannel;

        descFVarChannel.numValues    = container.fvarValueCount;
        descFVarChannel.valueIndices = &container.fvarValueIndices[0];
    }
}


//
//  TestMesh methods:
//
TestMesh::TestMesh() :
    _preUvChannels(0), _postUvChannels(0), _topRefiner(0), _patchTable(0) {

}

TestMesh::TestMesh(TestMesh const & source) :
    _subdivType(source._subdivType),
    _subdivOptions(source._subdivOptions),
    _preUvChannels(source._preUvChannels),
    _postUvChannels(source._postUvChannels),
    _topContainer(source._topContainer),
    _topRefiner(0),
    _patchTable(0),
    _xyzCoords(source._xyzCoords),
    _uvwCoords(source._uvwCoords) {
}

TestMesh::~TestMesh() {
    delete _topRefiner;
    delete _patchTable;
}

bool
TestMesh::BuildFromShape(Shape const & shape) {

    _subdivType = GetSdcType(shape);
    _subdivOptions = GetSdcOptions(shape);

    _topRefiner = Far::TopologyRefinerFactory<Shape>::Create(shape,
            Far::TopologyRefinerFactory<Shape>::Options(_subdivType, _subdivOptions));

    initTopologyContainerFromRefiner();

    if (_topContainer.vertexCount > 0) {
        assert((int)shape.verts.size() == 3 * _topContainer.vertexCount);
        _xyzCoords.resize(_topContainer.vertexCount);
        std::memcpy(&_xyzCoords[0], &shape.verts[0], _xyzCoords.size() * sizeof(Coord3));
    }
    if (_topContainer.fvarValueCount > 0) {
        assert((int)shape.uvs.size() == 2 * _topContainer.fvarValueCount);
        _uvwCoords.resize(_topContainer.fvarValueCount);
        for (int i = 0; i < _topContainer.fvarValueCount; ++i) {
            _uvwCoords[i].x = shape.uvs[2*i + 0];
            _uvwCoords[i].y = shape.uvs[2*i + 1];
            _uvwCoords[i].z = 0.0f;
        }
    }
    return true;
}

void
TestMesh::initTopologyContainerFromRefiner()
{
    TopologyLevel const & baseLevel = _topRefiner->GetLevel(0);
    TopologyContainer &   container = _topContainer;

    int nVerts  = baseLevel.GetNumVertices();
    int nFaces  = baseLevel.GetNumFaces();
    int nEdges  = baseLevel.GetNumEdges();
    int nFVerts = baseLevel.GetNumFaceVertices();

    container.vertexCount = nVerts;
    container.faceCount   = nFaces;

    //  Identify base level face-verts
    container.faceVertexCounts.resize(nFaces);
    container.faceVertexOffsets.resize(nFaces);
    container.faceVertexIndices.reserve(nFVerts);

    for (int i = 0; i < nFaces; ++i) {
        ConstIndexArray faceVerts = baseLevel.GetFaceVertices(i);

        container.faceVertexCounts[i] = faceVerts.size();
        for (int j = 0; j < faceVerts.size(); ++j) {
            container.faceVertexIndices.push_back(faceVerts[j]);
        }
        container.faceVertexOffsets[i] = (i == 0) ? 0 :
                (container.faceVertexOffsets[i-1] + container.faceVertexCounts[i-1]);
    }

    //  Identify base level edge creases
    container.creaseVertexPairs.resize(0);
    container.creaseWeights.resize(0);

    for (int i = 0; i < nEdges; ++i) {
        float edgeSharpness = baseLevel.GetEdgeSharpness(i);
        if (edgeSharpness > 0.0f) {
            ConstIndexArray edgeVerts = baseLevel.GetEdgeVertices(i);

            container.creaseVertexPairs.push_back(edgeVerts[0]);
            container.creaseVertexPairs.push_back(edgeVerts[1]);
            container.creaseWeights.push_back(edgeSharpness);
        }
    }

    //  Identify base level vertex corners
    container.cornerVertexIndices.resize(0);
    container.cornerWeights.resize(0);

    for (int i = 0; i < nVerts; ++i) {
        float vertSharpness = baseLevel.GetVertexSharpness(i);
        if (vertSharpness > 0.0f) {
            container.cornerVertexIndices.push_back(i);
            container.cornerWeights.push_back(vertSharpness);
        }
    }

    //  Identify base level face holes
    container.holeFaceIndices.resize(0);

    for (int i = 0; i < nFaces; ++i) {
        if (baseLevel.IsFaceHole(i)) {
            container.holeFaceIndices.push_back(i);
        }
    }

    //  Identify base level face-varying values (first channel only)
    container.fvarValueCount = 0;
    container.fvarValueIndices.resize(0);

    if (baseLevel.GetNumFVarChannels() > 0) {
        container.fvarValueCount = baseLevel.GetNumFVarValues(0);

        container.fvarValueIndices.reserve(nFVerts);
        for (int i = 0; i < nFaces; ++i) {
            ConstIndexArray faceValues = baseLevel.GetFaceFVarValues(i, 0);

            for (int j = 0; j < faceValues.size(); ++j) {
                container.fvarValueIndices.push_back(faceValues[j]);
            }
        }
    }
}

bool
TestMesh::BuildFromUvChannel(TestMesh const & srcMesh) {

    TopologyContainer const & srcContainer = srcMesh._topContainer;
    TopologyLevel const &     srcBaseLevel = srcMesh.GetBaseLevel();
    int                       srcUvChannel = srcMesh.GetUvChannel();

    //  Aside from the face counts and per-face vertex counts, the face-vertex
    //  topology comes from the FVar channel:
    //
    _subdivType    = srcMesh._subdivType;
    _subdivOptions = srcMesh._subdivOptions;

    _topContainer.faceCount   = srcContainer.faceCount;
    _topContainer.vertexCount = srcContainer.fvarValueCount;

    _topContainer.faceVertexCounts  = srcContainer.faceVertexCounts;
    _topContainer.faceVertexOffsets = srcContainer.faceVertexOffsets;
    _topContainer.faceVertexIndices = srcContainer.fvarValueIndices;

    _xyzCoords = srcMesh._uvwCoords;
    _uvwCoords.resize(0);

    //  Creases and corners need to be transferred from the topology of the source,
    //  and since we don't have correllation between vertex and FVar values in the
    //  source, we inspect face-edges and face-vertices to detect sharp features
    //  and apply them using the corresponding FVar values (now vertices)
    //
    _topContainer.creaseVertexPairs.resize(0);
    _topContainer.creaseWeights.resize(0);

    _topContainer.cornerVertexIndices.resize(0);
    _topContainer.cornerWeights.resize(0);

    for (int i = 0; i < srcContainer.faceCount; ++i) {
        ConstIndexArray fEdges = srcBaseLevel.GetFaceEdges(i);
        ConstIndexArray fVerts = srcBaseLevel.GetFaceVertices(i);
        ConstIndexArray fValues = srcBaseLevel.GetFaceFVarValues(i, srcUvChannel);

        for (int j = 0; j < fVerts.size(); ++j) {
            Index e = fEdges[j];
            float eSharpness = srcBaseLevel.GetEdgeSharpness(e);
            if (eSharpness > 0.0f) {
                _topContainer.creaseVertexPairs.push_back(fValues[j]);
                _topContainer.creaseVertexPairs.push_back(fValues[(j+1)%fVerts.size()]);
                _topContainer.creaseWeights.push_back(eSharpness);
            }

            Index v = fVerts[j];
            float vSharpness = srcBaseLevel.GetVertexSharpness(v);
            if (vSharpness > 0.0f) {
                _topContainer.cornerVertexIndices.push_back(fValues[j]);
                _topContainer.cornerWeights.push_back(vSharpness);
            }
        }
    }

    //  Holes remain the same as the face list is unchanged
    _topContainer.holeFaceIndices = srcContainer.holeFaceIndices;

    //  The FVar channel remains empty
    _topContainer.fvarValueCount = 0;
    _topContainer.fvarValueIndices.resize(0);

    return true;
}

int
countFVarValuesAtVertex(Far::TopologyLevel const & baseLevel, Index vIndex, int channel) {

    ConstIndexArray      vFaces  = baseLevel.GetVertexFaces(vIndex);
    ConstLocalIndexArray vInFace = baseLevel.GetVertexFaceLocalIndices(vIndex);

    std::set<Index> fvarSet;
    for (int i = 0; i < vFaces.size(); ++i) {
        Index fvarValue = baseLevel.GetFaceFVarValues(vFaces[i], channel)[vInFace[i]];

        if (fvarSet.find(fvarValue) == fvarSet.end()) {
            fvarSet.insert(fvarValue);
        }
    }
    return (int)fvarSet.size();
}

void
TestMesh::SharpenFromUvChannel(TestMesh const & srcMesh, FVarOption option) {

    //
    //  Given a mesh whose UVs correspond to the vertices of this one, sharpen the
    //  vertices of this mesh according to the local topology of the UV value in
    //  the original source mesh, and according to the given face-varying linear
    //  interpolation option.
    //
    //  This sharpening will involve setting the vertex boundary interpolation to
    //  match that of the FVar option at its simplest, and will have to inspect the
    //  UV topology around the original source vertices in some cases.
    //
    //  Note that it is assumed that all sharpness inherent in the original source
    //  mesh has already applied, i.e. sharpness that is not associated with face-
    //  varying boundaries and should be equally reflected in the UV mesh.
    //
    TopologyContainer const & srcContainer = srcMesh._topContainer;
    TopologyLevel const &     srcBaseLevel = srcMesh.GetBaseLevel();
    int                       srcUvChannel = srcMesh.GetUvChannel();

    std::vector<float> srcVertexSharpness(srcContainer.vertexCount, 0.0f);
    bool               srcVerticesSharpened = false;

    switch (option) {
    case Sdc::Options::FVAR_LINEAR_NONE:
        _subdivOptions.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_ONLY);
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_ONLY:
        _subdivOptions.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);
        break;

    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS1:
    case Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2:
        _subdivOptions.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

        for (int i = 0; i < srcContainer.vertexCount; ++i) {
            if (srcBaseLevel.DoesVertexFVarTopologyMatch(i, srcUvChannel)) continue;

            int fvarValuesAtVertex = countFVarValuesAtVertex(srcBaseLevel, i, srcUvChannel);
            if (fvarValuesAtVertex > 2) {
                //  For both PLUS1 and PLUS2, the presence of 3 or more fvar values at a
                //  vertex forces all to be sharp.
                //
                srcVertexSharpness[i] = Sdc::Crease::SHARPNESS_INFINITE;
                srcVerticesSharpened = true;
            } else if (fvarValuesAtVertex == 1) {
                //  The PLUS2 option sharpens darts.  Note that only one fvar value for a
                //  vertex can also indicate a mismatched boundary as well as a dart, so
                //  ignore the boundary cases (handled by the vtx boundary interp option)
                //
                if (option == Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2) {
                    if (!srcBaseLevel.IsVertexBoundary(i)) {
                        srcVertexSharpness[i] = Sdc::Crease::SHARPNESS_INFINITE;
                        srcVerticesSharpened = true;
                    }
                }
            } else if (fvarValuesAtVertex == 2) {
                //  This is the messiest case -- both the PLUS1 and PLUS2 options have to
                //  deal with a case internally referred to as "dependent sharpness" which
                //  occurs when there are two FVar disjoint regions around a vertex.  If one
                //  is sharp, i.e. a corner rather than a crease (due to an interior sharp
                //  edge), the other must also be kept sharp for the same duration.  We
                //  accomplish this by sharpening the vertex to the same sharpness of the
                //  incident edges that make it a corner.  This then affects both values.
                //
                //  Remember that this is often triggered by semi-sharp cases and so the
                //  sharpness that should be assigned is the local maximum and not full
                //  infinite sharpness.
                //
                ConstIndexArray      vFaces  = srcBaseLevel.GetVertexFaces(i);
                ConstLocalIndexArray vInFace = srcBaseLevel.GetVertexFaceLocalIndices(i);

                float maxSharpness = srcBaseLevel.GetVertexSharpness(i);
                ConstIndexArray vEdges = srcBaseLevel.GetVertexEdges(i);
                for (int j = 0; j < vEdges.size(); ++j) {
                    float eSharpness = srcBaseLevel.GetEdgeSharpness(vEdges[j]);
                    if (eSharpness > maxSharpness) {
                        if (!srcBaseLevel.IsEdgeBoundary(vEdges[j]) &&
                            !srcBaseLevel.IsEdgeNonManifold(vEdges[j]) &&
                            srcBaseLevel.DoesEdgeFVarTopologyMatch(vEdges[j], srcUvChannel)) {
                            maxSharpness = eSharpness;
                            srcVertexSharpness[i] = maxSharpness;
                            srcVerticesSharpened = true;
                        }
                    }
                }

                //  The PLUS2 option includes a feature formerly known as "propagate corners"
                //  which sharpens the other value if one is a corner.  So simply count the
                //  number of occurrences of each and sharpen both if either is a corner.
                //
                if (option == Sdc::Options::FVAR_LINEAR_CORNERS_PLUS2) {
                    int index1 = -1;
                    int index2 = -1;;
                    int count1 = 1;
                    int count2 = 1;
                    for (int j = 0; j < vFaces.size(); ++j) {
                        int   fvarOffset = srcContainer.faceVertexOffsets[vFaces[j]] + vInFace[j];
                        Index fvarValue  = srcContainer.fvarValueIndices[fvarOffset];

                        if (index1 < 0) {
                            index1 = fvarValue;
                        } else if (fvarValue == index1) {
                            count1 ++;
                        } else if (index2 < 0) {
                            index2 = fvarValue;
                        } else {
                            count2 ++;
                        }
                    }
                    if ((count1 == 1) || (count2 == 1)) {
                        srcVertexSharpness[i] = Sdc::Crease::SHARPNESS_INFINITE;
                        srcVerticesSharpened = true;
                    }
                }
            }
        }
        srcVerticesSharpened = true;
        break;

    case Sdc::Options::FVAR_LINEAR_BOUNDARIES:
        _subdivOptions.SetVtxBoundaryInterpolation(Sdc::Options::VTX_BOUNDARY_EDGE_AND_CORNER);

        for (int i = 0; i < srcContainer.vertexCount; ++i) {
            if (!srcBaseLevel.DoesVertexFVarTopologyMatch(i, srcUvChannel)) {
                srcVertexSharpness[i] = Sdc::Crease::SHARPNESS_INFINITE;
                srcVerticesSharpened = true;
            }
        }
        break;

    case Sdc::Options::FVAR_LINEAR_ALL:
        //  Emulating LINEAR_ALL in a separate UV mesh should be done by changing the
        //  subdivision Scheme to Bilinear.  Unfortunately current limitations in both
        //  adaptive refinement and patch construction (exclusively for scheme Catmark)
        //  make that awkward...
        //
        assert(option != Sdc::Options::FVAR_LINEAR_ALL);
        break;
    default:
        break;
    }

    //  FVar values at non-manifold vertices are always sharpened when non-linear:
    //
    //  Note this does sharpen non-manifold vertices on non-manifold crease edges,
    //  which is NOT done for vertices, but needs to be done here as FVar values
    //  at non-manifold vertices are all effectively tagged as inf-sharp corners.
    //
    if (option != Sdc::Options::FVAR_LINEAR_ALL) {
        for (int i = 0; i < srcContainer.vertexCount; ++i) {
            if (srcBaseLevel.IsVertexNonManifold(i)) {
                srcVertexSharpness[i] = Sdc::Crease::SHARPNESS_INFINITE;
                srcVerticesSharpened = true;
            }
        }
    }

    //  For each src vertex that requires sharpening of its associated FVar values,
    //  append vertex corner weights to the TopologyContainer for the new vertices
    //  corresponding to each of the original vertex' FVar values:
    //
    if (srcVerticesSharpened) {
        for (int i = 0; i < srcContainer.vertexCount; ++i) {
            if (srcVertexSharpness[i] > 0.0f) {
                ConstIndexArray      vFaces  = srcBaseLevel.GetVertexFaces(i);
                ConstLocalIndexArray vInFace = srcBaseLevel.GetVertexFaceLocalIndices(i);

                for (int j = 0; j < vFaces.size(); ++j) {
                    int   fvarOffset = srcContainer.faceVertexOffsets[vFaces[j]] + vInFace[j];
                    Index fvarValue  = srcContainer.fvarValueIndices[fvarOffset];

                    _topContainer.cornerVertexIndices.push_back(fvarValue);
                    _topContainer.cornerWeights.push_back(srcVertexSharpness[i]);
                }
            }
        }
    }
}

void
TestMesh::AssignEmptyChannels(int preUvChannels, int postUvChannels) {

    _preUvChannels  = preUvChannels;
    _postUvChannels = postUvChannels;
}

void
TestMesh::ClearEmptyChannels() {

    _preUvChannels = 0;
    _postUvChannels = 0;

    Rebuild(0, 0, 0);
}

void
TestMesh::Rebuild(TopologyRefiner::AdaptiveOptions const * adaptiveOptions,
                  TopologyRefiner::UniformOptions const *  uniformOptions,
                  PatchTableOptions const *                patchOptions) {

    //
    //  Initialize the TopologyDescriptor to construct the TopologyRefiner, adding
    //  any "empty" FVar channels before the UV channel when specified:
    //
    typedef Far::TopologyDescriptor::FVarChannel FVarChannel;

    TopologyDescriptor       descriptor;
    std::vector<int>         descFVarIndices;
    std::vector<FVarChannel> descFVarChannels(1);

    _topContainer.GetTopologyDescriptor(descriptor, descFVarChannels[0]);

    if (HasUvs() && (_preUvChannels || _postUvChannels)) {
        //
        //  Load the "channel" with either per-vertex FVar values or a unique set of
        //  FVar values per face -- the former will not affect adaptive refinement
        //  of the collective set of FVar channels, but the latter will (particularly
        //  LINEAR_NONE when all FVar corners will be smoothed).
        //
        FVarChannel emptyChannel;

        bool fvarPerVertex = false;
        if (fvarPerVertex) {
            emptyChannel.numValues = _topContainer.vertexCount;

            descFVarIndices = _topContainer.faceVertexIndices;
        } else {
            emptyChannel.numValues = _topContainer.faceVertexIndices.size();

            descFVarIndices.resize(emptyChannel.numValues);
            for (int i = 0; i < emptyChannel.numValues; ++i) {
                descFVarIndices[i] = i;
            }
        }
        emptyChannel.valueIndices = (Index*) &descFVarIndices[0];

        int numFVarChannels = 1 + _preUvChannels + _postUvChannels;
        descFVarChannels.resize(numFVarChannels, emptyChannel);
        if (_preUvChannels) {
            std::swap(descFVarChannels[0], descFVarChannels[_preUvChannels]);
        }

        descriptor.numFVarChannels = numFVarChannels;
        descriptor.fvarChannels    = &descFVarChannels[0];
    }

    //
    //  Destroy pre-existing TopologyRefiner or PatchTable before constructing anew:
    //
    delete _topRefiner, _topRefiner = 0;
    delete _patchTable, _patchTable = 0;

    _topRefiner = Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Create(descriptor,
                            TopologyRefinerOptions(_subdivType, _subdivOptions));
    assert(_topRefiner);

    if (adaptiveOptions || uniformOptions) {
        if (adaptiveOptions) {
            _topRefiner->RefineAdaptive(*adaptiveOptions);
        } else {
            _topRefiner->RefineUniform(*uniformOptions);
        }
        if (patchOptions) {
            _patchTable = PatchTableFactory::Create(*_topRefiner, *patchOptions);
            assert(_patchTable);
        }
        interpolateVertexData();
        interpolateFaceVaryingData();
    }
}

void
TestMesh::AssignUvChannelInterpolation(FVarOption option) {

    _subdivOptions.SetFVarLinearInterpolation(option);
}

void
TestMesh::AssignUvChannelPerVertex() {

    //
    //  Assign the face-varying channel a value per vertex and copy the UV values
    //  from vertex positions.
    //
    //  Remember to set the FVar interpolation mode to match the vertex boundary
    //  interpolation mode.
    //
    _topContainer.fvarValueCount   = _topContainer.vertexCount;
    _topContainer.fvarValueIndices = _topContainer.faceVertexIndices;

    _uvwCoords = _xyzCoords;

    if (_subdivOptions.GetVtxBoundaryInterpolation() == Sdc::Options::VTX_BOUNDARY_EDGE_ONLY) {
        _subdivOptions.SetFVarLinearInterpolation(Sdc::Options::FVAR_LINEAR_NONE);
    } else {
        _subdivOptions.SetFVarLinearInterpolation(Sdc::Options::FVAR_LINEAR_CORNERS_ONLY);
    }
}

void
TestMesh::ExpandUvChannel() {

    //
    //  This process "expands" the number of FVar values so that the FVar values
    //  for each vertex are unique to that vertex and not shared by other vertices.
    //  This separation of shared FVar values across vertices gives us the desired
    //  partitioning of the vertex topology for which FVar values are intended,
    //  and allows us to construct a separate mesh from the resulting FVar topology
    //  that is reflective of the way it is interpreted as a FVar channel.
    //
    if (_topRefiner == 0) Rebuild(0, 0, 0);

    TopologyLevel const & baseLevel = _topRefiner->GetLevel(0);

    //  Initialize references and arrays for the old and new FVar values:
    std::vector<int> const & fvarOffsetsPerFace = _topContainer.faceVertexOffsets;

    std::vector<int> const & oldFVarValueIndices = _topContainer.fvarValueIndices;
    Coord3Vector     const & oldFVarCoords       = _uvwCoords;

    std::vector<int> newFVarValueIndices;
    Coord3Vector     newFVarCoords;

    newFVarValueIndices.resize(oldFVarValueIndices.size());
    newFVarCoords.reserve(2 * _topContainer.vertexCount);

    typedef std::map<int,int> FVarIndexMap;
    FVarIndexMap localFVarMap;

    for (int vIndex = 0; vIndex < _topContainer.vertexCount; ++vIndex) {
        ConstIndexArray      vFaces  = baseLevel.GetVertexFaces(vIndex);
        ConstLocalIndexArray vInFace = baseLevel.GetVertexFaceLocalIndices(vIndex);

        localFVarMap.clear();
        for (int i = 0; i < vFaces.size(); ++i) {
            //  Use a direct offset to the FVar value as we need it to assign later:
            int fvarOffset = fvarOffsetsPerFace[vFaces[i]] + vInFace[i];

            Index oldFVIndex = oldFVarValueIndices[fvarOffset];
            Index newFVIndex = (Index)newFVarCoords.size();

            assert(oldFVIndex < (Index)oldFVarCoords.size());
            FVarIndexMap::iterator itFound = localFVarMap.find(oldFVIndex);
            if (itFound != localFVarMap.end()) {
                newFVarValueIndices[fvarOffset] = itFound->second;
            } else {
                newFVarValueIndices[fvarOffset] = newFVIndex;
                newFVarCoords.push_back(oldFVarCoords[oldFVIndex]);

                localFVarMap[oldFVIndex] = newFVIndex;
            }
        }
    }

    //  Now assign the new FVar indices and values and rebuild the refiner:
    _uvwCoords.swap(newFVarCoords);
    _topContainer.fvarValueIndices.swap(newFVarValueIndices);
    _topContainer.fvarValueCount = (int)_uvwCoords.size();

    Rebuild(0, 0, 0);
}

void
TestMesh::ClearUvChannel() {

    _topContainer.fvarValueCount = 0;
    _topContainer.fvarValueIndices.clear();

    _uvwCoords.clear();
}

void
TestMesh::interpolateVertexData() {

    Far::PrimvarRefiner primvarRefiner(*_topRefiner);

    _xyzCoords.resize(_topRefiner->GetNumVerticesTotal());
    Coord3 * srcXyz = &_xyzCoords[0];

    for (int level = 1; level <= _topRefiner->GetMaxLevel(); ++level) {
        Coord3 * dstXyz = srcXyz + _topRefiner->GetLevel(level-1).GetNumVertices();

        primvarRefiner.Interpolate(level, srcXyz, dstXyz);

        srcXyz = dstXyz;
    }
    if (_patchTable && _patchTable->GetNumLocalPoints()) {
        int nRefinedPoints = _topRefiner->GetNumVerticesTotal();
        int nPatchPoints   = _patchTable->GetNumLocalPoints();

        _xyzCoords.resize(_xyzCoords.size() + nPatchPoints);
        _patchTable->ComputeLocalPointValues(&_xyzCoords[0], &_xyzCoords[nRefinedPoints]);
    }
}

void
TestMesh::interpolateFaceVaryingData() {

    if (_uvwCoords.size() == 0) return;

    Far::PrimvarRefiner primvarRefiner(*_topRefiner);

    int uvChannel = GetUvChannel();

    _uvwCoords.resize(_topRefiner->GetNumFVarValuesTotal(uvChannel));
    Coord3 * srcUvw = &_uvwCoords[0];

    for (int level = 1; level <= _topRefiner->GetMaxLevel(); ++level) {
        Coord3 * dstUvw = srcUvw + _topRefiner->GetLevel(level-1).GetNumFVarValues(uvChannel);

        primvarRefiner.InterpolateFaceVarying(level, srcUvw, dstUvw, uvChannel);

        srcUvw = dstUvw;
    }
    if (_patchTable && _patchTable->GetNumLocalPointsFaceVarying(uvChannel)) {
        int nRefinedPoints = _topRefiner->GetNumFVarValuesTotal(uvChannel);
        int nPatchPoints   = _patchTable->GetNumLocalPointsFaceVarying(uvChannel);

        _uvwCoords.resize(_uvwCoords.size() + nPatchPoints);
        _patchTable->ComputeLocalPointValuesFaceVarying(&_uvwCoords[0], &_uvwCoords[nRefinedPoints], uvChannel);
    }
}

bool
TestMesh::WriteToObjFile(char const * objFilename, int levelIndex) const {

    if (levelIndex < 0) levelIndex = _topRefiner->GetMaxLevel();
printf("... writing level %d to Obj file '%s' ...\n", levelIndex, objFilename);

    FILE * fptr = fopen(objFilename, "w");
    if (fptr == 0) return false;

    TopologyLevel const & level = _topRefiner->GetLevel(levelIndex);

    int nFaces = level.GetNumFaces();
    int nVerts = level.GetNumVertices();
    int nEdges = level.GetNumEdges();
    int nUvs   = HasUvs() ? level.GetNumFVarValues(GetUvChannel()) : 0;

    Coord3 const * xyzCoords =        &_xyzCoords[0];
    Coord3 const * uvwCoords = nUvs ? &_uvwCoords[0] : 0;
    for (int i = 1; i <= levelIndex; ++i) {
        xyzCoords +=        _topRefiner->GetLevel(i-1).GetNumVertices();
        uvwCoords += nUvs ? _topRefiner->GetLevel(i-1).GetNumFVarValues(GetUvChannel()) : 0;
    }

    //  Print vertex positions
    for (int i = 0; i < nVerts; ++i) {
        Coord3 const & xyz = xyzCoords[i];
        fprintf(fptr, "v %f %f %f\n", xyz.x, xyz.y, xyz.z);
    }
        
    //  Print UV values (only the UV of the stored UVW)
    for (int i = 0; i < nUvs; ++i) {
        Coord3 const & uvw = uvwCoords[i];
        fprintf(fptr, "vt %f %f\n", uvw.x, uvw.y);
    }

    //  Print faces
    for (int i = 0; i < nFaces; ++i) {
        ConstIndexArray fVerts = level.GetFaceVertices(i);
        ConstIndexArray fUvs   = nUvs ? level.GetFaceFVarValues(i, GetUvChannel()) : fVerts;

        fprintf(fptr, "f ");
        for (int j=0; j < fVerts.size(); ++j) {
            // Remember -- the Obj format uses 1-based indices...
            if (nUvs) {
                fprintf(fptr, "%d/%d ", 1 + fVerts[j], 1 + fUvs[j]);
            } else {
                fprintf(fptr, "%d ", 1 + fVerts[j]);
            }
        }
        fprintf(fptr, "\n");
    }

    //  Print corner and crease tags:
    for (int i = 0; i < nVerts; ++i) {
        float vSharpness = level.GetVertexSharpness(i);
        if (vSharpness > 0.0) {
            fprintf(fptr, "t corner 1/1/0  %d  %g\n", i, vSharpness);
        }
    }
    for (int i = 0; i < nEdges; ++i) {
        float eSharpness = level.GetEdgeSharpness(i);
        if (eSharpness > 0.0) {
            ConstIndexArray ev = level.GetEdgeVertices(i);
            fprintf(fptr, "t crease 2/1/0  %d %d  %g\n", ev[0], ev[1], eSharpness);
        }
    }

/*
    //  Add the local patch points after all of the faces:
    if (_patchTable) {
        if (_patchTable->GetNumLocalPoints()) {
            int nRefinedPoints = _topRefiner->GetNumVerticesTotal();
            int nPatchPoints   = _patchTable->GetNumLocalPoints();

            xyzCoords = &_xyzCoords[nRefinedPoints];
            for (int i = 0; i < nPatchPoints; ++i) {
                Coord3 const & xyz = xyzCoords[i];
                fprintf(fptr, "v %f %f %f\n", xyz.x, xyz.y, xyz.z);
            }
        }
        if (nUvs && _patchTable->GetNumLocalPointsFaceVarying(GetUvChannel())) {
            int nRefinedPoints = _topRefiner->GetNumFVarValuesTotal(GetUvChannel());
            int nPatchPoints   = _patchTable->GetNumLocalPointsFaceVarying(GetUvChannel());

            uvwCoords = &_uvwCoords[nRefinedPoints];
            for (int i = 0; i < nPatchPoints; ++i) {
                Coord3 const & uvw = uvwCoords[i];
                fprintf(fptr, "vt %f %f\n", uvw.x, uvw.y);
            }
        }
    }
*/
    fclose(fptr);

    return true;
}
