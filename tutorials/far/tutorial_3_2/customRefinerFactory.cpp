//
//   Copyright 2019 DreamWorks Animation LLC.
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

#include "customRefinerFactory.h"

#include <cassert>

//
//  Specializations defined in the appropriate namespace:
//
//  Requirements for these specializations can be found with the default
//  implementations in <opensubdiv/far/topologyRefinerFactory.h>.
//
namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Far {

namespace {
    template <typename T>
    void copyArray(Vtr::ConstArray<T> src, Vtr::Array<T> dst) {
        assert(src.size() == dst.size());
        std::memcpy(&dst[0], &src[0], src.size() * sizeof(T));
    }
}

template <>
bool
TopologyRefinerFactory<TopologyLevel>::resizeComponentTopology(
    TopologyRefiner & refiner, TopologyLevel const & level) {

    //
    //  Since defining a specific edge list, all six relations must be sized
    //  (though two are clearly defined -- edge-verts and face-edges -- leaving
    //  the remaining four to specify).
    //

    //  Faces and face-verts:
    int nFaces = level.GetNumFaces();
    setNumBaseFaces(refiner, nFaces);
    for (int face = 0; face < nFaces; ++face) {
        setNumBaseFaceVertices(refiner, face, level.GetFaceVertices(face).size());
    }

    //  Edges and edge-faces:
    int nEdges = level.GetNumEdges();
    setNumBaseEdges(refiner, nEdges);
    for (int edge = 0; edge < nEdges; ++edge) {
        setNumBaseEdgeFaces(refiner, edge, level.GetEdgeFaces(edge).size());
    }

    //  Vertices and vert-faces and vert-edges:
    int nVerts = level.GetNumVertices();
    setNumBaseVertices(refiner, nVerts);
    for (int vert = 0; vert < nVerts; ++vert) {
        setNumBaseVertexEdges(refiner, vert, level.GetVertexEdges(vert).size());
        setNumBaseVertexFaces(refiner, vert, level.GetVertexFaces(vert).size());
    }
    return true;
}

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignComponentTopology(
    TopologyRefiner & refiner, TopologyLevel const & level) {

    //
    //  Defining a specific edge list requires defining all six topological
    //  relations.  Given the TopologyLevel as the source, and that the base
    //  level of the new TopologyRefiner is similarly defined, we can simply
    //  copy all relations.
    //
    //  Note that, in general, attempting to define an edge list and relations
    //  for non-manifold meshes is not advised.  Unless intimately familiar
    //  with the topological requirements of a non-manifold mesh in the
    //  TopologyRefiner, it is best to simple rely on the face-vertex list and
    //  have edges constructed as needed.
    //

    //  Face relations -- face-verts and face-edges:
    for (int face = 0; face < level.GetNumFaces(); ++face) {
        copyArray(level.GetFaceVertices(face),
                  getBaseFaceVertices(refiner, face));

        copyArray(level.GetFaceEdges(face),
                  getBaseFaceEdges(refiner, face));
    }

    //  Edge relations -- edge-verts and edge-faces:
    for (int edge = 0; edge < level.GetNumEdges(); ++edge) {
        copyArray(level.GetEdgeVertices(edge),
                  getBaseEdgeVertices(refiner, edge));

        copyArray(level.GetEdgeFaces(edge),
                  getBaseEdgeFaces(refiner, edge));
        copyArray(level.GetEdgeFaceLocalIndices(edge),
                  getBaseEdgeFaceLocalIndices(refiner, edge));

        if (level.IsEdgeNonManifold(edge)) {
            setBaseEdgeNonManifold(refiner, edge, true);
        }
    }

    //  Vertex relations -- vert-edges and vert-faces:
    for (int vert = 0; vert < level.GetNumVertices(); ++vert) {
        copyArray(level.GetVertexEdges(vert),
                  getBaseVertexEdges(refiner, vert));
        copyArray(level.GetVertexEdgeLocalIndices(vert),
                  getBaseVertexEdgeLocalIndices(refiner, vert));

        copyArray(level.GetVertexFaces(vert),
                  getBaseVertexFaces(refiner, vert));
        copyArray(level.GetVertexFaceLocalIndices(vert),
                  getBaseVertexFaceLocalIndices(refiner, vert));

        if (level.IsVertexNonManifold(vert)) {
            setBaseVertexNonManifold(refiner, vert, true);
        }
    }
    return true;
}

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignComponentTags(
    TopologyRefiner & refiner, TopologyLevel const & level) {

    //
    //  Since the edge lists match between the TopologyLevel and the new
    //  TopologyRefiner, any non-zero sharpness values can simply be copied
    //  between -- avoiding the need to identify edges based on vertex pair.
    //
    //  Assign edge sharpness (creases):
    for (int edge = 0; edge < level.GetNumEdges(); ++edge) {
        float edgeSharpness = level.GetEdgeSharpness(edge);
        if (edgeSharpness > 0.0f) {
            setBaseEdgeSharpness(refiner, edge, edgeSharpness);
        }
    }

    //  Assign edge sharpness (corners):
    for (int vert = 0; vert < level.GetNumVertices(); ++vert) {
        float vertSharpness = level.GetVertexSharpness(vert);
        if (vertSharpness > 0.0f) {
            setBaseVertexSharpness(refiner, vert, vertSharpness);
        }
    }

    //  Assign face holes:
    for (int face = 0; face < level.GetNumFaces(); ++face) {
        if (level.IsFaceHole(face)) {
            setBaseFaceHole(refiner, face, true);
        }
    }
    return true;
}

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignFaceVaryingTopology(
    TopologyRefiner & refiner, TopologyLevel const & level) {

    //
    //  Since face-varying data is not related to edges in any way, its
    //  assignment is typical:
    //
    for (int channel = 0; channel < level.GetNumFVarChannels(); ++channel) {
        createBaseFVarChannel(refiner, level.GetNumFVarValues(channel));

        for (int face = 0; face < level.GetNumFaces(); ++face) {
            copyArray(level.GetFaceFVarValues(face, channel),
                      getBaseFaceFVarValues(refiner, face, channel));
        }
    }
    return true;
}

template <>
void
TopologyRefinerFactory<TopologyLevel>::reportInvalidTopology(
    TopologyError /* errCode */, char const * msg, TopologyLevel const& /* level */) {
    Warning(msg);
}

} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
