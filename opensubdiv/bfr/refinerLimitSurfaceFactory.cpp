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

#include "../far/topologyRefiner.h"

#include "../bfr/refinerLimitSurfaceFactory.h"
#include "../bfr/vertexTopology.h"

#include <map>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Bfr {

//
//  Main constructor and destructor:
//
RefinerLimitSurfaceFactory::RefinerLimitSurfaceFactory(
    Far::TopologyRefiner const & mesh, Options limitOptions) :
        LimitSurfaceFactory(mesh.GetSchemeType(),
                            mesh.GetSchemeOptions(),
                            limitOptions,
                            mesh.GetLevel(0).GetNumFaces(),
                            mesh.GetNumFVarChannels()),
        _mesh(mesh) {

}

RefinerLimitSurfaceFactory::~RefinerLimitSurfaceFactory() {

}

//
//  Virtual methods supporting LimitSurface creation and population:
//
//  Simple/trivial face queries:
//
bool
RefinerLimitSurfaceFactory::isFaceHole(Index face) const {

    return _mesh.HasHoles() && _mesh.getLevel(0).isFaceHole(face);
}

int
RefinerLimitSurfaceFactory::getFaceSize(Index baseFace) const {

    return _mesh.GetLevel(0).GetFaceVertices(baseFace).size();
}

//
//  Specifying vertex or face-varying indices for a face:
//
int
RefinerLimitSurfaceFactory::getFaceVertexIndices(Index baseFace,
        Index indices[]) const {

    ConstIndexArray fVerts = _mesh.GetLevel(0).GetFaceVertices(baseFace);

    std::memcpy(indices, &fVerts[0], fVerts.size() * sizeof(Index));
    return fVerts.size();
}

int
RefinerLimitSurfaceFactory::getFaceFVarValueIndices(Index baseFace,
        Index indices[], int fvarIndex) const {

    if (fvarIndex >= _mesh.GetNumFVarChannels()) return 0;

    ConstIndexArray fvarValues =
            _mesh.GetLevel(0).GetFaceFVarValues(baseFace, fvarIndex);

    std::memcpy(indices, &fvarValues[0], fvarValues.size() * sizeof(Index));
    return fvarValues.size();
}

//
//  Specifying the topology around a face-vertex:
//
int
RefinerLimitSurfaceFactory::populateFaceCornerTopology(
        Index baseFace, int cornerVertex,
        VertexTopology & vertexTopology) const {

    //
    //  Identify the vertex index for the specified corner of the face
    //  and topology information related to it:
    //
    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    Index vIndex = baseLevel.getFaceVertices(baseFace)[cornerVertex];

    ConstIndexArray vFaces = baseLevel.getVertexFaces(vIndex);
    int             nFaces = vFaces.size();

    Vtr::internal::Level::VTag vTag = baseLevel.getVertexTag(vIndex);
    bool isManifold = !vTag._nonManifold;

    //
    //  Initialize, assign and finalize the vertex topology:
    //
    vertexTopology.Initialize(nFaces);
    {
        //  Assign ordering and boundary status:
        if (isManifold) {
            vertexTopology.SetOrdered(true);
            vertexTopology.SetBoundary(vTag._boundary);
        }

        //  Assign face sizes -- variable/explicit or constant/implicit:
        if (vTag._incidIrregFace) {
            vertexTopology.SetCommonFaceSize(false);

            int * faceSizes = vertexTopology.AccessFaceSizeBuffer();
            for (int i = 0; i < nFaces; ++i) {
                faceSizes[i] = baseLevel.getFaceVertices(vFaces[i]).size();
            }
        } else {
            vertexTopology.SetCommonFaceSize(true);
        }

        //  Assign vertex sharpness:
        if (vTag._semiSharp || vTag._infSharp) {
            vertexTopology.SetVertexSharpness(
                    baseLevel.getVertexSharpness(vIndex));
        }

        //  Assign edge sharpness (try to avoid when sharpness is implicit):
        if (vTag._semiSharpEdges || vTag._infSharpEdges) {
            if (isManifold) {
                ConstIndexArray vEdges = baseLevel.getVertexEdges(vIndex);

                float * sharp = vertexTopology.AccessFaceEdgeSharpnessBuffer(0);

                *sharp++ = baseLevel.getEdgeSharpness(vEdges[0]);
                for (int i = 1; i < nFaces; ++i) {
                    float eSharp = baseLevel.getEdgeSharpness(vEdges[i]);
                    *sharp++ = eSharp;
                    *sharp++ = eSharp;
                }
                *sharp++ = vTag._boundary
                         ? baseLevel.getEdgeSharpness(vEdges[nFaces])
                         : baseLevel.getEdgeSharpness(vEdges[0]);
            } else {
                //  WIP - traverse faces, use leading/trailing edges
            }
        }
    }
    vertexTopology.Finalize();

    //
    //  Return the index of the base face around the vertex:
    //
    if (isManifold) {
        return vFaces.FindIndex(baseFace);
    } else {
        //  WIP - the face may occur multiple times around the vertex, so
        //  need to eventually use the instance matching this face-corner
        return vFaces.FindIndex(baseFace);
    }
}


//
//  Specifying vertex and face-varying indices around a face-vertex --
//  both virtual methods trivially use a common internal method to get
//  the indices for a particular vertex Index:
//
int
RefinerLimitSurfaceFactory::getFaceCornerIndices(
        Index baseFace, int cornerVertex,
        Index indices[], int fvarIndex) const {

    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    Index vIndex = baseLevel.getFaceVertices(baseFace)[cornerVertex];

    ConstIndexArray      vFaces  = baseLevel.getVertexFaces(vIndex);
    ConstLocalIndexArray vInFace = baseLevel.getVertexFaceLocalIndices(vIndex);

    int nIndices = 0;
    for (int i = 0; i < vFaces.size(); ++i) {
        ConstIndexArray srcIndices = (fvarIndex < 0) ?
                           baseLevel.getFaceVertices(vFaces[i]) :
                           baseLevel.getFaceFVarValues(vFaces[i], fvarIndex);

        int srcStart = vInFace[i];
        int srcCount = srcIndices.size();
        for (int j = 0; j < srcCount; ++j) {
            indices[nIndices++] = srcIndices[(srcStart + j) % srcCount];
        }
    }
    return nIndices;
}

int
RefinerLimitSurfaceFactory::getFaceCornerVertexIndices(
        Index baseFace, int cornerVertex,
        Index indices[]) const {

    return getFaceCornerIndices(baseFace, cornerVertex, indices, -1);
}

int
RefinerLimitSurfaceFactory::getFaceCornerFVarValueIndices(
        Index baseFace, int cornerVertex,
        Index indices[], int fvar) const {

    return getFaceCornerIndices(baseFace, cornerVertex, indices, fvar);
}

//
//  TEMPORARY methods for development and debugging to identify cases
//  that are not yet fully supported, and so which are likely producing
//  incorrect results (e.g. a linear proxy patch):
//
//  Currently Loop patches and two topological conditions for Catmark
//  are not fully supported:
//
//      - any kind of non-manifold feature
//      - faces with any valence-2 vertices
//
//  Other features are supported but some are relatively fresh (e.g.
//  incident non-quads or explicit creasing) and so not as well tested.
//
namespace {
    bool
    isVertNonManifold(Vtr::internal::Level const & level, Index vIndex) {
        return level.getVertexTag(vIndex)._nonManifold;
    }
    bool
    isVertVal2Interior(Vtr::internal::Level const & level, Index vIndex) {
        return (level.getVertexFaces(vIndex).size() == 2) &&
               !level.getVertexTag(vIndex)._boundary;
    }

    bool
    isFaceNonManifold(Vtr::internal::Level const & level, Index fIndex) {
        ConstIndexArray fVerts = level.getFaceVertices(fIndex);
        for (int i = 0; i < fVerts.size(); ++i) {
            if (isVertNonManifold(level, fVerts[i])) return true;
        }
        return false;
    }
    bool
    isFaceVal2Interior(Vtr::internal::Level const & level, Index fIndex) {
        ConstIndexArray fVerts = level.getFaceVertices(fIndex);
        for (int i = 0; i < fVerts.size(); ++i) {
            if (isVertVal2Interior(level, fVerts[i])) return true;
        }
        return false;
    }
}

bool
RefinerLimitSurfaceFactory::IsFaceUnsupported(Index fIndex) const {

    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    return (getRegularFaceSize() == 3) ||
           isFaceNonManifold(baseLevel, fIndex) ||
           isFaceVal2Interior(baseLevel, fIndex);
}

bool
RefinerLimitSurfaceFactory::HasUnsupportedFaces() const {

    int nFaces = _mesh.getLevel(0).getNumFaces();
    for (int fIndex = 0; fIndex < nFaces; ++fIndex) {
        if (IsFaceUnsupported(fIndex)) return true;
    }
    return false;
}

int
RefinerLimitSurfaceFactory::GetNumUnsupportedFaces() const {

    int nFacesUnsupported = 0;

    int nFaces = _mesh.getLevel(0).getNumFaces();
    for (int fIndex = 0; fIndex < nFaces; ++fIndex) {
        nFacesUnsupported += IsFaceUnsupported(fIndex);
    }
    return nFacesUnsupported;
}

int
RefinerLimitSurfaceFactory::GetNumNonManifoldFaces() const {

    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    int nFacesNonManifold = 0;

    int nFaces = _mesh.getLevel(0).getNumFaces();
    for (int fIndex = 0; fIndex < nFaces; ++fIndex) {
        nFacesNonManifold += isFaceNonManifold(baseLevel, fIndex);
    }
    return nFacesNonManifold;
}

int
RefinerLimitSurfaceFactory::GetNumVal2InteriorFaces() const {

    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    int nFacesVal2Interior = 0;

    int nFaces = _mesh.getLevel(0).getNumFaces();
    for (int fIndex = 0; fIndex < nFaces; ++fIndex) {
        nFacesVal2Interior += isFaceVal2Interior(baseLevel, fIndex);
    }
    return nFacesVal2Interior;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
