//
//   Copyright 2022
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

#include "subclassOfSurfaceFactory.h"

#include <opensubdiv/bfr/vertexTopology.h>
#include <opensubdiv/far/topologyLevel.h>


using OpenSubdiv::Far::TopologyRefiner;
using OpenSubdiv::Far::TopologyLevel;

using OpenSubdiv::Bfr::Index;
using OpenSubdiv::Bfr::ConstIndexArray;
using OpenSubdiv::Bfr::ConstLocalIndexArray;


//
//  Main constructor and destructor:
//
SubclassOfSurfaceFactory::SubclassOfSurfaceFactory(
    TopologyRefiner const & mesh, Options factoryOptions) :
        SurfaceFactory(mesh.GetSchemeType(),
                       mesh.GetSchemeOptions(),
                       factoryOptions),
        _mesh(mesh),
        _localCache() {

}

SubclassOfSurfaceFactory::~SubclassOfSurfaceFactory() {

}

//
//  Virtual methods supporting Surface creation and population:
//
//  Simple/trivial face queries:
//
bool
SubclassOfSurfaceFactory::isFaceHole(Index face) const {

    return _mesh.HasHoles() && _mesh.GetLevel(0).IsFaceHole(face);
}

int
SubclassOfSurfaceFactory::getFaceSize(Index baseFace) const {

    return _mesh.GetLevel(0).GetFaceVertices(baseFace).size();
}

//
//  Specifying vertex or face-varying indices for a face:
//
int
SubclassOfSurfaceFactory::getFaceVertexIndices(Index baseFace,
        Index indices[]) const {

    ConstIndexArray fVerts = _mesh.GetLevel(0).GetFaceVertices(baseFace);

    std::memcpy(indices, &fVerts[0], fVerts.size() * sizeof(Index));
    return fVerts.size();
}

int
SubclassOfSurfaceFactory::getFaceFVarValueIndices(Index baseFace,
        int fvarID, Index indices[]) const {

    if (fvarID >= GetNumFVarChannels()) return 0;

    ConstIndexArray fvarValues =
            _mesh.GetLevel(0).GetFaceFVarValues(baseFace, fvarID);

    std::memcpy(indices, &fvarValues[0], fvarValues.size() * sizeof(Index));
    return fvarValues.size();
}

//
//  Specifying the topology around a face-vertex:
//
int
SubclassOfSurfaceFactory::populateFaceVertexTopology(
        Index baseFace, int cornerVertex,
        OpenSubdiv::Bfr::VertexTopology * vertexTopology) const {

    OpenSubdiv::Bfr::VertexTopology & vt = *vertexTopology;

    TopologyLevel const & baseLevel = _mesh.GetLevel(0);

    //
    //  Identify the vertex index for the specified corner of the face
    //  and topology information related to it:
    //
    Index vIndex = baseLevel.GetFaceVertices(baseFace)[cornerVertex];

    ConstIndexArray vFaces = baseLevel.GetVertexFaces(vIndex);

    int  numFaces   = vFaces.size();
    bool isManifold = !baseLevel.IsVertexNonManifold(vIndex);

    //
    //  Initialize, assign and finalize the vertex topology:
    //
    vt.Initialize(numFaces);
    {
        //  Assign ordering and boundary status:
        vt.SetManifold(isManifold);
        vt.SetBoundary(baseLevel.IsVertexBoundary(vIndex));

        //  Assign sizes of incident faces:
        vt.SetCommonFaceSize(false);
        for (int i = 0; i < numFaces; ++i) {
            vt.SetIncidentFaceSize(i,
                        baseLevel.GetFaceVertices(vFaces[i]).size());
        }

        //  Assign vertex sharpness:
        vt.SetVertexSharpness(baseLevel.GetVertexSharpness(vIndex));

        //  Assign edge sharpness:
        if (isManifold) {
            //  Can use manifold/ordered edge indices here:
            ConstIndexArray vEdges = baseLevel.GetVertexEdges(vIndex);

            for (int i = 0; i < vEdges.size(); ++i) {
                vt.SetManifoldEdgeSharpness(i,
                        baseLevel.GetEdgeSharpness(vEdges[i]));
            }
        } else {
            //  Must use face-edges and identify next/prev edges in face:
            ConstLocalIndexArray vInFace =
                baseLevel.GetVertexFaceLocalIndices(vIndex);

            for (int i = 0; i < numFaces; ++i) {
                ConstIndexArray fEdges = baseLevel.GetFaceEdges(vFaces[i]);

                int eLeading  = vInFace[i];
                int eTrailing = (eLeading ? eLeading : fEdges.size()) - 1;

                vt.SetIncidentFaceEdgeSharpness(i,
                        baseLevel.GetEdgeSharpness(fEdges[eLeading]),
                        baseLevel.GetEdgeSharpness(fEdges[eTrailing]));
            }
        }
    }
    vt.Finalize();

    //
    //  Return the index of the base face in the set of incident faces
    //  around the vertex:
    //
    if (isManifold) {
        return vFaces.FindIndex(baseFace);
    } else {
        //
        //  Remember that for some non-manifold cases the face may occur
        //  multiple times around this vertex, so make sure to identify
        //  the instance of the base face whose corner face-vertex matches
        //  the one that was specified:
        //
        ConstLocalIndexArray vInFace =
                baseLevel.GetVertexFaceLocalIndices(vIndex);
        for (int i = 0; i < numFaces; ++i) {
            if ((vFaces[i] == baseFace) && (vInFace[i] == cornerVertex)) {
                return i;
            }
        }
        assert("Cannot identify face-vertex around non-manifold vertex." == 0);
    }
}


//
//  Specifying vertex and face-varying indices around a face-vertex --
//  both virtual methods trivially use a common internal method to get
//  the indices for a particular vertex Index:
//
int
SubclassOfSurfaceFactory::getFaceVertexIncidentFaceVertexIndices(
        Index baseFace, int cornerVertex,
        Index indices[]) const {

    return getFaceVertexPointIndices(baseFace, cornerVertex, indices, -1);
}

int
SubclassOfSurfaceFactory::getFaceVertexIncidentFaceFVarValueIndices(
        Index baseFace, int cornerVertex,
        int fvarID, Index indices[]) const {

    if (fvarID >= GetNumFVarChannels()) return 0;

    return getFaceVertexPointIndices(baseFace, cornerVertex, indices, fvarID);
}

int
SubclassOfSurfaceFactory::getFaceVertexPointIndices(
        Index baseFace, int cornerVertex,
        Index indices[], int vtxOrFVarChannel) const {

    TopologyLevel const & baseLevel = _mesh.GetLevel(0);

    Index vIndex = baseLevel.GetFaceVertices(baseFace)[cornerVertex];

    ConstIndexArray      vFaces  = baseLevel.GetVertexFaces(vIndex);
    ConstLocalIndexArray vInFace = baseLevel.GetVertexFaceLocalIndices(vIndex);

    int nIndices = 0;
    for (int i = 0; i < vFaces.size(); ++i) {
        ConstIndexArray srcIndices = (vtxOrFVarChannel < 0) ?
                baseLevel.GetFaceVertices(vFaces[i]) :
                baseLevel.GetFaceFVarValues(vFaces[i], vtxOrFVarChannel);

        int srcStart = vInFace[i];
        int srcCount = srcIndices.size();
        for (int j = 0; j < srcCount; ++j) {
            indices[nIndices++] = srcIndices[(srcStart + j) % srcCount];
        }
    }
    return nIndices;
}
