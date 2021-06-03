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
#include "../far/patchBuilder.h"
#include "../far/patchTreeFactory.h"

#include "../bfr/refinerLimitSurfaceFactory.h"
#include "../bfr/faceDescriptors.h"
#include "../bfr/faceBuilders.h"
#include "../bfr/topologyCache.h"

#include <map>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Bfr {

//
//  Main constructor and destructor -- configures the base class for
//  support of all current Descriptor types.
//
//  An instance of Far::PatchBuilder is useful as a member and requires
//  some trivial configuration to initialize:
//
RefinerLimitSurfaceFactory::RefinerLimitSurfaceFactory(
    Far::TopologyRefiner const & mesh, Options limitOptions) :
        LimitSurfaceFactory(mesh.GetSchemeType(),
                            mesh.GetSchemeOptions(),
                            limitOptions,
                            true,   // supports Regular descriptors
                            true,   // supports Manifold descriptors
                            true,   // supports General descriptors
                            mesh.GetLevel(0).GetNumFaces()),
        _mesh(mesh),
        _patchBuilder(0) {

    //
    //  We only need to populate the FVar indices in the face topology
    //  Descriptors when both the option to do so is present and the
    //  mesh actually has a FVar channel to populate:
    //
    _populateFVarTopology = limitOptions.CreateFVarEvaluators() &&
                            mesh.GetNumFVarChannels();

    //
    //  A Far::PatchBuilder is really useful for quickly (and robustly)
    //  identifying and populating regular patches:
    //
    Far::PatchBuilder::Options pBuilderOptions;
    pBuilderOptions.regBasisType   = Far::PatchBuilder::BASIS_REGULAR;
    pBuilderOptions.irregBasisType = Far::PatchBuilder::BASIS_GREGORY;
    pBuilderOptions.approxInfSharpWithSmooth    = false;
    pBuilderOptions.approxSmoothCornerWithSharp = false;
    pBuilderOptions.fillMissingBoundaryPoints   = false;

    _patchBuilder = Far::PatchBuilder::Create(mesh, pBuilderOptions);
}

RefinerLimitSurfaceFactory::~RefinerLimitSurfaceFactory() {

    delete _patchBuilder;
}

//
//  Methods supporting LimitSurface creation and population:
//
bool
RefinerLimitSurfaceFactory::isFaceHole(Index face) const {

    return _mesh.HasHoles() && _mesh.getLevel(0).isFaceHole(face);
}

bool
RefinerLimitSurfaceFactory::isFaceLimitRegular(Index face) const {

    //  Deal with linear schemes trivially first:
    if (_patchBuilder->GetRegularPatchType() == _patchBuilder->GetLinearPatchType()) {
        return (_mesh.getLevel(0).getFaceVertices(face).size() ==
                _patchBuilder->GetRegularFaceSize());
    }

    //
    //  We want to use the PatchBuilder to quickly identify and populate a
    //  regular patch, but due to a bug with level 0 patches, we can't use
    //  the PatchBuilder::IsPatchRegular(0,face) test.
    //
    //  A quick test of the VTags of the face corners covers most regular
    //  cases instead.
    //
    Vtr::internal::Level::VTag fTag = _mesh.getLevel(0).getFaceCompositeVTag(face);

    return !fTag._incidIrregFace &&
           !fTag._xordinary &&
           !fTag._nonManifold &&
           !fTag._semiSharp && !fTag._semiSharpEdges &&
           !fTag._infIrregular;
}

//
//  Methods to populate the different kinds of topology Descriptors -- these
//  will eventually be virtual methods that derived classes will define.
//
bool
RefinerLimitSurfaceFactory::populateDescriptor(Index baseFace,
        RegularFaceDescriptor & desc) const {

    if (!isFaceLimitRegular(baseFace)) {
        return false;
    }

    //
    //  Don't use Regular descriptor if FVar limit does not share matching
    //  topology, but don't penalize the linear FVar case:
    //
    if (_populateFVarTopology) {
        bool fvarIsLinear = (_mesh.GetFVarLinearInterpolation(0) ==
                             Sdc::Options::FVAR_LINEAR_ALL);

        //  WIP - Only linear FVar supported, keep regular patch for now
        fvarIsLinear = true;

        if (!fvarIsLinear) {
            if (!_patchBuilder->DoesFaceVaryingPatchMatch(0, baseFace, 0)) {
                return false;
            }
        }
    }

    //
    //  Identify the regular patch type and its size:
    //
    Far::PatchDescriptor::Type patchType = _patchBuilder->GetRegularPatchType();

    int patchSize = Far::PatchDescriptor(patchType).GetNumControlVertices();

    int boundaryMask = _patchBuilder->GetRegularPatchBoundaryMask(0, baseFace);

    //
    //  Initialize the Descriptor and load its control points and boundary mask:
    //
    desc.Initialize(patchSize);

    _patchBuilder->GetRegularPatchPoints(0, baseFace, boundaryMask,
            desc.AccessPatchVertexIndices());

    if (_populateFVarTopology) {
        //  Gather all FVar patch points, even if FVar is linear -- those
        //  around the face will be ignored:
        _patchBuilder->GetRegularPatchPoints(0, baseFace, boundaryMask,
                desc.AccessPatchFVarValueIndices(), 0);
    }

    desc.SetBoundaryMask(boundaryMask);

    desc.Finalize();

    return true;
}

//
//  Internal topology traversal methods to support populating Descriptors:
//
namespace {
    //
    //  A few useful low-level utilities first:
    //
    using Vtr::internal::Level;
    template <typename INT_TYPE>
    inline INT_TYPE fastMod4(INT_TYPE value) {

        return (value & 0x3);
    }

    template <class ARRAY_OF_TYPE, class TYPE>
    inline TYPE otherOfTwo(ARRAY_OF_TYPE const& arrayOfTwo, TYPE const& value) {

        return arrayOfTwo[value == arrayOfTwo[0]];
    }

    template <typename INT_TYPE>
    void
    printIntArray(INT_TYPE const array[], int size, char const * prefix = 0) {

        if (prefix) {
            printf("%s", prefix);
        }
        for (int i = 0; i < size; ++i) {
            if (i) printf(" ");
            printf("%3d", (int)array[i]);
        }
    }

    //
    //  This specialization for quads was copied from Vtr::Level, and the
    //  extensions required for face-varying removed for now.
    //
    //  For every incident quad, we want the two vertices clockwise in each
    //  face, i.e. the vertex at the end of the leading edge and the vertex
    //  opposite the given central vertex:
    //
    int
    meshGatherVertexOneRing_quads(Level const & mesh, Index vIndex, int ringVerts[]) {

        ConstIndexArray vEdges = mesh.getVertexEdges(vIndex);

        ConstIndexArray vFaces = mesh.getVertexFaces(vIndex);
        ConstLocalIndexArray vInFaces = mesh.getVertexFaceLocalIndices(vIndex);

        bool isBoundary = (vEdges.size() > vFaces.size());

        int ringIndex = 0;
        for (int i = 0; i < vFaces.size(); ++i) {
            ConstIndexArray fVerts = mesh.getFaceVertices(vFaces[i]);

            int vInThisFace = vInFaces[i];

            ringVerts[ringIndex++] = fVerts[fastMod4(vInThisFace + 1)];
            ringVerts[ringIndex++] = fVerts[fastMod4(vInThisFace + 2)];

            if (isBoundary && (i == (vFaces.size() - 1))) {
                ringVerts[ringIndex++] = fVerts[fastMod4(vInThisFace + 3)];
            }
        }
        return ringIndex;
    }

    int
    meshGatherVertexOneRing(Level const & mesh, Index vIndex,
                            int ringVerts[], int startFaceIndex = -1) {

        bool optimize = false;
        if (optimize) {
            if (!mesh.getVertexTag(vIndex)._incidIrregFace) {
              return meshGatherVertexOneRing_quads(mesh, vIndex, ringVerts);
            }
        }

        bool isBoundary = mesh.getVertexTag(vIndex)._boundary;

        ConstIndexArray      vFaces   = mesh.getVertexFaces(vIndex);
        ConstLocalIndexArray vInFaces = mesh.getVertexFaceLocalIndices(vIndex);
/*
printf("    meshGatherVertexOneRing()...\n");
printf("        start-face-index = %d\n", startFaceIndex);
printf("        vertex-faces (%d):  ", vFaces.size());
printIntArray(&vFaces[0], vFaces.size());
printf("\n");
*/
        int faceRingStart = 0;
        if ((startFaceIndex >= 0) && !isBoundary) {
            faceRingStart = vFaces.FindIndex(startFaceIndex);
            if (faceRingStart < 0) faceRingStart = 0;
        }

        int ringSize = 0;
        for (int face = 0; face < vFaces.size(); ++face) {
            int faceInRing = faceRingStart + face;
            if (faceInRing >= vFaces.size()) faceInRing -= vFaces.size();

            ConstIndexArray fVerts = mesh.getFaceVertices(vFaces[faceInRing]);

            int vInThisFace = vInFaces[faceInRing];

            //  Append the leading edge vertex and interior face-verts:
            int fvCount = fVerts.size();
            for (int fv = 1; fv < (fvCount - 1); ++fv) {
                int fvIndex = vInThisFace + fv;
                if (fvIndex >= fvCount) fvIndex -= fvCount;

                ringVerts[ringSize++] = fVerts[fvIndex];
            }

            //  If last face of a boundary, append the trailing edge vertex:
            if (isBoundary && (faceInRing == (vFaces.size() - 1))) {
                int fvIndex = vInThisFace ? (vInThisFace - 1) : (fvCount - 1);

                ringVerts[ringSize++] = fVerts[fvIndex];
            }
        }
        return ringSize;
    }
}

bool
RefinerLimitSurfaceFactory::populateDescriptor(Index baseFace,
        ManifoldFaceDescriptor & desc) const {

    Vtr::internal::Level const & baseLevel = _mesh.getLevel(0);

    Vtr::internal::Level::VTag fTag = baseLevel.getFaceCompositeVTag(baseFace);
    ConstIndexArray fVerts          = baseLevel.getFaceVertices(baseFace);

    bool skipUnsupported = true;
    if (skipUnsupported) {
        bool reportUnsupportedFeature = false;
        if (fTag._incidIrregFace) {
            if (reportUnsupportedFeature)
                printf("Manifold descriptor:  incident-irregular, skipping.\n");
            return false;
        }
        if (fTag._semiSharp || fTag._semiSharpEdges) {
            if (reportUnsupportedFeature)
                printf("Manifold descriptor:  semi-sharp, skipping.\n");
            return false;
        }
        if (fTag._infSharp || fTag._infSharpEdges) {
            //
            //  WIP - this condition will skip boundary faces that were
            //  implicitly sharpened, both regular and irregular
            //
            if (reportUnsupportedFeature)
                printf("Manifold descriptor:  inf-sharp, skipping.\n");
            return false;
        }
        if (fTag._nonManifold) {
            if (reportUnsupportedFeature)
                printf("Manifold descriptor:  non-manifold, skipping.\n");
            return false;
        }
    }
bool debug= false;
if (debug) {
    printf("Manifold descriptor:  base face %d, %d corners, %s:\n",
        baseFace, fVerts.size(), fTag._boundary ? "BOUNDARY" : "interior");
}

    //
    //  Identify the vertices of the base face and initialize the Descriptor:
    //
    bool faceSizesAreConstant = !fTag._incidIrregFace;

    ConstIndexArray baseFVarValues;
    if (_populateFVarTopology) {
        baseFVarValues = baseLevel.getFaceFVarValues(baseFace, 0);
    }

    desc.Initialize(fVerts.size(), faceSizesAreConstant);

    for (int i = 0; i < fVerts.size(); ++i) {
        int vIndex = fVerts[i];

        ConstIndexArray vFaces = baseLevel.getVertexFaces(vIndex);

        desc.SetCornerNumIncidentFaces(i, vFaces.size());
        desc.SetCornerVertexIndex(i, vIndex);
        if (_populateFVarTopology) {
            desc.SetCornerFVarValueIndex(i, baseFVarValues[i]);
        }

        bool isBoundary = baseLevel.getVertexTag(vIndex)._boundary;
        if (isBoundary) {
            desc.SetCornerBoundary(i, vFaces.FindIndex(baseFace));
        }

        //  WIP - assign incident face sizes here in future
        assert(faceSizesAreConstant);

        Index * ringVerts = desc.AccessCornerRingVertexIndices(i);
        int     ringSize  = meshGatherVertexOneRing(
                                baseLevel, vIndex, ringVerts, baseFace);
if (debug) {
    printf("    corner %d (vertex %3d, boundary = %d) - 1-ring given:  ",
            i, vIndex, isBoundary);
    printIntArray(ringVerts, ringSize);
    printf("\n");
}
}
    desc.Finalize();

if (debug) {
    internal::ManifoldFaceBuilder * builderPtr =
        dynamic_cast<internal::ManifoldFaceBuilder*>(&desc);
    internal::ManifoldFaceBuilder & manBuilder = *builderPtr;
    //
    //  Gather the full one-ring of control vertex indices around the face:
    //      - remember to consider parametrizing such a search with a
    //        rotation, which may be simpler than storing intermediate
    //        members related to "local start" and "local size"
    //      - the control vertices are only gathered once, and that should
    //        be after a possible rotation has been determined
    //
    Vtr::internal::Level const & level0 = _mesh.getLevel(0);

    Vtr::internal::Level::VTag fTag   = level0.getFaceCompositeVTag(baseFace);
    ConstIndexArray            fVerts = level0.getFaceVertices(baseFace);

    int numCVs = manBuilder.GetNumControlVertices();
    int cvs[numCVs];
    for (int rotation = 0; rotation < fVerts.size(); ++rotation) {
        int n = manBuilder.GetControlVertexIndices(cvs, rotation);
        assert(n == numCVs);

        printf("    control vertices (size = %d, rot = %d) :   ", numCVs, rotation);
        printIntArray(cvs, numCVs);
        printf("\n");
    }

    bool faceSizesAreConstant = !fTag._incidIrregFace;
    assert(faceSizesAreConstant);

    int numCFaces = manBuilder.GetNumControlFaces();
    int numFVerts = manBuilder.GetNumControlFaceVertices();
    int faceSizes[numCFaces];
    int faceVerts[numFVerts];

    int n = manBuilder.GetLocalFaceVertices(faceVerts, faceSizes);
    assert(n == numFVerts);

    printf("    control face vertices (faces = %d):\n", numCFaces);
    for (int i = 0; i < numCFaces; ++i) {
        printf("        face %2d:  ", i);
        printIntArray(faceVerts + i*fVerts.size(), fVerts.size());
        printf("\n");
    }
}
    return true;
}

//
//  WIP -- the NonManifoldFaceDescriptor (and its NonManifoldFaceBuilder
//  subclass) was a complete but hasty and inefficient solution to
//  specifying the full face.  Both the NonManifoldFaceDescriptor and
//  the method here to populate it will be retired (though the name may
//  be reused for a completely different solution).
//
bool
RefinerLimitSurfaceFactory::populateDescriptor(Index baseFace,
        NonManifoldFaceDescriptor & desc) const {

    Far::TopologyRefiner const & baseRefiner = _mesh;

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

    //  Identify the unique list of faces involved (base face first):
    faceMap[baseFace] = 0;
    baseFaces.push_back(baseFace);

    ConstIndexArray baseFaceVerts = baseLevel.GetFaceVertices(baseFace);
    for (int i = 0; i < baseFaceVerts.size(); ++i) {
        ConstIndexArray vFaces = baseLevel.GetVertexFaces(baseFaceVerts[i]);

        for (int j = 0; j < vFaces.size(); ++j) {
            Index fIndex = vFaces[j];

            if (faceMap.find(fIndex) == faceMap.end()) {
                faceMap[fIndex] = (Index) faceMap.size();
                baseFaces.push_back(fIndex);
            }
        }
    }

    //  Traverse all faces, identify unique vertices while appending face-verts:
    desc.Initialize();

    std::vector<Index> & baseControlPoints = desc._controlVertices;

    std::vector<Index> & faceVerts = desc._data.faceVerts;
    std::vector<int>   & faceSizes = desc._data.faceSizes;

    baseControlPoints.clear();
    for (int i = 0; i < (int) baseFaces.size(); ++i) {
        Index           fIndex = baseFaces[i];
        ConstIndexArray fVerts = baseLevel.GetFaceVertices(fIndex);

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
    std::vector<float> & cornerSharpness = desc._data.cornerSharpness;
    std::vector<Index> & cornerIndices   = desc._data.cornerIndices;

    for (IndexMap::iterator vIt = vertMap.begin(); vIt != vertMap.end(); ++vIt) {
        Index vIndex = vIt->first;
        float vSharpness = baseLevel.GetVertexSharpness(vIndex);
        if (vSharpness > 0.0f) {
            cornerIndices.push_back(vIt->second);
            cornerSharpness.push_back(vSharpness);
        }
    }

    //  Assign any sharp edges (vertex pairs):
    std::vector<float> & creaseSharpness = desc._data.creaseSharpness;
    std::vector<Index> & creaseIndices   = desc._data.creaseIndices;

    for (int i = 0; i < baseFaceVerts.size(); ++i) {
        ConstIndexArray vEdges = baseLevel.GetVertexEdges(baseFaceVerts[i]);

        for (int j = 0; j < vEdges.size(); ++j) {
            Index eIndex = vEdges[j];
            float eSharpness = baseLevel.GetEdgeSharpness(eIndex);
            if ((eSharpness > 0.0f) && baseLevel.GetEdgeFaces(eIndex).size()) {
                ConstIndexArray eVerts = baseLevel.GetEdgeVertices(eIndex);

                creaseIndices.push_back(vertMap.find(eVerts[0])->second);
                creaseIndices.push_back(vertMap.find(eVerts[1])->second);
                creaseSharpness.push_back(eSharpness);
            }
        }
    }

    //  Optionally assign basic FVar values for the face:
    std::vector<Index> & fvarIndices = desc._data.faceFVarValues;

    if (_populateFVarTopology) {
        ConstIndexArray srcIndices = baseLevel.GetFaceFVarValues(baseFace, 0);

        int N = srcIndices.size();
        fvarIndices.resize(N);
        std::memcpy(&fvarIndices[0], &srcIndices[0], N * sizeof(int));
    }

    //  Finalize the General descriptor:
    desc.Finalize();

    return true;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
