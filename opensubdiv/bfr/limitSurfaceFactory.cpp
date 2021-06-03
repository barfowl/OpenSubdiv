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

#include "../far/patchTreeFactory.h"

#include "../bfr/refinerLimitSurfaceFactory.h"
#include "../bfr/limitSurface.h"
#include "../bfr/faceDescriptors.h"
#include "../bfr/faceBuilders.h"
#include "../bfr/topologyCache.h"

#include <map>
#include <cstdio>


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

using internal::RegularFaceBuilder;
using internal::ManifoldFaceBuilder;
using internal::NonManifoldFaceBuilder;

//
//  DEBUG - some static variables to keep track of a few things...
//
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
static int __numRegularPatches  = 0;
static int __numManifoldPatches = 0;
static int __numOtherPatches    = 0;
#endif

//
//  Main constructor and destructor -- an instance of Far::PatchBuilder
//  is useful as a member and requires some configuration to initialize
//  (though the execution cost of its construction is low)
//
LimitSurfaceFactory::LimitSurfaceFactory(
    Sdc::SchemeType schemeType,
    Sdc::Options    schemeOptions,
    Options         limitOptions,
    bool            supportsRegularDescriptors,
    bool            supportsManifoldDescriptors,
    bool            supportsNonManifoldDescriptors,
    int             numFaces) :
        _schemeType(schemeType),
        _schemeOptions(schemeOptions),
        _limitOptions(limitOptions),
        _supportsRegularDescriptors(supportsRegularDescriptors),
        _supportsManifoldDescriptors(supportsManifoldDescriptors),
        _supportsNonManifoldDescriptors(supportsNonManifoldDescriptors),
        _numFaces(numFaces),
        _topologyCache(0) {

    _regFaceSize = Sdc::SchemeTypeTraits::GetRegularFaceSize(_schemeType);

    //  Assign the topology cache -- externally or to an internal instance:
    if (_limitOptions.ExternalTopologyCache()) {
        _topologyCache = _limitOptions.ExternalTopologyCache();
    } else if (!_limitOptions.DisableTopologyCache()) {
        _topologyCache = new TopologyCache();
    }
}

LimitSurfaceFactory::~LimitSurfaceFactory() {

//  DEBUG - report and reset inventory:
bool debug = false;
if (debug) {
printf("LimitSurfaceFactory destructor:\n");
printf(    " _numFaces           = %6d\n", _numFaces);
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
printf(    "__numRegularPatches  = %6d\n", __numRegularPatches);
printf(    "__numManifoldPatches = %6d\n", __numManifoldPatches);
printf(    "__numOtherPatches    = %6d\n", __numOtherPatches);
#endif
if (_topologyCache) {
printf(    " _topologyCache size = %6d\n", (int) _topologyCache->Size());
} else {
printf(    " _topologyCache size = %6d (disabled)\n", 0);
}
}
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
__numRegularPatches  = 0;
__numManifoldPatches = 0;
__numOtherPatches    = 0;
#endif

    if (_limitOptions.ExternalTopologyCache() == 0) delete _topologyCache;
}


//
//  The "face has limit surface" query is a bit tricky to factor so that
//  clients can effectively support it for their meshes.  There are two
//  cases when a face will not have a limit surface:
//
//      - the face is a hole
//      - boundary interpolation option "none" is assigned:
//          - in which case some, not all, boundary faces have no limit
//
//  Dealing with holes is trivial.  But dealing with boundary faces when
// "boundary none" is set (not very often) is difficult.  It's not enough
//  to test if a face is a boundary -- if a boundary face has all of its
//  incident boundary edges (i.e. all boundary edges incident to all of
//  its face-vertices) then the boundary face has a limit surface.
//
//  In the spirit of using FaceDescriptors, a BoundaryFaceDesriptor could
//  be required and populated for testing -- only when the "boundary none"
//  options is assigned (again, not often).
//
//  And remember, that we want to test boundary edge sharpness before any
//  explicit sharpening of edges may have occurred.  Such is the case
//  with the Far::TopologyRefiner, which will have sharpened all boundary
//  edges, but thankfully sets the hole tag to meet these needs.
//
bool
LimitSurfaceFactory::FaceHasLimitSurface(Index face) const {

    if (isFaceHole(face)) {
        return false;
    }
    if (_schemeOptions.GetVtxBoundaryInterpolation() ==
            Sdc::Options::VTX_BOUNDARY_NONE) {
        //  Unclear what to do here (see note above)...
    }
    return true;
}


//
//  Methods supporting construction of regular and irregular patches:
//
bool
LimitSurfaceFactory::assignRegularPatch(LimitSurface::Evaluator & eval,
        RegularFaceBuilder const & builder) const {

    assert(builder.IsFinalized());

    //  Initialize the PatchParam for repeated use in evaluation (argument
    //  order is:  face, u, v, depth, non-quad, boundary, trans, regular):
    int patchSize = builder.GetPatchSize();

    eval._isRegular = true;
    eval._isLinear  = (patchSize > 4);
    eval._isCached  = false;

    eval._regPatchType = builder.GetPatchType();
    eval._regPatchParam.Set(0, 0, 0, 0, 0, builder.GetBoundaryMask(), 0, true);

    //  Assign the control points from the regular patch points:
    eval._controlPoints.SetSize(patchSize);
    builder.GetPatchPointIndices(&eval._controlPoints[0]);

    eval._numControlPoints = patchSize;
    eval._numPatchPoints   = patchSize;

    return true;
}

template <class BUILDER_TYPE>
bool
LimitSurfaceFactory::assignLinearPatch(LimitSurface::Evaluator & eval,
        BUILDER_TYPE const & builder, bool useFaceVaryingIndices) const {

    assert(builder.IsFinalized());

    //  Cannot construct a face-varying evaluator when no FVar values:
    if (useFaceVaryingIndices && !builder.HasFVarIndices()) {
        return false;
    }

    //  Initialize instance members from the associated irregular patch:
    int faceSize = builder.GetFaceSize();

    eval._isRegular = (faceSize == _regFaceSize);
    eval._isLinear  = true;
    eval._isCached  = false;

    eval._regPatchType = (_regFaceSize == 4)
                       ?  Far::PatchDescriptor::QUADS
                       :  Far::PatchDescriptor::TRIANGLES;
    eval._regPatchParam.Clear();

    //  Assign control points from face-vertices of the descriptor/builder:
    eval._controlPoints.SetSize(faceSize);
    if (useFaceVaryingIndices) {
        builder.GetFaceFVarValueIndices(&eval._controlPoints[0]);
    } else {
        builder.GetFaceVertexIndices(&eval._controlPoints[0]);
    }

    eval._numControlPoints = faceSize;
    eval._numPatchPoints   = faceSize;
    return true;
}

namespace {
    Far::PatchTreeFactory::Options
    getIrregPatchOptions(Bfr::LimitSurfaceFactory::Options limitOptions) {

        Far::PatchTreeFactory::Options patchTreeOptions;
        patchTreeOptions.maxPatchDepthSharp = limitOptions.MaxLevelPrimary();
        patchTreeOptions.maxPatchDepthSmooth = limitOptions.MaxLevelSecondary();
        patchTreeOptions.includeInteriorPatches = false;

        return patchTreeOptions;
    }

    inline Far::PatchTree*
    createIrregPatch(ManifoldFaceBuilder const & builder,
                     Sdc::SchemeType schemeType, Sdc::Options schemeOptions,
                     Bfr::LimitSurfaceFactory::Options limitOptions) {

        int vertCount  = builder.GetNumControlVertices();
        int faceCount  = builder.GetNumControlFaces();
        int fVertCount = builder.GetNumControlFaceVertices();

        Vtr::internal::StackBuffer<int, 64,true> faceSizes(faceCount);
        Vtr::internal::StackBuffer<int,256,true> faceVerts(fVertCount);

        builder.GetLocalFaceVertices(faceVerts, faceSizes);

        Far::TopologyDescriptor farTopology;

        farTopology.numVertices = vertCount;
        farTopology.numFaces    = faceCount;

        farTopology.numVertsPerFace    = faceSizes;
        farTopology.vertIndicesPerFace = faceVerts;

        //  Construct a TopologyRefiner in order to create a PatchTree:
        Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Options topOptions;
        topOptions.schemeType = schemeType;
        topOptions.schemeOptions = schemeOptions;
        topOptions.validateFullTopology = true;  // WIP - remove when stable

        Far::TopologyRefiner * refiner =
            Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Create(
                farTopology, topOptions);

        Far::PatchTree *patchTree = Far::PatchTreeFactory::Create(
                *refiner, getIrregPatchOptions(limitOptions));

        delete refiner;
        return patchTree;
    }

    inline Far::PatchTree*
    createIrregPatch(NonManifoldFaceBuilder const & builder,
                     Sdc::SchemeType schemeType, Sdc::Options schemeOptions,
                     Bfr::LimitSurfaceFactory::Options limitOptions) {

        //  Construct a TopologyRefiner in order to create a PatchTree:
        Far::TopologyRefiner * refiner =
            Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Create(
                builder._topology,
                Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Options(
                    schemeType, schemeOptions));

        Far::PatchTree *patchTree = Far::PatchTreeFactory::Create(
                *refiner, getIrregPatchOptions(limitOptions));

        delete refiner;
        return patchTree;
    }

/*
    //
    //  May still be of historical interest -- computing a TopologyKey from
    //  the contents of the TopologyRefiner's base level rather than from a
    //  FaceDescriptor...
    //
    TopologyCache::Key
    createTopologyKey(Far::TopologyRefiner const & mesh, Index face) {

        Vtr::internal::Level const & baseLevel = mesh.getLevel(0);
        Vtr::internal::Level::VTag fTag = baseLevel.getFaceCompositeVTag(face);

        //  Currently we only hash/cache manifold interior faces:
        TopologyCache::Key key;
        if (fTag._boundary || fTag._nonManifold || fTag._incidIrregFace ||
            fTag._semiSharp || fTag._semiSharpEdges ||
            fTag._infSharp || fTag._infSharpEdges) {
            key.hashBits = 0;
        } else {
            ConstIndexArray fVerts = baseLevel.getFaceVertices(face);
            assert(fVerts.size() <= 4);

            key.hashBits  = baseLevel.getVertexFaces(fVerts[0]).size();
            key.hashBits |= baseLevel.getVertexFaces(fVerts[1]).size() <<  8;
            key.hashBits |= baseLevel.getVertexFaces(fVerts[2]).size() << 16;
            if (fVerts.size() == 4) {
                key.hashBits |= baseLevel.getVertexFaces(fVerts[3]).size() << 24;
            }
        }
        return key;
    }
*/
}

template <class BUILDER_TYPE>
bool
LimitSurfaceFactory::assignIrregularPatch(LimitSurface::Evaluator & eval,
        BUILDER_TYPE const & builder) const {

    Far::PatchTree const * patch = 0;
    bool                   patchIsCached = false;
    TopologyCache::Key     patchKey;

    //  Try to retrieve the patch from the cache:
    if (_topologyCache) {
        patchKey = builder.ComputeTopologyKey();
        if (patchKey.IsValid()) {
            patch = _topologyCache->Find(patchKey);
            patchIsCached = (patch != 0);
        }
    }

    //  Create a new patch (no cache, no valid key or not cached):
    if (patch == 0) {
        patch = createIrregPatch(builder, _schemeType, _schemeOptions, _limitOptions);
    }

    //  Update the cache with the new patch when necessary:
    if (patchKey.IsValid() && !patchIsCached) {
        //  Beware the race condition when adding to the cache:
        Far::PatchTree const * patchAdded = _topologyCache->Add(patchKey, patch);
        if (patchAdded != patch) {
            delete patch;
            patch = patchAdded;
        }
        patchIsCached = true;
    }

    //  Initialize instance members from the associated irregular patch:
    eval._isRegular = false;

    eval._irregPatch = patch;
    eval._isCached   = patchIsCached;

    //  Assign the control points from the descriptor/builder:
    assert(patch->GetNumControlPoints() == builder.GetNumControlVertices());
    eval._controlPoints.SetSize(builder.GetNumControlVertices());
    builder.GetControlVertexIndices(&eval._controlPoints[0]);

    eval._numControlPoints = patch->GetNumControlPoints();
    eval._numPatchPoints   = patch->GetNumPointsTotal();

    return true;
}

void
LimitSurfaceFactory::buildSurface(LimitSurface & s,
        RegularFaceBuilder & regBuilder) const {

    //
    //  Assign vertex, varying and/or face-varying Evaluators:
    //
    s.parameterize(Parameterization(_schemeType, _regFaceSize));

    if (_limitOptions.CreateVertexEvaluators()) {
        assignRegularPatch(s._vtxEval, regBuilder);
    }
    if (_limitOptions.CreateVaryingEvaluators()) {
        assignLinearPatch(s._varEval, regBuilder);
    }
    if (_limitOptions.CreateFVarEvaluators()) {
        //  WIP - need to support non-linear FVar patch in future,
        //        which may not be regular (!)
        assignLinearPatch(s._fvarEval, regBuilder, true);
    }
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
    __numRegularPatches++;
#endif
}

void
LimitSurfaceFactory::buildSurface(LimitSurface & s,
        ManifoldFaceBuilder & manBuilder) const {

    if (!_supportsRegularDescriptors) {
        RegularFaceBuilder regBuilder;
        if (manBuilder.IsRegular(regBuilder)) {
            buildSurface(s, regBuilder);
            return;
        }
    }

    //
    //  Assign vertex, varying and/or face-varying Evaluators:
    //
    s.parameterize(Parameterization(_schemeType, manBuilder.GetFaceSize()));

    if (_limitOptions.CreateVertexEvaluators()) {
        assignIrregularPatch(s._vtxEval, manBuilder);
    }
    if (_limitOptions.CreateVaryingEvaluators()) {
        assignLinearPatch(s._varEval, manBuilder);
    }
    if (_limitOptions.CreateFVarEvaluators()) {
        //  WIP - need to support non-linear FVar patch in future
        assignLinearPatch(s._fvarEval, manBuilder, true);
    }
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
    __numManifoldPatches++;
#endif
}

void
LimitSurfaceFactory::buildSurface(LimitSurface & s,
        NonManifoldFaceBuilder & nonmanBuilder) const {

    //  WIP - future plans...
    bool manifoldConversionSupported = false;
    if (manifoldConversionSupported) {
        ManifoldFaceBuilder manBuilder;
        nonmanBuilder.GetManifoldSubset(manBuilder);

        RegularFaceBuilder regBuilder;
        if (manBuilder.IsRegular(regBuilder)) {
            buildSurface(s, regBuilder);
        } else {
            buildSurface(s, manBuilder);
        }
        return;
    }

    //
    //  Assign vertex, varying and/or face-varying Evaluators:
    //
    s.parameterize(Parameterization(_schemeType, nonmanBuilder.GetFaceSize()));

    if (_limitOptions.CreateVertexEvaluators()) {
        assignIrregularPatch(s._vtxEval, nonmanBuilder);
    }
    if (_limitOptions.CreateVaryingEvaluators()) {
        assignLinearPatch(s._varEval, nonmanBuilder);
    }
    if (_limitOptions.CreateFVarEvaluators()) {
        //  WIP - need to support non-linear FVar patch in future
        assignLinearPatch(s._fvarEval, nonmanBuilder, true);
    }
#ifdef _BFR_DEBUG_TOP_TYPE_STATS
    __numOtherPatches++;
#endif
}

bool
LimitSurfaceFactory::Populate(LimitSurface & s, Index baseFace) const {

    //
    //  Clear and re-initialize the existing instance before re-populating.
    //  If the given face does not have a limit surface, it will not be
    //  parameterized and so can be detected as invalid:
    //
    s.clear();
    s.initialize();

    s._faceIndex = baseFace;

    if (!FaceHasLimitSurface(baseFace)) return false;

    //
    //  Three Descriptor types may be supported and are executed in an order
    //  that favors user optimizations for regular and manifold topology.
    //  If the face topology does not support a more specific topology, it
    //  will fail and defer to one less specific.
    //
    if (_supportsRegularDescriptors) {
        RegularFaceBuilder regBuilder;
        if (populateDescriptor(baseFace, regBuilder)) {
            buildSurface(s, regBuilder);
            return true;
        }
    }
    if (_supportsManifoldDescriptors) {
        ManifoldFaceBuilder manBuilder;
        if (populateDescriptor(baseFace, manBuilder)) {
            buildSurface(s, manBuilder);
            return true;
        }
    }
    if (_supportsNonManifoldDescriptors) {
        NonManifoldFaceBuilder nonmanBuilder;
        if (populateDescriptor(baseFace, nonmanBuilder)) {
            buildSurface(s, nonmanBuilder);
            return true;
        }
    }
    return false;
}

LimitSurface *
LimitSurfaceFactory::Create(Index baseFace) const {

    //
    //  Avoid allocation if face has no limit surface (hole):
    //
    if (FaceHasLimitSurface(baseFace)) {
        LimitSurface * limitSurface = new LimitSurface();

        Populate(*limitSurface, baseFace);
        assert(limitSurface->IsValid());

        return limitSurface;
    }
    return 0;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
