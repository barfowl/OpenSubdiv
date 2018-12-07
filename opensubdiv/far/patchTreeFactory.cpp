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
#include "../far/stencilTableFactory.h"
#include "../far/patchTreeFactory.h"
#include "../far/patchBuilder.h"
#include "../far/sparseMatrix.h"
#include "../vtr/stackBuffer.h"

#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Far {

using Vtr::internal::Level;
using Vtr::internal::StackBuffer;


//
//  Simple PatchTreeBuilder class to maintain the state and a few methods
//  needed to assemble the PatchTree:
//
class PatchTreeBuilder {
public:
    //
    //  Public interface intended for use by the PatchTreeFactory -- all
    //  else is solely for internal use:
    //
    typedef PatchTreeFactory::Options Options;

    PatchTreeBuilder(TopologyRefiner const & refiner, Index face, Options options);
    ~PatchTreeBuilder();

    void InitializePatches();
    void InitializeStencils();
    void InitializeQuadTree();

    PatchTree * GetPatchTree() const { return _patchTree; }

private:
    struct PatchFace {
        PatchFace(int levelArg, int faceArg) : level(levelArg), face(faceArg) { }

        int level;
        int face;
    };

    void assignRegularPatch(PatchFace const & pf, int patchIndex);
    void assignIrregularPatch(PatchFace const & pf, int patchIndex);

    void appendLocalPointStencils(SparseMatrix<float> const & conversionMatrix,
                                  Index const sourcePoints[], int sourcePointOffset);

private:
    //  The PatchTree instance being assembled:
    PatchTree * _patchTree;

    //  Member variables supporing its assembly:
    TopologyRefiner *         _faceRefiner;
    Index                     _faceAtRoot;
    std::vector<int>          _levelOffsets;
    PtexIndices               _ptexIndices;
    PatchBuilder *            _patchBuilder;
    SparseMatrix<float>       _conversionMatrix;
    StencilTableReal<float> * _localPointStencils;
};

PatchTreeBuilder::PatchTreeBuilder(TopologyRefiner const & refiner, Index face, Options options) :
    _patchTree(new PatchTree),
    _faceRefiner(0),
    _faceAtRoot(face),
    _ptexIndices(refiner),
    _patchBuilder(0),
    _localPointStencils(0) {

    //
    //  Apply adaptive refinement to a local refiner for this face:
    //
    TopologyRefiner::AdaptiveOptions adaptiveOptions(options.maxPatchDepth);
    adaptiveOptions.useInfSharpPatch     = true;
    adaptiveOptions.useSingleCreasePatch = false;
    adaptiveOptions.considerFVarChannels = false;

    _faceRefiner = TopologyRefinerFactory<TopologyDescriptor>::Create(refiner);
    _faceRefiner->RefineAdaptive(adaptiveOptions, ConstIndexArray(&_faceAtRoot, 1));

    //
    //  Determine offsets per level (we could eventually include local points in
    //  the levels in which the patch occurs)
    //
    int numLevels = _faceRefiner->GetNumLevels();
    _levelOffsets.resize(1 + numLevels);
    _levelOffsets[0] = 0;
    for (int i = 0; i < numLevels; ++i) {
        _levelOffsets[1 + i] = _levelOffsets[i] + _faceRefiner->GetLevel(i).GetNumVertices();
    }

    //
    //  Create a PatchBuilder for this refiner:
    //
    PatchBuilder::BasisType patchBuilderIrregularBasis;
    if (options.irregularBasis == Options::REGULAR) {
        patchBuilderIrregularBasis = PatchBuilder::BASIS_REGULAR;
    } else if (options.irregularBasis == Options::LINEAR) {
        patchBuilderIrregularBasis = PatchBuilder::BASIS_LINEAR;
    } else {
        patchBuilderIrregularBasis = PatchBuilder::BASIS_GREGORY;
    }

    PatchBuilder::Options patchOptions;
    patchOptions.regBasisType                = PatchBuilder::BASIS_REGULAR;
    patchOptions.irregBasisType              = patchBuilderIrregularBasis;
    patchOptions.approxInfSharpWithSmooth    = false;
    patchOptions.approxSmoothCornerWithSharp = false;
    patchOptions.fillMissingBoundaryPoints   = true;

    _patchBuilder = PatchBuilder::Create(*_faceRefiner, patchOptions);

    //
    //  Initialize general PatchTree members relating to patch topology:
    //
    _patchTree->_numControlPoints  = _faceRefiner->GetLevel(0).GetNumVertices();
    _patchTree->_numSubPatchPoints = _faceRefiner->GetNumVerticesTotal()
                                   - _patchTree->_numControlPoints;

    _patchTree->_patchesAreTriangular = (_patchBuilder->GetRegularFaceSize() == 3);

    _patchTree->_regPatchType = _patchBuilder->GetRegularPatchType();
    _patchTree->_regPatchSize =
        PatchDescriptor(_patchTree->_regPatchType).GetNumControlVertices();

    _patchTree->_irregPatchType = _patchBuilder->GetIrregularPatchType();
    _patchTree->_irregPatchSize =
        PatchDescriptor(_patchTree->_irregPatchType).GetNumControlVertices();

    _patchTree->_patchPointStride =
        std::max(_patchTree->_regPatchSize, _patchTree->_irregPatchSize);
}

PatchTreeBuilder::~PatchTreeBuilder() {
    delete _faceRefiner;
    delete _patchBuilder;
    delete _localPointStencils;
}

void
PatchTreeBuilder::InitializePatches() {

    //
    //  Take inventory of the patches.  Only one face exists at the base level --
    //  the root face.  Check all other levels breadth first:
    //
    std::vector<PatchFace> patchFaces;

    if (_patchBuilder->IsFaceAPatch(0, _faceAtRoot)) {
        patchFaces.push_back(PatchFace(0, _faceAtRoot));
    }
    for (int levelIndex = 1; levelIndex < _faceRefiner->GetNumLevels(); ++levelIndex) {
        int numFaces = _faceRefiner->getLevel(levelIndex).getNumFaces();

        for (int faceIndex = 0; faceIndex < numFaces; ++faceIndex) {
            if (_patchBuilder->IsFaceAPatch(levelIndex, faceIndex)) {
                patchFaces.push_back(PatchFace(levelIndex, faceIndex));
            }
        }
    }

    //
    //  Allocate and populate the arrays of patch data for the identified patches:
    //
    int numPatches = (int) patchFaces.size();

    _patchTree->_patchPoints.resize(numPatches * _patchTree->_patchPointStride);
    _patchTree->_patchParams.resize(numPatches);

    for (int i = 0; i < numPatches; ++i) {
        PatchFace const & pf = patchFaces[i];

        bool isRegular = _patchBuilder->IsPatchRegular(pf.level, pf.face);
        if (isRegular) {
            assignRegularPatch(pf, i);
        } else {
            assignIrregularPatch(pf, i);
        }

        //  Annoying/temporary workaround for the "isRegular" bit -- which is not
        //  set via the common PatchParam construction methods:
        PatchParam & param = _patchTree->_patchParams[i];
        param.Set(param.GetFaceId(), param.GetU(), param.GetV(), param.GetDepth(),
                  param.NonQuadRoot(), param.GetBoundary(), param.GetTransition(),
                  isRegular);
    }
}

void
PatchTreeBuilder::InitializeStencils() {

    if (_patchTree->_numSubPatchPoints == 0) return;

    //  Create factorized stencils for all refined points using the Factory
    StencilTableFactoryReal<float>::Options stencilOptions;
    stencilOptions.generateOffsets = true;
    stencilOptions.generateControlVerts = true;
    stencilOptions.generateIntermediateLevels = true;
    stencilOptions.factorizeIntermediateLevels = true;

    StencilTableReal<float> const * refinedPointStencils =
            StencilTableFactoryReal<float>::Create(*_faceRefiner, stencilOptions);

    //  Assign the refined point stencils or append local point stencils:
    if (_localPointStencils == 0) {
        _patchTree->_stencilTable = refinedPointStencils;
    } else {
        _localPointStencils->generateOffsets();

        _patchTree->_stencilTable = 
            StencilTableFactoryReal<float>::AppendLocalPointStencilTable(
                *_faceRefiner, refinedPointStencils, _localPointStencils, true);
        delete refinedPointStencils;
    }
    assert(_patchTree->_stencilTable->GetNumStencils() ==
          (_patchTree->_numControlPoints + _patchTree->_numSubPatchPoints));

    //  Would like to prune the first N control point stencils from the table --
    //  is there any reason form them to be present?
}

void
PatchTreeBuilder::InitializeQuadTree() {

    _patchTree->buildQuadtree();
}

void
PatchTreeBuilder::assignRegularPatch(PatchFace const & pf, int patchIndex) {

    //
    //  The topology of a regular patch is solely determined by its boundary mask:
    //
    int boundaryMask = _patchBuilder->GetRegularPatchBoundaryMask(pf.level, pf.face);

    //
    //  Gather the points of the patch -- since we load them directly into the
    //  PatchTree's buffer, we need to offset the indices as a post-process:
    //
    Index * patchPoints = &_patchTree->_patchPoints[patchIndex * _patchTree->_patchPointStride];

    int numPatchPoints = _patchBuilder->GetRegularPatchPoints(pf.level, pf.face,
             boundaryMask, patchPoints);

    int pointIndexOffset = _levelOffsets[pf.level];
    for (int i = 0; i < numPatchPoints; ++i) {
        patchPoints[i] += pointIndexOffset;
    }

    //  Support for single-crease patch would go here -- simple matter to query
    //  the PatchBuilder and replace the boundary mask with the sharp edge mask

    //  Compute the PatchParam for a regular patch:
    _patchTree->_patchParams[patchIndex] = _patchBuilder->ComputePatchParam(
            pf.level, pf.face, _ptexIndices, boundaryMask, true);
}

void
PatchTreeBuilder::assignIrregularPatch(PatchFace const & pf, int patchIndex) {

    //
    //  The topology of an irregular patch is determined by its four corners:
    //
    Level::VSpan cornerSpans[4];
    _patchBuilder->GetIrregularPatchCornerSpans(pf.level, pf.face, cornerSpans);

    //
    //  Compute the conversion matrix from refined/source points to the
    //  set of points local to this patch:
    //
    _patchBuilder->GetIrregularPatchConversionMatrix(pf.level, pf.face,
            cornerSpans, _conversionMatrix);

    //
    //  Assign indices of new points for this irregular patch -- the starting
    //  index is the sum of refined and local patch points generated thus far:
    //
    Index *patchPoints = &_patchTree->_patchPoints[patchIndex * _patchTree->_patchPointStride];

    int numPatchPoints = _conversionMatrix.GetNumRows();

    int pointIndexBase = _patchTree->_numControlPoints + _patchTree->_numSubPatchPoints;
    for (int i = 0; i < numPatchPoints; ++i) {
        patchPoints[i] = pointIndexBase + i;
    }
    _patchTree->_numSubPatchPoints += numPatchPoints;

    //
    //  Identify the refined/source points for the patch and append stencils
    //  for the local patch points in terms of the source points:
    //
    int numSourcePoints = _conversionMatrix.GetNumColumns();

    StackBuffer<Index,64,true> sourcePoints(numSourcePoints);

    _patchBuilder->GetIrregularPatchSourcePoints(pf.level, pf.face, cornerSpans, sourcePoints);

    int sourceIndexOffset = _levelOffsets[pf.level];
    appendLocalPointStencils(_conversionMatrix, sourcePoints, sourceIndexOffset);

    //  Compute the PatchParam for an irregular patch:
    _patchTree->_patchParams[patchIndex] = _patchBuilder->ComputePatchParam(
            pf.level, pf.face, _ptexIndices, 0, false);
}

void
PatchTreeBuilder::appendLocalPointStencils(
    SparseMatrix<float> const & conversionMatrix,
    Index const                 sourcePoints[],
    int                         sourcePointOffset) {

    if (_localPointStencils == 0) {
        _localPointStencils = new StencilTableReal<float>(0);
    }

    //
    //  This is where we need protected access to the StencilTable as it lacks
    //  any public modifiers.  This is essentially the same method used by the
    //  PatchTableBuilder to append a SparseMatrix and set of associated column
    //  indices to a StencilTable.
    //
    //  Resize the StencilTable members to accomodate all rows and elements from
    //  the given set of points represented by the matrix
    //
    StencilTableReal<float>* stencilTable = _localPointStencils;

    int numNewStencils = conversionMatrix.GetNumRows();
    int numNewElements = conversionMatrix.GetNumElements();

    size_t numOldStencils = stencilTable->_sizes.size();
    size_t numOldElements = stencilTable->_indices.size();

    //  Assign the sizes for the new stencils:
    stencilTable->_sizes.resize(numOldStencils + numNewStencils);

    int * newSizes = &stencilTable->_sizes[numOldStencils];
    for (int i = 0; i < numNewStencils; ++i) {
        newSizes[i] = conversionMatrix.GetRowSize(i);
    }

    //  Assign remapped indices for the stencils:
    stencilTable->_indices.resize(numOldElements + numNewElements);

    int const * mtxIndices = &conversionMatrix.GetColumns()[0];
    int *       newIndices = &stencilTable->_indices[numOldElements];

    for (int i = 0; i < numNewElements; ++i) {
        newIndices[i] = sourcePoints[mtxIndices[i]] + sourcePointOffset;
    }

    //  Copy the stencil weights direct from the matrix elements:
    stencilTable->_weights.resize(numOldElements + numNewElements);

    float const * mtxWeights = &conversionMatrix.GetElements()[0];
    float *       newWeights = &stencilTable->_weights[numOldElements];

    std::memcpy(newWeights, mtxWeights, numNewElements * sizeof(float));
}


//
//  Public PatchTreeFactory method to create a PatchTree from a local
//  topology descriptor:
//
PatchTree *
PatchTreeFactory::Create(PatchTree::Descriptor const & descriptor,
                         Options options) {

    Far::TopologyRefiner * refiner =
        Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Create(
                descriptor.topology,
                Far::TopologyRefinerFactory<Far::TopologyDescriptor>::Options(
                    descriptor.schemeType,
                    descriptor.schemeOptions));

    PatchTreeBuilder builder(*refiner, 0, options);

    builder.InitializePatches();
    builder.InitializeStencils();
    builder.InitializeQuadTree();

    delete refiner;

    return builder.GetPatchTree();
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
