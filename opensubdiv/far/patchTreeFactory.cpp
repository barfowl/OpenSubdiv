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

#include "../far/primvarRefiner.h"
#include "../far/topologyRefiner.h"
#include "../far/topologyDescriptor.h"
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

    PatchTreeBuilder(TopologyRefiner & refiner, Options options);
    ~PatchTreeBuilder();

    void IdentifyPatches();
    void InitializePatches();
    void InitializeStencilMatrix();
    void InitializeQuadTree();

    PatchTree * GetPatchTree() const { return _patchTree; }

private:
    struct PatchFace {
        PatchFace(int levelArg, int faceArg, bool isReg = true) :
                face(faceArg), level(levelArg), isRegular(isReg) { }

        int   face;
        short level;
        short isRegular;
    };

    //
    //  Internal helper functions to assign a full matrix of stencils
    //  converting points of irregular patches from source points in
    //  the refined levels:
    //
    template <typename REAL>
    void initializeStencilMatrix();

    template <typename REAL>
    void getIrregularPatchConversion(PatchFace const & patchFace,
                                     SparseMatrix<REAL> & convMatrix,
                                     std::vector<Index> & srcPoints);

    template <typename REAL>
    void appendConversionStencilsToMatrix(int stencilIndexBase,
                                          SparseMatrix<REAL> const & convMatrix,
                                          std::vector<Index> const & srcPoints);

private:
    //  The PatchTree instance being assembled:
    PatchTree * _patchTree;

    //  Member variables supporting its assembly:
    TopologyRefiner &         _faceRefiner;
    Index                     _faceAtRoot;
    std::vector<int>          _levelOffsets;
    PtexIndices               _ptexIndices;
    std::vector<PatchFace>    _patchFaces;
    PatchBuilder *            _patchBuilder;
};

PatchTreeBuilder::PatchTreeBuilder(TopologyRefiner & faceRefiner,
                                   Options options) :
    _patchTree(new PatchTree),
    _faceRefiner(faceRefiner),
    _faceAtRoot(0),
    _ptexIndices(faceRefiner),
    _patchBuilder(0) {

    //
    //  If generating patches for the base level, force one level of
    //  refinement if the face is or is adjacent to a non-quad:
    //
    Vtr::internal::Level const & baseLevel = _faceRefiner.getLevel(0);

    int adaptiveLevelPrimary = options.maxPatchDepthSharp;
    if (adaptiveLevelPrimary == 0) {
        //  Vertices incident non-quads are tagged, so inspect combined tags:
        if (baseLevel.getFaceCompositeVTag(_faceAtRoot)._incidIrregFace)
            adaptiveLevelPrimary = 1;
    }

    int adaptiveLevelSecondary = options.maxPatchDepthSmooth;
    if (adaptiveLevelSecondary > adaptiveLevelPrimary) {
        adaptiveLevelSecondary = adaptiveLevelPrimary;
    }

    //
    //  Apply adaptive refinement to a local refiner for this face:
    //
    ConstIndexArray baseFaceArray(&_faceAtRoot, 1);

    TopologyRefiner::AdaptiveOptions adaptiveOptions(adaptiveLevelPrimary);
    adaptiveOptions.secondaryLevel       = adaptiveLevelSecondary;
    adaptiveOptions.useInfSharpPatch     = true;
    adaptiveOptions.useSingleCreasePatch = false;
    adaptiveOptions.considerFVarChannels = false;

    _faceRefiner.RefineAdaptive(adaptiveOptions, baseFaceArray);

    //
    //  Determine offsets per level (we could eventually include local
    //  points in the levels in which the patch occurs)
    //
    int numLevels = _faceRefiner.GetNumLevels();
    _levelOffsets.resize(1 + numLevels);
    _levelOffsets[0] = 0;
    for (int i = 0; i < numLevels; ++i) {
        _levelOffsets[1 + i] = _levelOffsets[i]
                             + _faceRefiner.GetLevel(i).GetNumVertices();
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

    _patchBuilder = PatchBuilder::Create(faceRefiner, patchOptions);

    //
    //  Initialize general PatchTree members relating to patch topology:
    //
    int thisFaceSize = baseLevel.getFaceVertices(_faceAtRoot).size();
    int regFaceSize  = Sdc::SchemeTypeTraits::GetRegularFaceSize(
                                                _faceRefiner.GetSchemeType());

    //  Configuration:
    _patchTree->_useDoublePrecision = options.useDoublePrecision;

    _patchTree->_patchesIncludeNonLeaf = options.includeInteriorPatches;
    _patchTree->_patchesAreTriangular  = (regFaceSize == 3);

    _patchTree->_regPatchType   = _patchBuilder->GetRegularPatchType();
    _patchTree->_irregPatchType = _patchBuilder->GetIrregularPatchType();

    _patchTree->_regPatchSize =
        PatchDescriptor(_patchTree->_regPatchType).GetNumControlVertices();
    _patchTree->_irregPatchSize =
        PatchDescriptor(_patchTree->_irregPatchType).GetNumControlVertices();
    _patchTree->_patchPointStride =
        std::max(_patchTree->_regPatchSize, _patchTree->_irregPatchSize);

    //  Topology:
    _patchTree->_numSubFaces = (thisFaceSize == regFaceSize) ? 0 : thisFaceSize;

    _patchTree->_numControlPoints  = _faceRefiner.GetLevel(0).GetNumVertices();
    _patchTree->_numRefinedPoints  = _faceRefiner.GetNumVerticesTotal()
                                   - _patchTree->_numControlPoints;
    _patchTree->_numSubPatchPoints = _patchTree->_numRefinedPoints;
}

PatchTreeBuilder::~PatchTreeBuilder() {
    delete _patchBuilder;
}

void
PatchTreeBuilder::IdentifyPatches() {

    //
    //  Take inventory of the patches.  Only one face exists at the base
    //  level -- the root face.  Check all other levels breadth first:
    //
    bool incNonLeaf = _patchTree->_patchesIncludeNonLeaf;

    _patchFaces.clear();

    int numIrregPatches = 0;

    if (_patchBuilder->IsFaceAPatch(0, _faceAtRoot)) {
        if (incNonLeaf || _patchBuilder->IsFaceALeaf(0, _faceAtRoot)) {
            bool isRegular = _patchBuilder->IsPatchRegular(0, _faceAtRoot);
            _patchFaces.push_back(PatchFace(0, _faceAtRoot, isRegular));
            numIrregPatches += !isRegular;
        }
    }

    int numLevels = _faceRefiner.GetNumLevels();
    for (int lIndex = 1; lIndex < numLevels; ++lIndex) {
        int numFaces = _faceRefiner.getLevel(lIndex).getNumFaces();

        for (int fIndex = 0; fIndex < numFaces; ++fIndex) {
            if (_patchBuilder->IsFaceAPatch(lIndex, fIndex)) {
                if (incNonLeaf ||
                        _patchBuilder->IsFaceALeaf(lIndex, fIndex)) {
                    bool isReg = _patchBuilder->IsPatchRegular(lIndex, fIndex);
                    _patchFaces.push_back(PatchFace(lIndex, fIndex, isReg));
                    numIrregPatches += !isReg;
                }
            }
        }
    }

    //
    //  Allocate and populate the arrays of patch data for the identified
    //  patches:
    //
    int numPatches = (int) _patchFaces.size();

    _patchTree->_patchPoints.resize(numPatches * _patchTree->_patchPointStride);
    _patchTree->_patchParams.resize(numPatches);

    _patchTree->_numIrregPatches = numIrregPatches;

    _patchTree->_numSubPatchPoints += numIrregPatches *
                                      _patchTree->_irregPatchSize;
}

void
PatchTreeBuilder::InitializePatches() {

    //  Keep track of the growing index of local points in irregular patches:
    int irregPointIndexBase = _patchTree->_numControlPoints +
                              _patchTree->_numRefinedPoints;

    for (size_t i = 0; i < _patchFaces.size(); ++i) {
        PatchFace const & pf = _patchFaces[i];

        Index * patchPoints =
                &_patchTree->_patchPoints[i * _patchTree->_patchPointStride];

        if (pf.isRegular) {
            //  Determine boundary mask before computing/assigning PatchParam:
            int boundaryMask =
                _patchBuilder->GetRegularPatchBoundaryMask(pf.level, pf.face);

            _patchTree->_patchParams[i] = _patchBuilder->ComputePatchParam(
                pf.level, pf.face, _ptexIndices, true, boundaryMask, true);

            //  Gather the points of the patch -- since they are assigned
            //  directly into the PatchTree's buffer by the PatchBuilder
            //  here, they must be offset as a post-process:
            _patchBuilder->GetRegularPatchPoints(pf.level, pf.face,
                boundaryMask, patchPoints);

            for (int i = 0; i < _patchTree->_regPatchSize; ++i) {
                patchPoints[i] += _levelOffsets[pf.level];
            }
        } else {
            //  Compute/assign the PatchParam for an irregular patch:
            _patchTree->_patchParams[i] =
                _patchBuilder->ComputePatchParam(pf.level, pf.face,
                    _ptexIndices, false /*irreg*/, 0 /*mask*/, false);

            //  Assign indices of new/local points for this irregular patch:
            for (int i = 0; i < _patchTree->_irregPatchSize; ++i) {
                patchPoints[i] = irregPointIndexBase ++;
            }
        }
    }
}

//
//  Some local interpolatable types for combining stencil vectors -- the
//  rows of the stencil matrix:
//
namespace {
    //
    //  When accessing a "row" for a control point, the only non-zero
    //  entry is that at the index, with a value of 1, so just store
    //  that index so the StencilRows can combine it:
    //
    struct ControlRow {
        ControlRow(int index) : _index(index) { }
        ControlRow() { }

        ControlRow operator[] (int index) const {
            return ControlRow(index);
        }

        //  Members:
        int _index;
    };

    //
    //  A "row" for each stencil is just our typical vector of variable
    //  size that needs to support [].
    //
    //  For the first level, there are no source rows for the control
    //  points so combine with the proxy ControlRow defined above.  All
    //  other levels will accumulate StencilRows as weighted combinations
    //  of other StencilRows.
    //
    //  WIP - consider combining StencilRows to exploit SSE/AVZ vectorization
    //      - we can (in future) easily guarantee both are 4-word aligned
    //      - we can also pad the rows to a multiple of 4
    //      - prefer writing the combination in a portable way that makes
    //        use of auto-vectorization
    //
    template <typename REAL>
    struct StencilRow {
        StencilRow() : _data(0), _size(0) { }
        StencilRow(REAL * data, int size) :
                    _data(data), _size(size) { }
        StencilRow(REAL const * data, int size) :
                    _data(const_cast<REAL*>(data)), _size(size) { }

        void Clear() {
            for (int i = 0; i < _size; ++i) {
                _data[i] = 0.0f;
            }
        }

        void AddWithWeight(ControlRow const & src, REAL weight) {
            assert(src._index >= 0);
            _data[src._index] += weight;
        }

        void AddWithWeight(StencilRow const & src, REAL weight) {
            assert(src._size == _size);
            //  Weights passed here by PrimvarRefiner should be non-zero
            //  WIP - see note on potential/future auto-vectorization above
            for (int i = 0; i < _size; ++i) {
                _data[i] += weight * src._data[i];
            }
        }

        StencilRow operator[](int index) const {
            return StencilRow(_data + index * _size, _size);
        }

        //  Members:
        REAL * _data;
        int    _size;
    };
}

template <typename REAL>
void
PatchTreeBuilder::initializeStencilMatrix() {

    if (_patchTree->_numSubPatchPoints == 0) return;

    //
    //  Allocate and initialize a full matrix of true stencils (i.e.
    //  factored in terms of the control points):
    //
    int numPointStencils = _patchTree->_numRefinedPoints + 
                          (_patchTree->_numIrregPatches *
                           _patchTree->_irregPatchSize);
    int numControlPoints = _patchTree->_numControlPoints;

    std::vector<REAL> & stencilMatrix = _patchTree->getStencilMatrix<REAL>();

    stencilMatrix.resize(numPointStencils*numControlPoints);

    //
    //  Initialize successive rows of the stencil matrix a level at a
    //  time using the PrimvarRefiner to accumulate contributing rows:
    //
    Far::PrimvarRefinerReal<REAL> primvarRefiner(_faceRefiner);

    StencilRow<REAL> dstRow(&stencilMatrix[0], numControlPoints);
    primvarRefiner.Interpolate(1, ControlRow(-1), dstRow);

    int numLevels = _faceRefiner.GetNumLevels();
    for (int level = 2; level < numLevels; ++level) {
        StencilRow<REAL> srcRow = dstRow;
        dstRow = srcRow[_faceRefiner.getLevel(level-1).getNumVertices()];
        primvarRefiner.Interpolate(level, srcRow, dstRow);
    }

    //
    //  Now assign stencils for the points of any irregular patches:
    //
    if (_patchTree->_numIrregPatches) {
        SparseMatrix<REAL> irregConvMatrix;
        std::vector<Index> irregSourcePoints;

        int stencilIndexBase = _patchTree->_numRefinedPoints;

        for (size_t i = 0; i < _patchFaces.size(); ++i) {
            if (!_patchFaces[i].isRegular) {
                getIrregularPatchConversion(_patchFaces[i],
                        irregConvMatrix, irregSourcePoints);

                appendConversionStencilsToMatrix(stencilIndexBase,
                        irregConvMatrix,irregSourcePoints);

                stencilIndexBase += _patchTree->_irregPatchSize;
            }
        }
    }
}

template <typename REAL>
void
PatchTreeBuilder::appendConversionStencilsToMatrix(
        int                        stencilBaseIndex,
        SparseMatrix<REAL> const & conversionMatrix,
        std::vector<Index> const & sourcePoints) {

    //
    //  Each row of the sparse conversion matrix corresponds to a row
    //  of the stencil matrix -- which will be computed from the weights
    //  and indices of stencils indicated by the SparseMatrix row:
    //
    int numControlPoints = _patchTree->_numControlPoints;
    int numPatchPoints   = conversionMatrix.GetNumRows();

    std::vector<REAL> & stencilMatrix = _patchTree->getStencilMatrix<REAL>();

    StencilRow<REAL> srcStencils(&stencilMatrix[0], numControlPoints);
    StencilRow<REAL> dstStencils = srcStencils[stencilBaseIndex];

    for (int i = 0; i < numPatchPoints; ++i) {
        StencilRow<REAL> dstStencil = dstStencils[i];
        dstStencil.Clear();

        int  const * rowIndices = &conversionMatrix.GetRowColumns(i)[0];
        REAL const * rowWeights = &conversionMatrix.GetRowElements(i)[0];
        int          rowSize    =  conversionMatrix.GetRowSize(i);

        for (int j = 0; j < rowSize; ++j) {
            REAL srcWeight       = rowWeights[j];
            int  srcStencilIndex = sourcePoints[rowIndices[j]]
                                 - numControlPoints;

            StencilRow<REAL> srcStencil = srcStencils[srcStencilIndex];

            dstStencil.AddWithWeight(srcStencil, srcWeight);
        }
    }
}

void
PatchTreeBuilder::InitializeStencilMatrix() {

    if (_patchTree->_useDoublePrecision) {
        initializeStencilMatrix<double>();
    } else {
        initializeStencilMatrix<float>();
    }
}

void
PatchTreeBuilder::InitializeQuadTree() {

    _patchTree->buildQuadtree();
}

template <typename REAL>
void
PatchTreeBuilder::getIrregularPatchConversion(PatchFace const & pf,
    SparseMatrix<REAL> & conversionMatrix,
    std::vector<Index> & sourcePoints) {

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
            cornerSpans, conversionMatrix);

    //
    //  Identify the refined/source points for the patch and append stencils
    //  for the local patch points in terms of the source points:
    //
    int numSourcePoints = conversionMatrix.GetNumColumns();

    sourcePoints.resize(numSourcePoints);

    _patchBuilder->GetIrregularPatchSourcePoints(pf.level, pf.face,
                                                 cornerSpans, &sourcePoints[0]);

    int sourceIndexOffset = _levelOffsets[pf.level];
    for (int i = 0; i < numSourcePoints; ++i) {
        sourcePoints[i] += sourceIndexOffset;
    }
}

//
//  Public PatchTreeFactory method to create a PatchTree from a local
//  topology descriptor:
//
PatchTree *
PatchTreeFactory::Create(TopologyRefiner & faceRefiner,
                         Options options) {

    if (faceRefiner.GetNumLevels() > 1) faceRefiner.Unrefine();

    PatchTreeBuilder builder(faceRefiner, options);

    builder.IdentifyPatches();
    builder.InitializePatches();
    builder.InitializeStencilMatrix();
    builder.InitializeQuadTree();

    PatchTree * result = builder.GetPatchTree();

    faceRefiner.Unrefine();
    return result;
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
