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

#include "../far/patchTree.h"
#include "../far/patchBasis.h"
#include "../far/stencilTable.h"

#include "../bfr/limitSurface.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {


//
//  LimitSurface::Evaluator methods to use for re-initialization:
//
void
LimitSurface::Evaluator::initialize() {

    _numControlPoints = 0;
    _numPatchPoints   = 0;

    _isValid   = false;
    _isRegular = true;
    _isLinear  = false;
    _isCached  = false;

    _irregPatch = 0;
}

void
LimitSurface::Evaluator::clear() {

    if (_irregPatch && !_isCached) delete _irregPatch;
}

//
//  LimitSurface constructor and destructor:
//
LimitSurface::LimitSurface() {

    initialize(0);
}

LimitSurface::~LimitSurface() {

    clear();
}

void
LimitSurface::initialize(int numFVarEvaluators) {

    _faceIndex = -1;

    if (_vtxEval.isValid()) _vtxEval.initialize();
    if (_varEval.isValid()) _varEval.initialize();

    if (numFVarEvaluators == (int)_fvarEval.GetSize()) {
        for (int i = 0; i < numFVarEvaluators; ++i) {
            if (_fvarEval[i].isValid()) _fvarEval[i].initialize();
        }
    } else {
        _fvarEval.SetSize(numFVarEvaluators);
    }
}

void
LimitSurface::parameterize(Parameterization const & param) {

    _param = param;
}

void
LimitSurface::clear() {

    if (_vtxEval.isValid()) _vtxEval.clear();
    if (_varEval.isValid()) _varEval.clear();

    for (int i = 0; i < (int)_fvarEval.GetSize(); ++i) {
        if (_fvarEval[i].isValid()) _fvarEval[i].clear();
    }
}

//
//  Evaluation methods accessing the local data for a simple regular patch:
//
void
Evaluator::evalRegularPatchBasis(float u, float v,
        float wP[], float wDu[], float wDv[]) const {

    Far::internal::EvaluatePatchBasisNormalized(
            _regPatchType, _regPatchParam, u, v, wP, wDu, wDv);
}

//
//  Evaluation methods accessing the Far::PatchTree for irregular patches:
//
ConstIndexArray
Evaluator::evalIrregularPatchBasis(float u, float v,
        float wP[], float wDu[], float wDv[]) const {

    int subFace = 0;
    if (_param.GetType() == Parameterization::QPOLY) {
        subFace = _param.ConvertQPolyUVToNormalizedSubQuad(u, v, u, v);
    }

    int subPatchIndex = _irregPatch->FindSubPatch(u, v, subFace);
    assert(subPatchIndex >= 0);

    _irregPatch->EvalSubPatchBasis(subPatchIndex, u, v, wP, wDu, wDv);

    return _irregPatch->GetSubPatchPoints(subPatchIndex);
}

//
//  Evaluation methods for the N-sided quadrangulated linear patch:
//
//  Rather than computing a set of weights for potentially large N, basis
//  evaluation computes basis functions on the containing bilinear sub-face
//  (only 4 weights) and transforms them to a set of 4 unique weights to
//  be used for all N base points.
//
namespace {
    //
    //  Regardless of N, there are four unique weights derived from the
    //  four bilinear weights of the sub-face.  Given these weights as
    //  input for a sub-face with origin at base point P, the resulting
    //  weights are associated with the N base points as follows:
    //
    //      w[0] = the point at the origin (P)
    //      w[1] = the point following P
    //      w[2] = the N-3 points not adjacent to P (contributing to center)
    //      w[3] = the point preceding P
    //
    inline void
    transformSubFaceWeightsToBase(int N, float w[4]) {

        float wOrigin = w[0];
        float wNext   = w[1] * 0.5f;
        float wCenter = w[2] / (float)N;
        float wPrev   = w[3] * 0.5f;

        w[0] = wCenter + wNext + wPrev + wOrigin;
        w[1] = wCenter + wNext;
        w[2] = wCenter;
        w[3] = wCenter + wPrev;
    }
}

int
Evaluator::evalMultiLinearPatchBasis(float u, float v,
        float wP[4], float wDu[4], float wDv[4]) const {

    assert(_param.GetType() == Parameterization::QPOLY);

    int subFace = _param.ConvertQPolyUVToNormalizedSubQuad(u, v, u, v);

    //  WIP - Prefer to eval Linear basis directly, i.e.:
    //
    //      Far::internal::EvalBasisLinear(u, v, wP, wDu, wDv);
    //
    //  but this internal Far function is sometimes optimized out, causing
    //  link errors.  Need to fix in Far with explicit instantiation...
    Far::internal::EvaluatePatchBasisNormalized(
        Far::PatchDescriptor::QUADS, Far::PatchParam(), u, v, wP, wDu, wDv);

    transformSubFaceWeightsToBase(_numControlPoints, wP);
    if (wDu) transformSubFaceWeightsToBase(_numControlPoints, wDu);
    if (wDv) transformSubFaceWeightsToBase(_numControlPoints, wDv);

    return subFace;
}

//
//  Access to PatchTree's StencilTable:
//
Far::StencilTableReal<float> const *
Evaluator::getIrregPatchStencilTable() const {

    return _irregPatch->GetStencilTable();
}


} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
