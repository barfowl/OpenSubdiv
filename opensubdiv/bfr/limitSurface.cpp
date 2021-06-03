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
LimitSurface::LimitSurface() :
        _vtxEval(*this), _varEval(*this), _fvarEval(*this) {

    initialize();
}

LimitSurface::~LimitSurface() {

    clear();
}

void
LimitSurface::initialize() {

    _faceIndex = -1;

    _vtxEval.initialize();
    _varEval.initialize();
    _fvarEval.initialize();
}

void
LimitSurface::parameterize(Parameterization const & param) {

    _param = param;
}

void
LimitSurface::clear() {

    _vtxEval.clear();
    _varEval.clear();
    _fvarEval.clear();
}

//
//  Evaluation methods accessing the local data for a simple regular patch:
//
void
Evaluator::evalRegularPatch(float u, float v,
                               float wP[], float wDu[], float wDv[]) const {

    Far::internal::EvaluatePatchBasisNormalized(
            _regPatchType, _regPatchParam, u, v, wP, wDu, wDv);
}

//
//  Evaluation methods accessing the Far::PatchTree for irregular patches:
//
namespace {
    //  WIP - this should now be redundant given Bfr::Parameterization
    template <typename REAL, typename INT>
    inline void
    irregUvToPtexUv(REAL u, REAL v, INT & ptexID, REAL & ptexU, REAL & ptexV) {
        ptexID = (INT) u;
        ptexU = 2.0f * (u - ptexID);
        ptexV = 2.0f *  v;
    }

    inline void
    convertSubFaceWeightsToBase(int N, float const wSub[4], float wBase[4]) {

        float wOrigin = wSub[0];
        float wNext   = wSub[1] * 0.5f;
        float wCenter = wSub[2] / (float)N;
        float wPrev   = wSub[3] * 0.5f;

        wBase[0] = wCenter + wNext + wPrev + wOrigin;
        wBase[1] = wCenter + wNext;
        wBase[2] = wCenter;
        wBase[3] = wCenter + wPrev;
    }
}

ConstIndexArray
Evaluator::evalIrregularPatch(float u, float v,
                              float wP[], float wDu[], float wDv[]) const {

    Parameterization const & param = _limitSurface._param;

    int subFace = 0;
    if (param.GetType() == Parameterization::QPOLY) {
        subFace = param.ConvertQPolyUVToNormalizedSubQuad(u, v, u, v);
    }

    int subPatchIndex = _irregPatch->FindSubPatch(u, v, subFace);
    assert(subPatchIndex >= 0);

    _irregPatch->EvalSubPatchBasis(subPatchIndex, u, v, wP, wDu, wDv);

    return _irregPatch->GetSubPatchPoints(subPatchIndex);
}

int
Evaluator::evalIrregularLinearPatch(float u, float v,
                                    float wP[], float wDu[], float wDv[]) const {

    Parameterization const & param = _limitSurface._param;
    assert(param.GetType() == Parameterization::QPOLY);

    int subFace = param.ConvertQPolyUVToNormalizedSubQuad(u, v, u, v);

    float wsP[4], wsDu[4], wsDv[4];
    //  This may be optimized out internally and so may not link with -O...
    //      Far::internal::EvalBasisLinear(u, v, wsP, wsDu, wsDv);
    //  WIP - Fix far/patchBasis so we can use the Linear functions
    Far::internal::EvaluatePatchBasisNormalized(
        Far::PatchDescriptor::QUADS, Far::PatchParam(), u, v, wsP, wsDu, wsDv);

    convertSubFaceWeightsToBase(_numControlPoints, wsP,  wP);
    convertSubFaceWeightsToBase(_numControlPoints, wsDu, wDu);
    convertSubFaceWeightsToBase(_numControlPoints, wsDv, wDv);

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
