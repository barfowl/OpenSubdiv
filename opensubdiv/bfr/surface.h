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

#ifndef OPENSUBDIV3_BFR_SURFACE_H
#define OPENSUBDIV3_BFR_SURFACE_H

#include "../version.h"

#include "../bfr/parameterization.h"
#include "../bfr/types.h"
#include "../far/stencilTable.h"
#include "../far/patchDescriptor.h"
#include "../far/patchParam.h"
#include "../vtr/stackBuffer.h"

#include <vector>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class PatchTree;
}

namespace Bfr {

//
//  The Surface class encapsulates the limit surface of a face for any
//  of the data interpolation types (vertex, varying and face-varying)
//  and provides the public interface for its evaluation.
//
//  Instances of Surface are created or initialized by a subclass of the
//  SurfaceFactory. Since existing instances can be re-initialized, they
//  should be tested for validity after such re-initialization.  Surface
//  is also non-copyable, so care should be taken when managing multiple
//  surfaces declared for initialization by a factory.
//
//  All Surfaces are assigned a Parameterization based on the subdivision
//  scheme and the size of the face, which can then be used for evaluation
//  and tessellation of the surface.
//
class Surface {
private:  // non-copyable:
    Surface(Surface const &);
    Surface & operator=(Surface const &);

public:
    Surface() { initialize(); }
    ~Surface() { clear(); }

public:
    bool IsValid() const { return _isValid; }

    Parameterization const & GetParameterization() const { return _param; }

    int GetFaceSize() const  { return _param.GetFaceSize(); }

    //
    //  A Surface is evaluated by preparing a set of "patch points"
    //  required for subsequent evaluation methods.  The patch points
    //  consist of a subset of the control vertices of the mesh in the
    //  neighborhood of the face plus any additional points derived from
    //  them that may be required to represent the limit surface as one
    //  or more parametric patches.
    //
    int GetNumPatchPoints() const { return _numPatchPoints; }

    template <class T, class U>
    void PreparePatchPointValues(T const & meshVertices,
                                 U       & patchPoints) const;

    //  WIP - still need to extend interfaces for 2nd derivs
    //      - may prefer overloads for 1st & 2nd derivs vs dflt args
    template <class T, class U>
    void Evaluate(float u, float v, T const & patchPoints, U * P,
                                                           U * Du = 0,
                                                           U * Dv = 0) const;

    //
    //  The "control vertices" identify the subset of vertices of the
    //  mesh that contribute to the limit surface of the face (use of
    //  "vertex" here is intended to emphasize their presence in the
    //  mesh, unlike patch "points" which may be derived from them).
    //  They will be a subset of the patch points and are intended for
    //  combination with "limit stencils" that can be evaluated below.
    //
    //  Control vertices can be "gathered" into a local buffer for
    //  various purposes (e.g. repeated evaluation, computation of
    //  bounding box, etc.) and stencils can be optionally applied to
    //  control vertices in this form.
    //
    int GetNumControlVertices() const { return _numControlPoints; }

    ConstIndexArray GetControlVertexIndices() const;

    template <class T, class U>
    void GatherControlVertexValues(T const & meshVerts,
                                   U       & controlVerts) const;

    //  WIP - need to extend for 2nd derivs and template for <REAL>
    int EvaluateStencils(float u, float v, float sP[],
                                           float sDu[] = 0,
                                           float sDv[] = 0) const;

    template <class T, class U>
    void ApplyStencil(float const sD[], T const & meshVerts, U * D,
                      bool applyToGatheredControlVertices = false) const;

private:
    friend class SurfaceFactory;

    void clear();
    void initialize();
    void reinitialize() { if (_isValid) clear(), initialize(); }

    //  Access stencils to compute patch point values of client type <T>:
    template <typename REAL, class T>
    void applyIrregPatchStencils(T & patchPoints) const;

    template <typename REAL>
    REAL const * getIrregPatchStencilMatrix() const;

    bool areIrregPatchStencilsDouble() const;

    //  WIP - use of StencilTable is likely to be replaced
    bool irregPatchNeedsStencilTable() const;
    Far::StencilTableReal<float> const * getIrregPatchStencilTable() const;

    //  Evaluation of basis functions and contributing points of internal
    //  patch (implicit for a regular patch):
    //  WIP - will need to template these for <REAL>:
    void evalRegularPatchBasis(float u, float v,
            float wP[], float wDu[], float wDv[]) const;
    ConstIndexArray evalIrregularPatchBasis(float u, float v,
            float wP[], float wDu[], float wDv[]) const;
    int evalMultiLinearPatchBasis(float u, float v,
            float wP[4], float wDu[4], float wDv[4]) const;

    //  Evaluation to combine basis functions and contributing points:
    template <class T, class U>
    void evalRegularPatch(float u, float v, T const & patchPoints,
            U * P, U * Du = 0, U * Dv = 0) const;
    template <class T, class U>
    void evalIrregularPatch(float u, float v, T const & patchPoints,
            U * P, U * Du = 0, U * Dv = 0) const;
    template <class T, class U>
    void evalMultiLinearPatch(float u, float v, T const & patchPoints,
            U * P, U * Du = 0, U * Dv = 0) const;

    //  Evaluation of limit stencils (no templates for point types):
    //  WIP - will need to template these for <REAL>:
    int evalRegularPatchStencils(float u, float v,
            float * sP, float * sDu = 0, float * sDv = 0) const;
    int evalIrregularPatchStencils(float u, float v,
            float * sP, float * sDu = 0, float * sDv = 0) const;
    int evalMultiLinearPatchStencils(float u, float v,
            float * sP, float * sDu = 0, float * sDv = 0) const;

private:
    typedef Far::PatchTree const * IrregPatchPtr;

    Parameterization _param;

    Vtr::internal::StackBuffer<Index,20,true> _controlPoints;

    int _numControlPoints;
    int _numPatchPoints;

    unsigned int _isValid   : 1;
    unsigned int _isRegular : 1;
    unsigned int _isLinear  : 1;

    //  WIP - consider a union here for the reg/irreg members:
    unsigned int _irregOwner : 1;
    IrregPatchPtr _irregPatch;

    Far::PatchDescriptor::Type _regPatchType;
    Far::PatchParam            _regPatchParam;
};

//
//  Inline methods and templates for gathering control points:
//
inline ConstIndexArray
Surface::GetControlVertexIndices() const {
    return ConstIndexArray(&_controlPoints[0], (int)_controlPoints.GetSize());
}

template <class T, class U>
void
Surface::GatherControlVertexValues(T const & meshPoints,
                                   U       & controlPoints) const {
    for (int i = 0; i < _numControlPoints; ++i) {
        //  WIP - cannot guarantee that type T is copyable here, so must
        //        use Clear() and AddWithWeight():
        controlPoints[i].Clear();
        controlPoints[i].AddWithWeight(meshPoints[_controlPoints[i]], 1.0f);
    }
}

template <typename REAL, class T>
void
Surface::applyIrregPatchStencils(T & patchPoints) const {

    REAL const * stencilWeights = getIrregPatchStencilMatrix<REAL>();
    int          stencilStride  = _numControlPoints;

    for (int i = _numControlPoints; i < _numPatchPoints; ++i) {
        patchPoints[i].Clear();
        for (int j = 0; j < _numControlPoints; ++j) {
            patchPoints[i].AddWithWeight(patchPoints[j],
                                         stencilWeights[j]);
        }
        stencilWeights += stencilStride;
    }
}

template <class T, class U>
void
Surface::PreparePatchPointValues(T const & meshPoints,
                                 U       & patchPoints) const {

    GatherControlVertexValues(meshPoints, patchPoints);

    if (_numPatchPoints > _numControlPoints) {
        //  Apply the patch point stencils to compute remaining patch
        //  points from those gathered above from the control points:
        if (irregPatchNeedsStencilTable()) {
            //  WIP - use of the StencilTable will eventually be removed
            getIrregPatchStencilTable()->UpdateValues(
                patchPoints, patchPoints, GetNumControlVertices());
        } else {
            if (areIrregPatchStencilsDouble()) {
                applyIrregPatchStencils<double>(patchPoints);
            } else {
                applyIrregPatchStencils<float>(patchPoints);
            }
        }
    }
}

//
//  Evaluation method templates:
//
template <class T, class U>
void
Surface::evalRegularPatch(float u, float v, T const & patchPoints,
                          U * P, U * Du, U * Dv) const {
    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points:
    //
    if (Du && Dv) {
        float wP[20], wDu[20], wDv[20];
        evalRegularPatchBasis(u, v, wP, wDu, wDv);

        P->Clear();
        Du->Clear();
        Dv->Clear();
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight( patchPoints[i], wP[i]);
            Du->AddWithWeight(patchPoints[i], wDu[i]);
            Dv->AddWithWeight(patchPoints[i], wDv[i]);
        }
    } else {
        float wP[20];
        evalRegularPatchBasis(u, v, wP, 0, 0);

        P->Clear();
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight(patchPoints[i], wP[i]);
        }
    }
}

template <class T, class U>
void
Surface::evalIrregularPatch(float u, float v, T const & patchPoints,
                            U * P, U * Du, U * Dv) const {
    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points:
    //
    if (Du && Dv) {
        float wP[20], wDu[20], wDv[20];
        ConstIndexArray subPatchPointIndices =
                evalIrregularPatchBasis(u, v, wP, wDu, wDv);

        P->Clear();
        Du->Clear();
        Dv->Clear();
        for (int i = 0; i < subPatchPointIndices.size(); ++i) {
            P->AddWithWeight( patchPoints[subPatchPointIndices[i]], wP[i]);
            Du->AddWithWeight(patchPoints[subPatchPointIndices[i]], wDu[i]);
            Dv->AddWithWeight(patchPoints[subPatchPointIndices[i]], wDv[i]);
        }
    } else {
        float wP[20];
        ConstIndexArray subPatchPointIndices =
                evalIrregularPatchBasis(u, v, wP, 0, 0);

        P->Clear();
        for (int i = 0; i < subPatchPointIndices.size(); ++i) {
            P->AddWithWeight( patchPoints[subPatchPointIndices[i]], wP[i]);
        }
    }
}

template <class T, class U>
void
Surface::evalMultiLinearPatch(float u, float v, T const & patchPoints,
                              U * P, U * Du, U * Dv) const {
    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) quadrangulates the face implicitly
    //  as part of evaluation.  The control point that is the origin of
    //  the sub-face is returned along with the weights adjusted to apply
    //  to the full set of control points:
    //
    bool eval1stDerivs = (Du && Dv);

    float wP[4], wDu[4], wDv[4];

    int iOrigin = eval1stDerivs ?
                  evalMultiLinearPatchBasis(u, v, wP, wDu, wDv) :
                  evalMultiLinearPatchBasis(u, v, wP, 0, 0);

    int iNext = (iOrigin + 1) % _numControlPoints;
    int iPrev = (iOrigin + _numControlPoints - 1) % _numControlPoints;

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
    }
    for (int i = 0; i < _numControlPoints; ++i) {
        int wIndex = 2;
        if (i == iOrigin) {
            wIndex = 0;
        } else if (i == iNext) {
            wIndex = 1;
        } else if (i == iPrev) {
            wIndex = 3;
        }
        P->AddWithWeight( patchPoints[i], wP[wIndex]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[i], wDu[wIndex]);
            Dv->AddWithWeight(patchPoints[i], wDv[wIndex]);
        }
    }
}

template <class T, class U>
inline void
Surface::Evaluate(float u, float v, T const & patchPoints,
                  U * P, U * Du, U * Dv) const {

    if (_isRegular) {
        evalRegularPatch(u, v, patchPoints, P, Du, Dv);
    } else if (_isLinear) {
        evalMultiLinearPatch(u, v, patchPoints, P, Du, Dv);
    } else {
        evalIrregularPatch(u, v, patchPoints, P, Du, Dv);
    }
}

template <class T, class U>
void
Surface::ApplyStencil(float const sP[], T const & inputVertices, U * P,
                      bool applyToGatheredControlVerts) const {

    P->Clear();

    if (applyToGatheredControlVerts) {
        T const & gatheredVertices = inputVertices;
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight(gatheredVertices[i], sP[i]);
        }
    } else {
        T const & meshVertices = inputVertices;
        for (int i = 0; i < _numControlPoints; ++i) {
            P->AddWithWeight(meshVertices[_controlPoints[i]], sP[i]);
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE */
