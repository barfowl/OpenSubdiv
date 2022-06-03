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

#include "../bfr/surfaceData.h"
#include "../bfr/parameterization.h"
#include "../vtr/array.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

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
template <typename REAL>
class Surface {
public:
    Surface();
    ~Surface() { }

private:  // non-copyable:
    Surface(Surface const &);
    Surface & operator=(Surface const &);

public:
    //
    //  Simple public queries:
    //
    bool IsValid() const { return _data.isValid(); }

    Parameterization GetParameterization() const { return _data.getParam(); }

    int GetFaceSize() const  { return GetParameterization().GetFaceSize(); }

    //
    //  A Surface is evaluated by preparing a set of "patch points"
    //  required for subsequent evaluation methods.  The patch points
    //  consist of a subset of the control vertices of the mesh in the
    //  neighborhood of the face plus any additional points derived from
    //  them that may be required to represent the limit surface as one
    //  or more parametric patches.
    //
    int GetNumPatchPoints() const;

    template <class T, class U>
    void PreparePatchPointValues(T const & meshVertices,
                                 U       & patchPoints) const;

    template <class T, class U>
    void Evaluate(REAL const uv[2], T const & patchPoints, U * P) const;

    template <class T, class U>
    void Evaluate(REAL const uv[2], T const & patchPoints, U * P,
                                    U * Du, U * Dv) const;
    template <class T, class U>
    void Evaluate(REAL const uv[2], T const & patchPoints, U * P,
                                    U * Du,  U * Dv,
                                    U * Duu, U * Duv, U * Dvv) const;

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
    typedef internal::SurfaceData::Index Index;

    int GetNumControlVertices() const { return _data.getNumCVs(); }

    Index const * GetControlVertexIndices() const {return _data.getCVIndices();}

    int EvaluateStencils(REAL const uv[2], REAL sP[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[], REAL sDv[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[],  REAL sDv[],
                         REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

    template <class T, class U>
    void ApplyStencil(REAL const sD[], T const & meshVerts, U * D) const;

    //  Convenience methods to gather control vertices and apply stencil
    //  to the resulting local array of control vertices:
    template <class T, class U>
    void GatherControlVertexValues(T const & meshVerts, U & cVerts) const;

    template <class T, class U>
    void ApplyStencilGathered(REAL const sD[], T const & cVerts, U * D) const;

private:
    //  Internal evaluation methods applying weighted combinations of client
    //  type <T>, and so inlined below
    template <class T, class U>
    void evaluate(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du,  U * Dv, U * Duu, U * Duv, U * Dvv) const;

    template <class T, class U>
    void evalRegularPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;
    template <class T, class U>
    void evalIrregularPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;
    template <class T, class U>
    void evalMultiLinearPatch(REAL u, REAL v, T const & patchPoints,
            U * P, U * Du, U * Dv, U * Duu, U * Dvu, U * Dvv) const;

private:
    //  Internal evaluation methods for basis functions and contributing
    //  patch points (implicit for a regular patch) and limit stencils:
    typedef Vtr::ConstArray<int> PatchPointArray;

    void evalRegularPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;
    PatchPointArray evalIrregularPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;
    int evalMultiLinearPatchBasis(REAL u, REAL v, REAL wP[],
        REAL wDu[], REAL wDv[], REAL wDuu[], REAL wDuv[], REAL wDvv[]) const;

    int evalRegularPatchStencils(REAL u, REAL v, REAL sP[],
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;
    int evalIrregularPatchStencils(REAL u, REAL v, REAL sP[],
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;
    int evalMultiLinearPatchStencils(REAL u, REAL v, REAL sP[], 
        REAL sDu[], REAL sDv[], REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

private:
    //  Access to necessary details of the irregular patch representation,
    //  hidden to avoid publicly exposing that representation:
    int getNumIrregPatchPoints() const;

    REAL const * getIrregPatchPointMatrix() const;

private:
    //  Access to the set of member variables - provided to the Factory:
    friend class SurfaceFactory;

    typedef internal::SurfaceData SurfaceData;

    SurfaceData       & getSurfaceData()       { return _data; }
    SurfaceData const & getSurfaceData() const { return _data; }

private:
    //  Additional simple member accessors for internal use:
    typedef SurfaceData::IrregPatchPtr IrregPatchPtr;

    bool isValid() const   { return _data.isValid(); }
    bool isRegular() const { return _data.isRegular(); }
    bool isLinear() const  { return _data.isLinear(); }

    unsigned char getRegPatchType() const { return _data.getRegPatchType(); }
    unsigned char getRegPatchMask() const { return _data.getRegPatchMask(); }

    bool          hasIrregPatch() const { return _data.hasIrregPatch(); }
    IrregPatchPtr getIrregPatch() const { return _data.getIrregPatch(); }

private:
    //  All member variables encapsulated in a single class:
    SurfaceData _data;
};

//
//  Inline methods and templates for gathering control points:
//
template <typename REAL>
template <class T, class U>
void
Surface<REAL>::GatherControlVertexValues(T const & meshPoints,
                                         U       & controlPoints) const {
    Index const * cvs = GetControlVertexIndices();
    for (int i = 0; i < GetNumControlVertices(); ++i) {
        //  WIP - cannot guarantee that type T is copyable here, so must
        //        use Clear() and AddWithWeight():
        controlPoints[i].Clear();
        controlPoints[i].AddWithWeight(meshPoints[cvs[i]], 1.0f);
    }
}

template <typename REAL>
inline int
Surface<REAL>::GetNumPatchPoints() const {
    return hasIrregPatch() ? getNumIrregPatchPoints() : GetNumControlVertices();
}

template <typename REAL>
template <class T, class U>
void
Surface<REAL>::PreparePatchPointValues(T const & meshPoints,
                                       U       & patchPoints) const {

    GatherControlVertexValues(meshPoints, patchPoints);

    int numControlPoints = GetNumControlVertices();
    int numPatchPoints   = GetNumPatchPoints();

    //  Apply the coefficient matrix to compute any patch points in
    //  addition to the control points gathered above:
    if (numPatchPoints > numControlPoints) {
        REAL const * matrixRow = getIrregPatchPointMatrix();

        for (int i = numControlPoints; i < numPatchPoints; ++i) {
            patchPoints[i].Clear();
            for (int j = 0; j < numControlPoints; ++j) {
                patchPoints[i].AddWithWeight(patchPoints[j], matrixRow[j]);
            }
            matrixRow += numControlPoints;
        }
    }
}

//
//  Evaluation method templates:
//
template <typename REAL>
template <class T, class U>
void
Surface<REAL>::evalRegularPatch(REAL u, REAL v, T const & patchPoints,
                          U * P,   U * Du,  U * Dv,
                          U * Duu, U * Duv, U * Dvv) const {
    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    if (!eval1stDerivs) {
        evalRegularPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        evalRegularPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        evalRegularPatchBasis(u, v, wP, wDu, wDv, wDuu, wDuv, wDvv);
    }

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
    }

    for (int i = 0; i < GetNumControlVertices(); ++i) {
        P->AddWithWeight(patchPoints[i], wP[i]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[i], wDu[i]);
            Dv->AddWithWeight(patchPoints[i], wDv[i]);
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[i], wDuu[i]);
                Duv->AddWithWeight(patchPoints[i], wDuv[i]);
                Dvv->AddWithWeight(patchPoints[i], wDvv[i]);
            }
        }
    }
}

template <typename REAL>
template <class T, class U>
void
Surface<REAL>::evalIrregularPatch(REAL u, REAL v, T const & patchPoints,
                            U * P,   U * Du,  U * Dv,
                            U * Duu, U * Duv, U * Dvv) const {
    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[20], wDu[20], wDv[20], wDuu[20], wDuv[20], wDvv[20];
    PatchPointArray subPatchPoints;

    if (!eval1stDerivs) {
        subPatchPoints = evalIrregularPatchBasis(u, v,
                wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subPatchPoints = evalIrregularPatchBasis(u, v,
                wP, wDu, wDv, 0, 0, 0);
    } else {
        subPatchPoints = evalIrregularPatchBasis(u, v,
                wP, wDu, wDv, wDuu, wDuv, wDvv);
    }

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
    }

    for (int i = 0; i < subPatchPoints.size(); ++i) {
        P->AddWithWeight( patchPoints[subPatchPoints[i]], wP[i]);
        if (eval1stDerivs) {
            Du->AddWithWeight(patchPoints[subPatchPoints[i]], wDu[i]);
            Dv->AddWithWeight(patchPoints[subPatchPoints[i]], wDv[i]);
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[subPatchPoints[i]], wDuu[i]);
                Duv->AddWithWeight(patchPoints[subPatchPoints[i]], wDuv[i]);
                Dvv->AddWithWeight(patchPoints[subPatchPoints[i]], wDvv[i]);
            }
        }
    }
}

template <typename REAL>
template <class T, class U>
void
Surface<REAL>::evalMultiLinearPatch(REAL u, REAL v, T const & patchPoints,
                              U * P,   U * Du,  U * Dv,
                              U * Duu, U * Duv, U * Dvv) const {
    //
    //  Linear evaluation of irregular N-sided faces (usually for varying
    //  or linear face-varying cases) quadrangulates the face implicitly
    //  as part of evaluation.  The control point that is the origin of
    //  the sub-face is returned along with the weights adjusted to apply
    //  to the full set of control points:
    //
    bool eval1stDerivs = (Du && Dv);
    bool eval2ndDerivs = eval1stDerivs && (Duu && Duv && Dvv);

    REAL wP[4], wDu[4], wDv[4], wDuu[4], wDuv[4], wDvv[4];

    int iOrigin = -1;
    if (!eval1stDerivs) {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        iOrigin = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv,
                                                        wDuu, wDuv, wDvv);
    }

    P->Clear();
    if (eval1stDerivs) {
        Du->Clear();
        Dv->Clear();
        if (eval2ndDerivs) {
            Duu->Clear();
            Duv->Clear();
            Dvv->Clear();
        }
    }

    int numControlPoints = GetNumControlVertices();

    int iNext = (iOrigin + 1) % numControlPoints;
    int iPrev = (iOrigin + numControlPoints - 1) % numControlPoints;

    for (int i = 0; i < numControlPoints; ++i) {
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
            if (eval2ndDerivs) {
                Duu->AddWithWeight(patchPoints[i], wDuu[wIndex]);
                Duv->AddWithWeight(patchPoints[i], wDuv[wIndex]);
                Dvv->AddWithWeight(patchPoints[i], wDvv[wIndex]);
            }
        }
    }
}

template <typename REAL>
template <class T, class U>
inline void
Surface<REAL>::evaluate(REAL u, REAL v, T const & patchPoints,
                  U * P, U * Du, U * Dv, U * Duu, U * Duv, U * Dvv) const {

    if (isRegular()) {
        evalRegularPatch<T,U>(u, v, patchPoints,
                                   P, Du, Dv, Duu, Duv, Dvv);
    } else if (isLinear()) {
        evalMultiLinearPatch<T,U>(u, v, patchPoints,
                                       P, Du, Dv, Duu, Duv, Dvv);
    } else {
        evalIrregularPatch<T,U>(u, v, patchPoints,
                                     P, Du, Dv, Duu, Duv, Dvv);
    }
}

template <typename REAL>
template <class T, class U>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], T const & patchPoints,
                  U * P, U * Du, U * Dv, U * Duu, U * Duv, U * Dvv) const {

    evaluate<T,U>(uv[0], uv[1], patchPoints, P, Du, Dv, Duu, Duv, Dvv);
}

template <typename REAL>
template <class T, class U>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], T const & patchPoints,
                  U * P, U * Du, U * Dv) const {

    evaluate<T,U>(uv[0], uv[1], patchPoints, P, Du, Dv, 0, 0, 0);
}

template <typename REAL>
template <class T, class U>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], T const & patchPoints, U * P) const {

    evaluate<T,U>(uv[0], uv[1], patchPoints, P, 0, 0, 0, 0, 0);
}

template <typename REAL>
inline int
Surface<REAL>::EvaluateStencils(REAL const uv[2],
                          REAL sP[], REAL sDu[], REAL sDv[]) const {

    return EvaluateStencils(uv, sP, sDu, sDv, 0, 0, 0);
}

template <typename REAL>
inline int
Surface<REAL>::EvaluateStencils(REAL const uv[2], REAL sP[]) const {

    return EvaluateStencils(uv, sP, 0, 0, 0, 0, 0);
}

template <typename REAL>
template <class T, class U>
void
Surface<REAL>::ApplyStencil(REAL const sD[], T const & meshVertices, U * D) const {

    D->Clear();
    Index const * cvs = GetControlVertexIndices();
    for (int i = 0; i < GetNumControlVertices(); ++i) {
        D->AddWithWeight(meshVertices[cvs[i]], sD[i]);
    }
}
template <typename REAL>
template <class T, class U>
void
Surface<REAL>::ApplyStencilGathered(REAL const sD[], T const & cvs, U * D) const {

    D->Clear();
    for (int i = 0; i < GetNumControlVertices(); ++i) {
        D->AddWithWeight(cvs[i], sD[i]);
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE */
