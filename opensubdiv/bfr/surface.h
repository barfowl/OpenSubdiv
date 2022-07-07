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

#include <cstring>

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
    //  Data from both the mesh and the Surface are managed in arrays of
    //  floating point values with variable size and stride. PointBuffer
    //  provides a simple struct to reference a const set of such data:
    //
    struct PointBuffer {
        PointBuffer(const REAL * aData, int aSize, int aStride = 0) :
                data(aData), size(aSize), stride(aStride ? aStride : aSize) { }

        REAL const * data;
        int          size;
        int          stride;
    };

    //
    //  A Surface is evaluated by preparing a set of "patch points"
    //  required for subsequent evaluation methods.  The patch points
    //  consist of a subset of the control vertices of the mesh in the
    //  neighborhood of the face plus any additional points derived from
    //  them that may be required to represent the limit surface as one
    //  or more parametric patches.
    //
    //  If the control points are already or must be separate gathered
    //  as part of a larger collection of patch points, the remaining
    //  patch points can be computed with a separate method.
    //
    int GetNumPatchPoints() const;

    void PreparePatchPoints(PointBuffer const & meshPoints,
                            REAL * patchPoints,
                            int    patchPointStride = 0) const;

    void ComputePatchPoints(REAL * patchPoints, int patchPointSize,
                            int    patchPointStride = 0) const;

    void Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
                                    REAL * P) const;

    void Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
                                    REAL * P, REAL * Du, REAL * Dv) const;

    void Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
                                    REAL * P, REAL * Du,  REAL * Dv,
                                    REAL * Duu, REAL * Duv, REAL * Dvv) const;

    //
    //  The "control points" identify the subset of vertices of the
    //  mesh that contribute to the limit surface of the face.  They
    //  will be a subset of the patch points and are intended for
    //  combination with "limit stencils" that can be evaluated below.
    //
    //  Control points can be "gathered" into a local buffer for
    //  various purposes (e.g. repeated evaluation, computation of
    //  bounding box, etc.) and stencils can be optionally applied to
    //  control points in this form.
    //
    typedef int Index;

    int GetNumControlPoints() const;
    int GetControlPointIndices(Index meshPointIndices[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[], REAL sDv[]) const;

    int EvaluateStencils(REAL const uv[2], REAL sP[],
                         REAL sDu[],  REAL sDv[],
                         REAL sDuu[], REAL sDuv[], REAL sDvv[]) const;

    //  WIP - with "mesh points" and "control points" more clearly defined,
    //        consider their use in naming of the two ApplyStencil methods
    void ApplyStencil(REAL const s[], PointBuffer const & meshPoints,
                      REAL result[]) const;

    //  Convenience methods to gather control points and apply stencil
    //  to the resulting local array of control points:
    void GatherControlPoints(PointBuffer const & meshPoints,
                             REAL * controlPoints,
                             int    controlPointStride = 0) const;

    void ApplyStencilGathered(REAL const s[], PointBuffer const & controlPoints,
                              REAL result[]) const;

private:
    //  Internal evaluation methods applying weighted combinations of client
    //  point data:
    void evaluate(REAL u, REAL v, PointBuffer const & pts, REAL P[],
        REAL Du[], REAL Dv[], REAL Duu[], REAL Dvu[], REAL Dvv[]) const;

    void evalRegularPatch(REAL u, REAL v, PointBuffer const & pts, REAL P[],
        REAL Du[], REAL Dv[], REAL Duu[], REAL Dvu[], REAL Dvv[]) const;
    void evalIrregularPatch(REAL u, REAL v, PointBuffer const & pts, REAL P[],
        REAL Du[], REAL Dv[], REAL Duu[], REAL Dvu[], REAL Dvv[]) const;
    void evalMultiLinearPatch(REAL u, REAL v, PointBuffer const & pts, REAL P[],
        REAL Du[], REAL Dv[], REAL Duu[], REAL Dvu[], REAL Dvv[]) const;

    void pointClear(REAL point[], int size) const;
    void pointCopy( REAL point[], int size, REAL const src[]) const;
    void pointSet(  REAL point[], int size, REAL const src[], REAL w) const;
    void pointAdd(  REAL point[], int size, REAL const src[], REAL w) const;

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
    //  WIP - some of this is now unnecessary as inline use no longer needed
    bool hasIrregPatch() const { return _data.hasIrregPatch(); }
    int  getNumIrregPatchPoints() const;

    REAL const * getIrregPatchPointMatrix() const;

    internal::IrregularPatchType const & getIrregPatch() const;

private:
    //  Access to the set of member variables - provided to the Factory:
    friend class SurfaceFactory;

    internal::SurfaceData       & getSurfaceData()       { return _data; }
    internal::SurfaceData const & getSurfaceData() const { return _data; }

private:
    //  Additional simple member accessors for internal use:
    bool isValid() const   { return _data.isValid(); }
    bool isRegular() const { return _data.isRegular(); }
    bool isLinear() const  { return _data.isLinear(); }

    unsigned char getRegPatchType() const { return _data.getRegPatchType(); }
    unsigned char getRegPatchMask() const { return _data.getRegPatchMask(); }

private:
    //  All member variables encapsulated in a single class:
    internal::SurfaceData _data;
};

//
//  WIP - inline methods for points -- to be moved to source file later
//      * serious performance degradation with arbitrary size vs old <T,U>
//          - so optimization for 3D points included 
//
template <typename REAL>
inline void
Surface<REAL>::pointClear(REAL p[], int size) const {

    if (size == 3) {
        p[0] = 0.0f;
        p[1] = 0.0f;
        p[2] = 0.0f;
    } else {
        for (int i = 0; i < size; ++i) {
            p[i] = 0.0f;
        }
    }
}

template <typename REAL>
inline void
Surface<REAL>::pointCopy(REAL p[], int size, REAL const src[]) const {

    if (size == 3) {
        p[0] = src[0];
        p[1] = src[1];
        p[2] = src[2];
    } else {
        for (int i = 0; i < size; ++i) {
            p[i] = src[i];
        }
    }
}

template <typename REAL>
inline void
Surface<REAL>::pointSet(REAL p[], int size, REAL const src[], REAL w) const {

    if (size == 3) {
        p[0] = w * src[0];
        p[1] = w * src[1];
        p[2] = w * src[2];
    } else {
        for (int i = 0; i < size; ++i) {
            p[i] = w * src[i];
        }
    }
}

template <typename REAL>
inline void
Surface<REAL>::pointAdd(REAL p[], int size, REAL const src[], REAL w) const {

    if (size == 3) {
        p[0] += w * src[0];
        p[1] += w * src[1];
        p[2] += w * src[2];
    } else {
        for (int i = 0; i < size; ++i) {
            p[i] += w * src[i];
        }
    }
}

//
//  Inline methods and templates for gathering control points:
//
template <typename REAL>
inline int
Surface<REAL>::GetNumControlPoints() const {
    return _data.getNumCVs();
}

template <typename REAL>
inline int
Surface<REAL>::GetControlPointIndices(Index cvs[]) const {
    std::memcpy(cvs, _data.getCVIndices(), _data.getNumCVs() * sizeof(Index));
    return _data.getNumCVs();
}

template <typename REAL>
void
Surface<REAL>::GatherControlPoints(PointBuffer const & meshPoints,
        REAL * controlPoints, int controlPointStride) const {

    if (controlPointStride == 0) controlPointStride = meshPoints.size;

    Index const * index = _data.getCVIndices();
    for (int i = 0; i < GetNumControlPoints(); ++i) {
        REAL const * meshPoint = meshPoints.data + meshPoints.stride * index[i];
        REAL       * controlPoint = controlPoints + controlPointStride * i;

        pointCopy(controlPoint, meshPoints.size, meshPoint);
    }
}

template <typename REAL>
inline int
Surface<REAL>::GetNumPatchPoints() const {
    return isRegular() ? GetNumControlPoints() :
                (isLinear() ? (2*GetNumControlPoints() + 1) :
                                getNumIrregPatchPoints());
}

template <typename REAL>
inline internal::IrregularPatchType const &
Surface<REAL>::getIrregPatch() const {
    return _data.getIrregPatch();
}

template <typename REAL>
void
Surface<REAL>::ComputePatchPoints(REAL * patchPoints,
        int patchPointSize, int patchPointStride) const {

    if (patchPointStride == 0) patchPointStride = patchPointSize;

    int numControlPoints = GetNumControlPoints();
    int numPatchPoints   = GetNumPatchPoints();

    if (numPatchPoints == numControlPoints) return;

    if (isLinear()) {
        //  Following the N control points, compute patch points for the
        //  midpoint of the face followed by the midpoint of the N edges:
        int N = numControlPoints;

        REAL * facePoint = patchPoints + patchPointStride * N;
        REAL   facePointWeight = 1.0f / (REAL) N;
        pointClear(facePoint, patchPointSize);

        for (int i = 0; i < N; ++i) {
            int iNext = (i < (N - 1)) ? (i + 1) : 0;

            REAL * v0Point = patchPoints + patchPointStride * i;
            REAL * v1Point = patchPoints + patchPointStride * iNext;

            REAL * edgePoint = patchPoints + patchPointStride * (N + 1 + i);
            pointSet(edgePoint, patchPointSize, v0Point, 0.5f);
            pointAdd(edgePoint, patchPointSize, v1Point, 0.5f);

            pointAdd(facePoint, patchPointSize, v0Point, facePointWeight);
        }
    } else {
        //  Apply the coefficient matrix to compute any patch points
        //  in addition to the control points gathered previously:
        REAL const * matrixRow       = getIrregPatchPointMatrix();
        int          matrixRowStride = numControlPoints;

        REAL * patchPoint = patchPoints + patchPointStride * numControlPoints;

        for (int i = numControlPoints; i < numPatchPoints; ++i) {
            pointClear(patchPoint, patchPointSize);

            REAL * controlPoint = patchPoints;
            for (int j = 0; j < numControlPoints; ++j) {
                pointAdd(patchPoint, patchPointSize, controlPoint,matrixRow[j]);
                controlPoint += patchPointStride;
            }

            patchPoint += patchPointStride;
            matrixRow  += matrixRowStride;
        }
    }
}

template <typename REAL>
inline void
Surface<REAL>::PreparePatchPoints(PointBuffer const & meshPoints,
        REAL * patchPoints, int patchPointStride) const {

    if (patchPointStride == 0) patchPointStride = meshPoints.size;

    GatherControlPoints(meshPoints, patchPoints, patchPointStride);

    if (!isRegular()) {
        ComputePatchPoints(patchPoints, meshPoints.size, patchPointStride);
    }
}

//
//  Evaluation method templates:
//
template <typename REAL>
void
Surface<REAL>::evalRegularPatch(REAL u, REAL v, PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {

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

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    int size = points.size;

    REAL const * controlPoint = points.data;

    pointSet(P, size, points.data, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, size, points.data, wDu[0]);
        pointSet(Dv, size, points.data, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, size, points.data, wDuu[0]);
            pointSet(Duv, size, points.data, wDuv[0]);
            pointSet(Dvv, size, points.data, wDvv[0]);
        }
    }

    for (int i = 1; i < GetNumControlPoints(); ++i) {
        controlPoint += points.stride;

        pointAdd(P, size, controlPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, size, controlPoint, wDu[i]);
            pointAdd(Dv, size, controlPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, size, controlPoint, wDuu[i]);
                pointAdd(Duv, size, controlPoint, wDuv[i]);
                pointAdd(Dvv, size, controlPoint, wDvv[i]);
            }
        }
    }
}

template <typename REAL>
void
Surface<REAL>::evalIrregularPatch(REAL u, REAL v, PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {

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
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        subPatchPoints = evalIrregularPatchBasis(u, v, wP, wDu, wDv,
                                                       wDuu, wDuv, wDvv);
    }

    //
    //  Apply each successive control point to all derivatives at once,
    //  rather than computing each derivate independently:
    //
    int size = points.size;

    REAL const * patchPoint = points.data + points.stride * subPatchPoints[0];

    pointSet(P, size, patchPoint, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, size, patchPoint, wDu[0]);
        pointSet(Dv, size, patchPoint, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, size, patchPoint, wDuu[0]);
            pointSet(Duv, size, patchPoint, wDuv[0]);
            pointSet(Dvv, size, patchPoint, wDvv[0]);
        }
    }

    for (int i = 1; i < subPatchPoints.size(); ++i) {
        patchPoint = points.data + points.stride * subPatchPoints[i];

        pointAdd(P, size, patchPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, size, patchPoint, wDu[i]);
            pointAdd(Dv, size, patchPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, size, patchPoint, wDuu[i]);
                pointAdd(Duv, size, patchPoint, wDuv[i]);
                pointAdd(Dvv, size, patchPoint, wDvv[i]);
            }
        }
    }
}


template <typename REAL>
void
Surface<REAL>::evalMultiLinearPatch(REAL u, REAL v, PointBuffer const &points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const {
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

    int subQuad = -1;
    if (!eval1stDerivs) {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, 0, 0, 0, 0, 0);
    } else if (!eval2ndDerivs) {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv, 0, 0, 0);
    } else {
        subQuad = evalMultiLinearPatchBasis(u, v, wP, wDu, wDv,
                                                  wDuu, wDuv, wDvv);
    }

    int N = GetNumControlPoints();

    int quadIndices[4];
    quadIndices[0] = subQuad;
    quadIndices[1] = N + 1 + subQuad;
    quadIndices[2] = N;
    quadIndices[3] = N + 1 + (subQuad + N - 1) % N;

    int size = points.size;

    REAL const * quadPoint = points.data + points.stride * quadIndices[0];

    pointSet(P, size, quadPoint, wP[0]);
    if (eval1stDerivs) {
        pointSet(Du, size, quadPoint, wDu[0]);
        pointSet(Dv, size, quadPoint, wDv[0]);
        if (eval2ndDerivs) {
            pointSet(Duu, size, quadPoint, wDuu[0]);
            pointSet(Duv, size, quadPoint, wDuv[0]);
            pointSet(Dvv, size, quadPoint, wDvv[0]);
        }
    }

    for (int i = 1; i < 4; ++i) {
        quadPoint = points.data + points.stride * quadIndices[i];

        pointAdd(P, size, quadPoint, wP[i]);
        if (eval1stDerivs) {
            pointAdd(Du, size, quadPoint, wDu[i]);
            pointAdd(Dv, size, quadPoint, wDv[i]);
            if (eval2ndDerivs) {
                pointAdd(Duu, size, quadPoint, wDuu[i]);
                pointAdd(Duv, size, quadPoint, wDuv[i]);
                pointAdd(Dvv, size, quadPoint, wDvv[i]);
            }
        }
    }
}

template <typename REAL>
inline void
Surface<REAL>::evaluate(REAL u, REAL v, PointBuffer const & points,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const{

    if (isRegular()) {
        evalRegularPatch(u, v, points, P, Du, Dv, Duu, Duv, Dvv);
    } else if (isLinear()) {
        evalMultiLinearPatch(u, v, points, P, Du, Dv, Duu, Duv, Dvv);
    } else {
        evalIrregularPatch(u, v, points, P, Du, Dv, Duu, Duv, Dvv);
    }
}

template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
    REAL * P, REAL * Du, REAL * Dv, REAL * Duu, REAL * Duv, REAL * Dvv) const{

    evaluate(uv[0], uv[1], patchPoints, P, Du, Dv, Duu, Duv, Dvv);
}

template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
                        REAL * P, REAL * Du, REAL * Dv) const {

    evaluate(uv[0], uv[1], patchPoints, P, Du, Dv, 0, 0, 0);
}

template <typename REAL>
inline void
Surface<REAL>::Evaluate(REAL const uv[2], PointBuffer const & patchPoints,
                        REAL * P) const {

    evaluate(uv[0], uv[1], patchPoints, P, 0, 0, 0, 0, 0);
}

//
//  Inline stencil evaluation methods for derivative overloads:
//
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
void
Surface<REAL>::ApplyStencil(REAL const s[],
        PointBuffer const & meshPoints, REAL result[]) const {

    Index const * index = _data.getCVIndices();

    REAL const * meshPoint = meshPoints.data + meshPoints.stride * index[0];
    pointSet(result, meshPoints.size, meshPoint, s[0]);

    for (int i = 1; i < GetNumControlPoints(); ++i) {
        meshPoint = meshPoints.data + meshPoints.stride * index[i];
        pointAdd(result, meshPoints.size, meshPoint, s[i]);
    }
}

template <typename REAL>
void
Surface<REAL>::ApplyStencilGathered(REAL const s[],
        PointBuffer const & controlPoints, REAL result[]) const {

    REAL const * controlPoint = controlPoints.data;
    pointSet(result, controlPoints.size, controlPoint, s[0]);

    for (int i = 1; i < GetNumControlPoints(); ++i) {
        controlPoint += controlPoints.stride;
        pointAdd(result, controlPoints.size, controlPoint, s[i]);
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE */
