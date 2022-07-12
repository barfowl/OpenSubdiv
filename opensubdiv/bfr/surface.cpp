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

#include "../bfr/surface.h"
#include "../bfr/surfaceData.h"
#include "../bfr/patchTree.h"
#include "../far/patchParam.h"
#include "../far/patchDescriptor.h"
#include "../far/patchBasis.h"

#include <cassert>
#include <cstdio>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Internal utilities for dealing with "points" -- floating point tuples:
//
namespace points {
    //
    //  Class for point operations that will be specialized for small,
    //  fixed sizes (via template parameter <int SIZE>).
    //
    //  A class is used for this purpose -- rather than simple inline
    //  functions -- since template functions do not support the desired
    //  partial specialization.
    //
    template <typename REAL, int SIZE = 0>
    struct Point {
        static void Set(REAL p[], REAL w, REAL const src[], int size) {
            for (int i = 0; i < size; ++i) {
                p[i] = w * src[i];
            }
        }
        static void Add(REAL p[], REAL w, REAL const src[], int size) {
            for (int i = 0; i < size; ++i) {
                p[i] += w * src[i];
            }
        }
    };

    //  Point specialization for SIZE = 1:
    template <typename REAL>
    struct Point<REAL, 1> {
        static void Set(REAL * p, REAL w, REAL const * src, int) {
            p[0] = w * src[0];
        }
        static void Add(REAL * p, REAL w, REAL const * src, int) {
            p[0] += w * src[0];
        }
    };

    //  Point specialization for SIZE = 2:
    template <typename REAL>
    struct Point<REAL, 2> {
        static void Set(REAL * p, REAL w, REAL const * src, int) {
            p[0] = w * src[0];
            p[1] = w * src[1];
        }
        static void Add(REAL * p, REAL w, REAL const * src, int) {
            p[0] += w * src[0];
            p[1] += w * src[1];
        }
    };

    //  Point specialization for SIZE = 3:
    template <typename REAL>
    struct Point<REAL, 3> {
        static void Set(REAL * p, REAL w, REAL const * src, int) {
            p[0] = w * src[0];
            p[1] = w * src[1];
            p[2] = w * src[2];
        }
        static void Add(REAL * p, REAL w, REAL const * src, int) {
            p[0] += w * src[0];
            p[1] += w * src[1];
            p[2] += w * src[2];
        }
    };

    //  Point specialization for SIZE = 4:
    template <typename REAL>
    struct Point<REAL, 4> {
        static void Set(REAL * p, REAL w, REAL const * src, int) {
            p[0] = w * src[0];
            p[1] = w * src[1];
            p[2] = w * src[2];
            p[3] = w * src[3];
        }
        static void Add(REAL * p, REAL w, REAL const * src, int) {
            p[0] += w * src[0];
            p[1] += w * src[1];
            p[2] += w * src[2];
            p[3] += w * src[3];
        }
    };

    //
    //  Simple descriptor for a combination of control points, which
    //  will be used to gather parameteris to simplify the dispatch to
    //  potential specializations:
    //
    template <typename REAL>
    struct CombinationDescriptor {
        REAL const * pointData;
        int          pointSize;
        int          pointStride;

        int          numSrcPoints;
        int  const * srcPointIndices;

        int                  numResults;
        REAL const * const * resultWeights;
        REAL              ** resultData;
    };

    //
    //  Template class with methods to combine sets of points -- with
    //  the intention of specializing for small point sizes:
    //
    template <typename REAL, int SIZE = 0>
    struct Operations {

        //
        //  Linear combination of source points into a single result:
        //
        static void
        Combine1(CombinationDescriptor<REAL> const & args) {
            int pSize   = args.pointSize;
            int pStride = args.pointStride;

            REAL const * srcData    = args.pointData;
            int  const * srcIndices = args.srcPointIndices;

            REAL const * w = args.resultWeights[0];
            REAL       * p = args.resultData[0];

            if (srcIndices == 0) {
                REAL const * pSrc = srcData;
                Point<REAL,SIZE>::Set(p, w[0], pSrc, pSize);

                for (int i = 1; i < args.numSrcPoints; ++i) {
                    pSrc += pStride;
                    Point<REAL,SIZE>::Add(p, w[i], pSrc, pSize);
                }
            } else {
                REAL const * pSrc = srcData + pStride * srcIndices[0];
                Point<REAL,SIZE>::Set(p, w[0], pSrc, pSize);

                for (int i = 1; i < args.numSrcPoints; ++i) {
                    pSrc = srcData + pStride * srcIndices[i];
                    Point<REAL,SIZE>::Add(p, w[i], pSrc, pSize);
                }
            }
        }

        //
        //  Linear combination of source points into 3 results -- for use
        //  computing position and 1st derivatives:
        //
        static void
        Combine3(CombinationDescriptor<REAL> const & args) {
            int pSize   = args.pointSize;
            int pStride = args.pointStride;

            REAL const * srcData    = args.pointData;
            int  const * srcIndices = args.srcPointIndices;

            REAL const * const * wArray = args.resultWeights;
            REAL              ** pArray = args.resultData;

            //
            //  Apply each successive control point to all derivatives at once,
            //  rather than computing each derivate independently:
            //
            REAL const * pSrc = srcIndices ? (srcData + pStride * srcIndices[0])
                                           : srcData;

            Point<REAL,SIZE>::Set(pArray[0], wArray[0][0], pSrc, pSize);
            Point<REAL,SIZE>::Set(pArray[1], wArray[1][0], pSrc, pSize);
            Point<REAL,SIZE>::Set(pArray[2], wArray[2][0], pSrc, pSize);

            for (int i = 1; i < args.numSrcPoints; ++i) {
                pSrc = srcIndices ? (srcData + pStride * srcIndices[i]) :
                                    (pSrc + pStride);

                Point<REAL,SIZE>::Add(pArray[0], wArray[0][i], pSrc, pSize);
                Point<REAL,SIZE>::Add(pArray[1], wArray[1][i], pSrc, pSize);
                Point<REAL,SIZE>::Add(pArray[2], wArray[2][i], pSrc, pSize);
            }
        }

        //
        //  Linear combination of source points into an aribtrary number
        //  of specified results -- for use computing position and all
        //  derivatives (6 results) or other sparse sets of derivatives:
        //
        static void
        CombineMultiple(CombinationDescriptor<REAL> const & args) {
            int pSize   = args.pointSize;
            int pStride = args.pointStride;

            REAL const * srcData    = args.pointData;
            int  const * srcIndices = args.srcPointIndices;

            REAL const * const * wArray = args.resultWeights;
            REAL              ** pArray = args.resultData;

            //
            //  Apply each successive control point to all derivatives at once,
            //  rather than computing each derivate independently:
            //
            REAL const * pSrc = srcIndices ? (srcData + pStride * srcIndices[0])
                                           : srcData;
            for (int j = 0; j < args.numResults; ++j) {
                Point<REAL,SIZE>::Set(pArray[j], wArray[j][0], pSrc, pSize);
            }

            for (int i = 1; i < args.numSrcPoints; ++i) {
                pSrc = srcIndices ? (srcData + pStride * srcIndices[i]) :
                                    (pSrc + pStride);
                for (int j = 0; j < args.numResults; ++j) {
                    Point<REAL,SIZE>::Add(pArray[j], wArray[j][i], pSrc, pSize);
                }
            }
        }

        //
        //  Linear combination of source points into an aribtrary number
        //  of results that are contiguous in memory.  Both the sets of
        //  weights and the corresponding results are contiguous and are
        //  stored as the first entry of the Descriptor. For use applying
        //  a full stencil matrix to a set of consecutive patch points:
        //
        static void
        CombineConsecutive(CombinationDescriptor<REAL> const & args) {
            int pSize   = args.pointSize;
            int pStride = args.pointStride;

            REAL const * w = args.resultWeights[0];
            REAL       * p = args.resultData[0];

            //  Currently only supports source points as first N points,
            //  i.e. the control points and not the points of a sub-patch:
            assert(args.srcPointIndices == 0);;

            for (int i = 0; i < args.numResults; ++i) {
                REAL const * pSrc = args.pointData;
                Point<REAL,SIZE>::Set(p, w[0], pSrc, pSize);

                for (int i = 1; i < args.numSrcPoints; ++i) {
                    pSrc += pStride;
                    Point<REAL,SIZE>::Add(p, w[i], pSrc, pSize);
                }

                p += pStride;
                w += args.numSrcPoints;
            }
        }

        static void
        SplitFace(REAL * patchPoints, int N, int pSize, int pStride) {
            REAL const * controlPointData = patchPoints;
            REAL       * patchPointData   = patchPoints + pStride * N;

            REAL invN = 1.0f / (REAL) N;

            REAL * facePoint = patchPointData;
            std::memset(facePoint, 0, pSize * sizeof(REAL));

            for (int i = 0; i < N; ++i) {
                int iNext = (i < (N - 1)) ? (i + 1) : 0;

                REAL const * v0Point = controlPointData + pStride * i;
                REAL const * v1Point = controlPointData + pStride * iNext;

                Point<REAL,SIZE>::Add(facePoint, invN, v0Point, pSize);

                REAL * edgePoint = patchPointData + pStride * (1 + i);
                Point<REAL,SIZE>::Set(edgePoint, 0.5f, v0Point, pSize);
                Point<REAL,SIZE>::Add(edgePoint, 0.5f, v1Point, pSize);
            }
        }
    };

    //
    //  Main combination function to invoke specializations:
    //
    template <typename REAL>
    inline void
    Combine(CombinationDescriptor<REAL> const & args) {

        if (args.numResults == 1) {
            switch (args.pointSize) {
            case 1:  Operations<REAL,1>::Combine1(args); break;
            case 2:  Operations<REAL,2>::Combine1(args); break;
            case 3:  Operations<REAL,3>::Combine1(args); break;
            case 4:  Operations<REAL,4>::Combine1(args); break;
            default: Operations<REAL>::Combine1(args); break;
            }
        } else if (args.numResults == 3) {
            switch (args.pointSize) {
            case 1:  Operations<REAL,1>::Combine3(args); break;
            case 2:  Operations<REAL,2>::Combine3(args); break;
            case 3:  Operations<REAL,3>::Combine3(args); break;
            case 4:  Operations<REAL,4>::Combine3(args); break;
            default: Operations<REAL>::Combine3(args); break;
            }
        } else {
            switch (args.pointSize) {
            case 1:  Operations<REAL,1>::CombineMultiple(args); break;
            case 2:  Operations<REAL,2>::CombineMultiple(args); break;
            case 3:  Operations<REAL,3>::CombineMultiple(args); break;
            case 4:  Operations<REAL,4>::CombineMultiple(args); break;
            default: Operations<REAL>::CombineMultiple(args); break;
            }
        }
    }
}


//
//  Constructor for the Surface -- defers to the constructor for its full
//  set of member variables, but marks the precision as double when needed
//  (as a specialization here):
//
template <typename REAL>
Surface<REAL>::Surface() : _data() {

    //  Surface<> should not be adding members outside its SurfaceData:
    assert(sizeof(*this) == sizeof(internal::SurfaceData));
}

template <>
Surface<double>::Surface() : _data() {

    _data.setDouble(true);
}


//
//  Simple internal utilities:
//
template <typename REAL>
inline internal::IrregularPatchType const &
Surface<REAL>::getIrregPatch() const {

    return _data.getIrregPatch();
}

template <typename REAL>
int
Surface<REAL>::GetNumPatchPoints() const {

    if (isRegular()) {
        return GetNumControlPoints();
    } else if (isLinear()) {
        return 2 * GetNumControlPoints() + 1;
    } else {
        return getIrregPatch().GetNumPointsTotal();
    }
}

template <typename REAL>
int
Surface<REAL>::GetControlPointIndices(Index cvs[]) const {

    std::memcpy(cvs, _data.getCVIndices(), _data.getNumCVs() * sizeof(Index));
    return _data.getNumCVs();
}


//
//  Methods for gathering and computing control and patch points:
//
template <typename REAL>
void
Surface<REAL>::GatherControlPoints(
        REAL const meshPoints[],  PointDescriptor const & meshDesc,
        REAL     * controlPoints, PointDescriptor const & controlDesc) const {

    Index const * meshIndices = _data.getCVIndices();
    for (int i = 0; i < GetNumControlPoints(); ++i) {
        REAL const * pSrc = meshPoints    + meshDesc.stride * meshIndices[i];
        REAL       * pDst = controlPoints + controlDesc.stride * i;

        std::memcpy(pDst, pSrc, meshDesc.size * sizeof(REAL));
    }
}

template <typename REAL>
void
Surface<REAL>::computeLinearPatchPoints(REAL * patchPoints,
        PointDescriptor const & pointDesc) const {

    //
    //  Following the N control points, compute patch points for the
    //  midpoint of the face followed by the midpoint of the N edges:
    //
    int    N      = GetNumControlPoints();
    REAL * P      = patchPoints;
    int    size   = pointDesc.size;
    int    stride = pointDesc.stride;

    switch (size) {
    case 1:  points::Operations<REAL,1>::SplitFace(P, N, size, stride); break;
    case 2:  points::Operations<REAL,2>::SplitFace(P, N, size, stride); break;
    case 3:  points::Operations<REAL,3>::SplitFace(P, N, size, stride); break;
    case 4:  points::Operations<REAL,4>::SplitFace(P, N, size, stride); break;
    default: points::Operations<REAL>::SplitFace(P, N, size, stride); break;
    }
}

template <typename REAL>
void
Surface<REAL>::computeIrregularPatchPoints(REAL * allPatchPoints,
        PointDescriptor const & pointDesc) const {

    //
    //  An "irregular patch" may be represented by a regular patch in
    //  rare cases, so be sure there are patch points to compute:
    //
    internal::IrregularPatchType const & irregPatch = getIrregPatch();

    int numControlPoints = GetNumControlPoints();
    int numPatchPoints   = irregPatch.GetNumPointsTotal();

    if (numPatchPoints == numControlPoints) return;

    //
    //  Identify the control points, the stencil matrix with coefficients
    //  to compute remaining patch points, and the target patch points:
    //
    REAL const * controlPoints = allPatchPoints;
    REAL const * stencilMatrix = irregPatch.GetStencilMatrix<REAL>();

    REAL * patchPoints = allPatchPoints + pointDesc.stride * numControlPoints;

    //
    //  Assemble arguments used by methods to combine points and apply:
    //
    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = controlPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = numControlPoints;
    combineArgs.srcPointIndices = 0;

    combineArgs.numResults    = numPatchPoints - numControlPoints;
    combineArgs.resultWeights = &stencilMatrix;
    combineArgs.resultData    = &patchPoints;

    switch (combineArgs.pointSize) {
    case 1:  points::Operations<REAL,1>::CombineConsecutive(combineArgs); break;
    case 2:  points::Operations<REAL,2>::CombineConsecutive(combineArgs); break;
    case 3:  points::Operations<REAL,3>::CombineConsecutive(combineArgs); break;
    case 4:  points::Operations<REAL,4>::CombineConsecutive(combineArgs); break;
    default: points::Operations<REAL>::CombineConsecutive(combineArgs); break;
    }
}

namespace {
    template <typename REAL>
    inline int
    assignWeightsPerDeriv(REAL * const deriv[6], int wSize, REAL wBuffer[],
                          REAL * wDeriv[6]) {

        std::memset(wDeriv, 0, 6 * sizeof(REAL*));

        wDeriv[0] = wBuffer;
        if (deriv[1] && deriv[2]) {
            wDeriv[1] = wDeriv[0] + wSize;
            wDeriv[2] = wDeriv[1] + wSize;
            if (deriv[3] && deriv[4] && deriv[5]) {
                wDeriv[3] = wDeriv[2] + wSize;
                wDeriv[4] = wDeriv[3] + wSize;
                wDeriv[5] = wDeriv[4] + wSize;
                return 6;
            }
            return 3;
        }
        return 1;
    }
}


//
//  Evaluation methods accessing the local data for a simple regular patch:
//
template <typename REAL>
void
Surface<REAL>::evalRegularBasis(REAL const uv[2], REAL * wDeriv[]) const {

    Far::PatchParam patchParam;
    patchParam.Set(0, 0, 0, 0, 0, getRegPatchMask(), 0, true);

    Far::internal::EvaluatePatchBasisNormalized(
        getRegPatchType(), patchParam, uv[0], uv[1],
        wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);
}

template <typename REAL>
int
Surface<REAL>::evalRegularStencils(REAL const uv[2], REAL * sDeriv[]) const {

    //
    //  The control points of a regular patch are always the full set
    //  of points required by a patch, i.e. phantom points will have an
    //  entry of some kind (a duplicate).  For example, for an isolated
    //  quad, its regular patch still has 16 control points.  So we can
    //  return the basis weights as stencil weights for all cases.
    //
    Far::PatchParam patchParam;
    patchParam.Set(0, 0, 0, 0, 0, getRegPatchMask(), 0, true);

    Far::internal::EvaluatePatchBasisNormalized(
        getRegPatchType(), patchParam, uv[0], uv[1],
        sDeriv[0], sDeriv[1], sDeriv[2], sDeriv[3], sDeriv[4], sDeriv[5]);

    return GetNumControlPoints();
}

template <typename REAL>
void
Surface<REAL>::evalRegularDerivs(REAL const uv[2],
        REAL const patchPoints[], PointDescriptor const & pointDesc,
        REAL * deriv[]) const {

    //
    //  Regular basis evaluation simply returns weights for use with
    //  the entire set of patch control points.
    //
    //  Assign weights for requested derivatives and evaluate:
    //
    REAL   wBuffer[6 * 20];
    REAL * wDeriv[6];

    int numDerivs = assignWeightsPerDeriv(deriv, 20, wBuffer, wDeriv);

    evalRegularBasis(uv, wDeriv);

    //  Assemble the combination parameters and apply:
    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = patchPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = GetNumControlPoints();
    combineArgs.srcPointIndices = 0;

    combineArgs.numResults    = numDerivs;
    combineArgs.resultWeights = wDeriv;
    combineArgs.resultData    = deriv;

    points::Combine(combineArgs);
}

//
//  Evaluation methods accessing the PatchTree for irregular patches:
//
template <typename REAL>
typename Surface<REAL>::IndexArray
Surface<REAL>::evalIrregularBasis(REAL const UV[2], REAL * wDeriv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { UV[0], UV[1] };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    irregPatch.EvalSubPatchBasis(subPatchIndex, uv[0], uv[1],
            wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);

    return irregPatch.GetSubPatchPoints(subPatchIndex);
}

template <typename REAL>
int
Surface<REAL>::evalIrregularStencils(REAL const UV[2], REAL * sDeriv[]) const {

    Parameterization param = GetParameterization();
    REAL uv[2] = { UV[0], UV[1] };
    int subFace = param.HasSubFaces() ?
                  param.ConvertCoordToNormalizedSubFace(uv, uv) : 0;

    internal::IrregularPatchType const & irregPatch = getIrregPatch();
    int subPatchIndex = irregPatch.FindSubPatch(uv[0], uv[1], subFace);
    assert(subPatchIndex >= 0);

    return irregPatch.EvalSubPatchStencils(
            subPatchIndex, uv[0], uv[1],
            sDeriv[0], sDeriv[1], sDeriv[2], sDeriv[3], sDeriv[4], sDeriv[5]);
}

template <typename REAL>
void
Surface<REAL>::evalIrregularDerivs(REAL const uv[2],
        REAL const patchPoints[], PointDescriptor const & pointDesc,
        REAL * deriv[]) const {

    //
    //  Non-linear irregular basis evaluation returns both the weights
    //  and the corresponding points of a sub-patch defined by a subset
    //  of the given patch points.
    //
    //  Assign weights for requested derivatives and evaluate:
    //
    REAL   wBuffer[6 * 20];
    REAL * wDeriv[6];

    int numDerivs = assignWeightsPerDeriv(deriv, 20, wBuffer, wDeriv);

    IndexArray indices = evalIrregularBasis(uv, wDeriv);

    //  Assemble the combination parameters and apply:
    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = patchPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = indices.size();
    combineArgs.srcPointIndices = &indices[0];

    combineArgs.numResults    = numDerivs;
    combineArgs.resultWeights = wDeriv;
    combineArgs.resultData    = deriv;

    points::Combine(combineArgs);
}

//
//  Supporting methods for the N-sided quadrangulated linear patch:
//
namespace {
    //
    //  For stencils, there are four unique weights derived from the
    //  four bilinear weights of the sub-face.  Given these weights as
    //  input for a sub-face with origin at base point P, the resulting
    //  weights are associated with the N base points as follows:
    //
    //      w[0] = the point at the origin (P)
    //      w[1] = the point following P
    //      w[2] = the N-3 points not adjacent to P (contributing to center)
    //      w[3] = the point preceding P
    //
    template <typename REAL>
    inline void
    transformLinearQuadWeightsToStencil(REAL w[4], int N) {

        REAL wOrigin = w[0];
        REAL wNext   = w[1] * 0.5f;
        REAL wCenter = w[2] / (REAL)N;
        REAL wPrev   = w[3] * 0.5f;

        w[0] = wCenter + wNext + wPrev + wOrigin;
        w[1] = wCenter + wNext;
        w[2] = wCenter;
        w[3] = wCenter + wPrev;
    }

    template <typename REAL>
    inline void
    scaleWeights4(REAL w[4], REAL derivScale) {
        if (w) {
            w[0] *= derivScale;
            w[1] *= derivScale;
            w[2] *= derivScale;
            w[3] *= derivScale;
        }
    }
}

template <typename REAL>
int
Surface<REAL>::evalMultiLinearBasis(REAL const UV[2], REAL *wDeriv[]) const {

    Parameterization param = GetParameterization();
    assert(param.GetType() == Parameterization::QUAD_SUBFACES);

    REAL uv[2];
    int subFace = param.ConvertCoordToNormalizedSubFace(UV, uv);

    //  WIP - Prefer to eval Linear basis directly, i.e.:
    //
    //      Far::internal::EvalBasisLinear(u, v, wP, wDu, wDv);
    //
    //  but this internal Far function is sometimes optimized out, causing
    //  link errors.  Need to fix in Far with explicit instantiation...
    Far::internal::EvaluatePatchBasisNormalized(Far::PatchDescriptor::QUADS,
            Far::PatchParam(), uv[0], uv[1],
            wDeriv[0], wDeriv[1], wDeriv[2], wDeriv[3], wDeriv[4], wDeriv[5]);

    //  Scale weights for derivatives (only mixed partial of 2nd is non-zero):
    scaleWeights4<REAL>(wDeriv[1], 2.0f);
    scaleWeights4<REAL>(wDeriv[2], 2.0f);

    scaleWeights4<REAL>(wDeriv[4], 4.0f);

    return subFace;
}

template <typename REAL>
int
Surface<REAL>::evalMultiLinearStencils(REAL const uv[2], REAL *sDeriv[]) const {

    //
    //  Linear evaluation of irregular N-sided faces evaluates one of N
    //  locally subdivided quad faces and identifes that sub-face -- also
    //  the origin vertex of the quad. The basis weights are subsequently
    //  transformed into the four unique values that are then assigned to
    //  the N vertices of the face.
    //
    //  Assign weights for requested stencils and evaluate:
    //
    REAL   wBuffer[6 * 4];
    REAL * wDeriv[6];

    int numDerivs = assignWeightsPerDeriv(sDeriv, 4, wBuffer, wDeriv);

    int iOrigin = evalMultiLinearBasis(uv, wDeriv);

    //
    //  Transform the four linear weights to four unique stencil weights:
    //
    int numControlPoints = GetNumControlPoints();

    transformLinearQuadWeightsToStencil(wDeriv[0], numControlPoints);
    if (numDerivs > 1) {
        transformLinearQuadWeightsToStencil(wDeriv[1], numControlPoints);
        transformLinearQuadWeightsToStencil(wDeriv[2], numControlPoints);
        if (numDerivs > 3) {
            transformLinearQuadWeightsToStencil(wDeriv[4], numControlPoints);
        }
    }

    //
    //  Assign the N stencil weights from the four unique values:
    //
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

        sDeriv[0][i] = wDeriv[0][wIndex];
        if (numDerivs > 1) {
            sDeriv[1][i] = wDeriv[1][wIndex];
            sDeriv[2][i] = wDeriv[2][wIndex];
            if (numDerivs > 3) {
                sDeriv[3][i] = 0.0f;
                sDeriv[4][i] = wDeriv[4][wIndex];
                sDeriv[5][i] = 0.0f;
            }
        }
    }
    return numControlPoints;
}

template <typename REAL>
void
Surface<REAL>::evalMultiLinearDerivs(REAL const uv[],
        REAL const patchPoints[], PointDescriptor const & pointDesc,
        REAL * deriv[]) const {

    //
    //  Linear evaluation of irregular N-sided faces evaluates one of N
    //  locally subdivided quad faces and identifes that sub-face.
    //
    //  Assign weights for requested derivatives and evaluate:
    //
    REAL   wBuffer[6 * 4];
    REAL * wDeriv[6];

    int numDerivs = assignWeightsPerDeriv(deriv, 4, wBuffer, wDeriv);

    int subQuad = evalMultiLinearBasis(uv, wDeriv);

    //
    //  Identify the patch points for the sub-face and interpolate:
    //
    int N = GetNumControlPoints();

    int quadIndices[4];
    quadIndices[0] = subQuad;
    quadIndices[1] = N + 1 + subQuad;
    quadIndices[2] = N;
    quadIndices[3] = N + 1 + (subQuad + N - 1) % N;

    //  Assemble the combination parameters and apply:
    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = patchPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = 4;
    combineArgs.srcPointIndices = quadIndices;

    combineArgs.numResults    = numDerivs;
    combineArgs.resultWeights = wDeriv;
    combineArgs.resultData    = deriv;

    points::Combine(combineArgs);
}


//
//  Public methods to apply stencils:
//
template <typename REAL>
void
Surface<REAL>::ApplyStencil(REAL const stencil[],
        REAL const meshPoints[], PointDescriptor const & pointDesc,
        REAL result[]) const {

    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = meshPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = GetNumControlPoints();
    combineArgs.srcPointIndices = _data.getCVIndices();

    combineArgs.numResults    = 1;
    combineArgs.resultWeights = &stencil;
    combineArgs.resultData    = &result;

    points::Combine(combineArgs);
}

template <typename REAL>
void
Surface<REAL>::ApplyStencilGathered(REAL const stencil[],
        REAL const controlPoints[], PointDescriptor const & pointDesc,
        REAL result[]) const {

    points::CombinationDescriptor<REAL> combineArgs;
    combineArgs.pointData   = controlPoints;
    combineArgs.pointSize   = pointDesc.size;
    combineArgs.pointStride = pointDesc.stride;

    combineArgs.numSrcPoints    = GetNumControlPoints();
    combineArgs.srcPointIndices = 0;

    combineArgs.numResults    = 1;
    combineArgs.resultWeights = &stencil;
    combineArgs.resultData    = &result;

    points::Combine(combineArgs);
}


//
//  Explicitly instantiate Surface<> implementations for float and double:
//
template class Surface<float>;
template class Surface<double>;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
