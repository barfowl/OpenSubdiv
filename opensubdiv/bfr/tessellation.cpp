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

#include <cstdio>
#include <cstring>

#include "../bfr/tessellation.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Tessellation patterns are composed of concentric rings of Coords and
//  Facets -- beginning with the boundary and moving inward.  Each ring
//  of Coords or Facets can be further divided into subsets corresponding
//  to each edge of the face.
//
//  Common and trivial utilities for assembling Coords and Facets:
//
namespace {
    //
    //  Trivial functions for assembling sequences of Coords:
    //
    inline int
    getUIsoLineCoords(int nCoords, float u, float v, float dv, Coord coords[]) {

        for (int i = 0; i < nCoords; ++i, v += dv) {
            coords[i] = Coord(u,v);
        }
        return nCoords;
    }
    inline int
    getVIsoLineCoords(int nCoords, float u, float v, float du, Coord coords[]) {

        for (int i = 0; i < nCoords; ++i, u += du) {
            coords[i] = Coord(u,v);
        }
        return nCoords;
    }
    inline int
    getUVLineCoords(int nCoords,
                    float u, float v, float du, float dv, Coord coords[]) {

        for (int i = 0; i < nCoords; ++i, u += du, v += dv) {
            coords[i] = Coord(u,v);
        }
        return nCoords;
    }

    //
    //  Trivial functions for assembling simple Facets:
    //
    inline int
    getSimpleFacet(int size, int startIndex, Facet facets[]) {

        facets[0] = Facet(startIndex, startIndex+1, startIndex+2,
                          (size == 4) ? (startIndex+3) : -1);
        return 1;
    }

    inline int
    getTriFanFacets(int size, int startIndex, Facet facets[]) {

        for (int i = 1; i <= size; ++i) {
            facets[i-1] = Facet(startIndex + (i - 1),
                                startIndex + ((i < size) ? i : 0),
                                startIndex + size);
        }
        return size;
    }

    //
    //  Useful struct for storing the full topology of a strip of facets
    //  so that they can be connected in various ways:
    //
    //  A strip of facets is defined between an outer and inner ring of
    //  Coords -- denoted as follows, where the "I" and "O" prefixes are
    //  used to designate Coords on the inner and outer rings:
    //
    //    OPrev  ---  IFirst  ... IFirst+/-i ...  ILast    --- ONext
    //      |                                                    |
    //    OFirst --- OFirst+1 ...  OFirst+j ... OFirst+N-1 --- OLast
    //
    //  Since these point formpart of a ring, they will wrap around to
    //  the beginning of the ring for the last edge and so the sequence
    //  is not always sequential.  Note also the "prev" and "next" Coords
    //  of the outer ring -- which preceed and follow the Coords of the
    //  inner ring.
    //
    //  This topological structure is similar but slightly different for
    //  quad-based versus triangular parameterizations.  For quad-based
    //  parameterizations the parametric range of the inner and outer
    //  sequences are the same, but for triangular, the extent of the
    //  inner ring is one edge less (and the triangular domain may be
    //  offset a half edge length so that uniformly spaced points on
    //  both will alternate).
    //
    struct FacetStrip {
    public:
        bool quadTopology;
        bool innerReversed;

        //  Note:  not all of the following members need be set for all
        //  operations -- need to be clearer about some of this...
        int outerEdges;
        int outerFirst, outerLast;

        int innerEdges;
        int innerFirst, innerLast;
        int outerPrev, outerNext;

    public:
        int connectUniformQuads(int n, Facet facets[]) const;
        int connectUniformTris(int n, Facet facets[]) const;

        int connectNonUniformFacets(Facet facets[]) const;
    };

    int
    FacetStrip::connectUniformQuads(int n, Facet facets[]) const {

        assert(quadTopology);
        //
        //  Identify pairs of coords for opposing edges on the outer and
        //  inner rows, iterate forward and assemble:
        //
        int out0 = this->outerFirst;
        int out1 = out0 + 1;

        int in0 = this->outerPrev;
        int in1 = this->innerFirst;
        int inN = this->innerLast;

        int inDelta = this->innerReversed ? -1 : 1;

        facets[0] = Facet(out0++, out1++, in1, in0);

        for (int quad = 1; quad < (n-1); ++quad) {
            in0 = in1;
            in1 = in1 + inDelta;
            facets[quad] = Facet(out0++, out1++, in1, in0);
        }
        if (n > 1) {
            facets[n-1] = Facet(out0, out1, inN, in1);
        }
        return n;
    }

    int
    FacetStrip::connectNonUniformFacets(Facet facets[]) const {

        //
        //  Consider the special case when both inner and outer edges have
        //  the same resolution -- generate triangles only at the corners
        //  and quads between:
        //
        if (quadTopology && (outerEdges == (innerEdges + 2))) {
        }

        //
        //  General case:
        //
        //     *--- in0     ...    in+/-i   ...    inM-2 --*
        //     |    /                                 \    |
        //     |  /                                    \   |
        //     out0 --- L0+1  ...  out+i ... outN-1 --- outN
        //
        //  The sequence of edges -- both inner and outer -- is parameterized
        //  over the integer range [0 .. M*N] where M and N are the resolution
        //  (number of edges) of the inner and outer rings respectively.
        //
        int M = innerEdges + (quadTopology ? 2 : 3);
        int N = outerEdges;

        int dtOuter = M;
        int dtInner = N;

        int tOuterLast   = dtOuter *  N;
        int tOuterMiddle = tOuterLast / 2;

        int tInnerOffset = 0;
        int tInnerLast   = dtInner * (M - 1);

        if (!quadTopology) {
            tInnerOffset = dtInner / 2;
            tInnerLast  += tInnerOffset - dtInner;
        }

        int dInner = innerReversed ? -1 : 1;

        //
        //  Two points are successively identified on each of the inner and
        //  outer sequence of edges, from which facets will be generated:
        //
        //           inner0  inner1
        //              * ----- * . . .
        //             /
        //            /
        //           * ----------- * . . .
        //        outer0        outer1
        //
        //  Identify the parameterization and coordinate indices for the
        //  points starting the sequence:
        //
        int tOuter0 = 0;
        int cOuter0 = outerFirst;

        int tOuter1 = dtOuter;
        int cOuter1 = (N == 1) ? outerLast : (outerFirst + 1);

        int tInner0 = tInnerOffset + dtInner;
        int cInner0 = innerFirst;

        int tInner1 = tInner0 + (innerEdges ? dtInner : 0);
        int cInner1 = (innerEdges == 1) ? innerLast : (innerFirst + dInner);

        //
        //  Walk forward through the strip, identifying each successive quad
        //  and choosing the most "vertical" edge to use to triangulate it:
        //
        //  WIP - eventually want to preserve quads here when reasonable
        //
        int nFacetsExpected = innerEdges + outerEdges;
        int nFacets = 0;

        while (nFacets < nFacetsExpected) {
            bool generateTriFromOuterEdge = false;
            bool generateTriFromInnerEdge = false;

            if (tInner1 == tInner0) {
                generateTriFromOuterEdge = true;
            } else if (tOuter1 == tOuter0) {
                generateTriFromInnerEdge = true;
            } else {
                //  Choose the edge spanning the shortest parametric interval
                //  (when equal, choose relative to midpoint for symmetry):
                int dtInner0ToOuter1 = tOuter1 - tInner0;
                int dtOuter0ToInner1 = tInner1 - tOuter0;

                bool useInner0ToOuter1 = (dtInner0ToOuter1 == dtOuter0ToInner1)
                                       ? (tOuter1 > tOuterMiddle)
                                       : (dtInner0ToOuter1 < dtOuter0ToInner1);
                if (useInner0ToOuter1) {
                    generateTriFromOuterEdge = true;
                } else {
                    generateTriFromInnerEdge = true;
                }
            }

            if (generateTriFromOuterEdge) {
                facets[nFacets++] = Facet(cOuter0, cOuter1, cInner0);

                //  Advance to the next point of the next outer edge:
                tOuter0 = tOuter1;
                cOuter0 = cOuter1;

                tOuter1 += dtOuter;
                if (tOuter1 < tOuterLast) {
                    cOuter1 = cOuter1 + 1;
                } else {
                    tOuter1 = tOuterLast;
                    cOuter1 = outerLast;
                }
            }
            if (generateTriFromInnerEdge) {
                facets[nFacets++] = Facet(cInner1, cInner0, cOuter0);

                //  Advance to the next point of the next inner edge:
                tInner0 = tInner1;
                cInner0 = cInner1;

                tInner1 += dtInner;
                if (tInner1 < tInnerLast) {
                    cInner1  = cInner1 + dInner;
                } else {
                    tInner1  = tInnerLast;
                    cInner1  = innerLast;
                }
            }
        }
        return nFacets;
    }

    int
    FacetStrip::connectUniformTris(int n, Facet facets[]) const {

        assert(!quadTopology);
        //
        //  Assign the set of tris for the "sawtooth" strip with N lower and
        //  N-1 upper tris with the following vertex indices in the two rows:
        //
        //      U0 ---- U1 ---- U2 ..  Ui  .. UN
        //     / 2\1  0/ 2\    /             / 2\.
        //    /0  1\2 /0  1\  /             /0  1\.
        //   L0 -- L0+1 .. L0+2 ..  Li  .. * --- L0+n
        //
        //  First assign the leading triangle {L0,L1,U0} then assign pairs
        //  of successive triangles (after incrementing L0, L1) from the quad
        //  {L0,L1,U1,U0}.  The bases of all tris assigned are intended to
        //  lie on edges of either the upper or lower rows, not between.
        //
        int L0 = outerFirst;
        int L1 = L0 + 1;

        int U0 = outerPrev;
        int U1 = innerFirst;
        int UN = innerLast;

        int nFacets = 0;
        facets[nFacets++] = Facet(L0++, L1++, U0);

        for (int i = 1; i < (n-1); ++i, ++L0, ++L1, ++U1) {
            facets[nFacets++] = Facet(U1, U0, L0);
            facets[nFacets++] = Facet(L0, L1, U1);

            //  U0 and U1 not initially successive so assign don't increment:
            U0 = U1;
        }
        facets[nFacets++] = Facet(UN, U0, L0);
        facets[nFacets++] = Facet(L0, L1, UN);

        return nFacets;
    }
}

//
//  Utility functions to help assembly of tessellation patterns -- grouped
//  into local structs/namespaces for each of the supported parameterization
//  types:  quad, triangle (tri) or quadranglated N-sided polygon (qpoly):
//
//  Given the similar structure to these -- the construction of patterns
//  using concentric rings of Coords, rings of Facets between successive
//  concentric rings, etc. -- there are some opportunities for refactoring
//  some of these.  (But there are typically subtle differences between
//  each that complicate doing so.)
//
struct quad {
    //  Higher level methods supporting Tessellation:
    static int countUniformFacets(int edgeRes);
    static int countNonUniformFacets(int nBoundaryEdges, int uRes, int vRes);

    static int countUniformCoords(int edgeRes);
    static int countInteriorCoords(int edgeRes);
    static int countInteriorCoords(int uRes, int vRes);

    static int getBoundaryEdgeCoords(int edge, int edgeRes,
                                     bool v0, bool v1, Coord coords[]);
    static int getBoundaryCoords(int const edgeRates[], Coord coords[]);
    static int getInteriorCoords(int const uvRes[2], Coord coords[]);

    static int getUniformFacets(int uniformRes, Facet facets[]);
    static int getNonUniformFacets(int const outerRes[], int const innerRes[],
                                   int nBoundaryEdges, Facet facets[]);

    //  Lower level methods used by those above:
    static int getCenterCoord(Coord coords[]);

    static int getInteriorRingCoords(int   uRes,   int   vRes,
                                     float uStart, float vStart,
                                     float uDelta, float vDelta,
                                     Coord coords[]);

    static int getInteriorRingFacets(int   uRes, int vRes,
                                     int   indexOfFirstCoord,
                                     Facet facets[]);

    static int getBoundaryRingFacets(int const outerRes[], int uRes, int vRes,
                                     int       nBoundaryEdges,
                                     Facet     facets[]);
};

struct tri {
    //  Higher level methods supporting Tessellation:
    static int countUniformFacets(int edgeRes);
    static int countNonUniformFacets(int nBoundaryEdges, int innerRes);

    static int countUniformCoords(int edgeRes);
    static int countInteriorCoords(int edgeRes);

    static int getBoundaryEdgeCoords(int edge, int edgeRes,
                                     bool v0, bool v1, Coord coords[]);
    static int getBoundaryCoords(int const edgeRates[], Coord coords[]);
    static int getInteriorCoords(int edgeRes, Coord coords[]);

    static int getUniformFacets(int uniformRes, Facet facets[]);
    static int getNonUniformFacets(int const outerRes[], int innerRes,
                                   int nBoundaryEdges, Facet facets[]);

    //  Lower level methods used by those above:
    static int getCenterCoord(Coord coords[]);

    static int getInteriorRingCoords(int   edgeRes,
                                     float uStart, float vStart,
                                     float tDelta,
                                     Coord coords[]);

    static int getInteriorRingFacets(int   edgeRes,
                                     int   indexOfFirstCoord,
                                     Facet facets[]);

    static int getBoundaryRingFacets(int const outerRes[], int innerRes,
                                     int       nBoundaryEdges,
                                     Facet     facets[]);
};

struct qpoly {
    //  Higher level methods supporting Tessellation:
    static int countUniformFacets(int N, int edgeRes);
    static int countNonUniformFacets(int N, int nBoundaryEdges, int innerRes);

    static int countUniformCoords(int N, int edgeRes);
    static int countInteriorCoords(int N, int edgeRes);

    static int getBoundaryEdgeCoords(int N, int edge,
                                     int edgeRes, bool incFirst, bool incLast,
                                     Coord coords[]);
    static int getBoundaryCoords(int N, int const edgeRates[], Coord coords[]);
    static int getInteriorCoords(int N, int edgeRes, Coord coords[]);

    static int getUniformFacets(int N, int uniformRes, Facet facets[]);
    static int getNonUniformFacets(int N, int const outerRes[], int innerRes,
                                   int nBoundaryEdges, Facet facets[]);

    //  Lower level methods used by those above:
    static int getCenterCoord(Coord coords[]);
    static int getCenterRingCoords(int N, float tStart, Coord coords[]);

    static int getRingEdgeCoords(int edgeRes, bool incFirst, bool incLast,
                                 float uCorner0, float uCorner1,
                                 float tStart, float tDelta,
                                 Coord coords[]);
    static int getInteriorRingCoords(int N, int edgeRes,
                                     float tStart, float tDelta,
                                     Coord coords[]);

    static int getCenterFacets(int N, int indexOfFirstCoord, Facet facets[]);

    static int getInteriorRingFacets(int N, int edgeRes, int indexOfFirstCoord,
                                     Facet facets[]);

    static int getBoundaryRingFacets(int N, int const outerRes[], int innerRes,
                                     int nBoundaryEdges, Facet facets[]);
};

//
//  Implementations for quad functions:
//
inline int
quad::countUniformFacets(int edgeRes) {
    return edgeRes * edgeRes;
}

inline int
quad::countNonUniformFacets(int nBoundaryEdges, int uRes, int vRes) {

    int innerUEdges = std::max(uRes - 2, 0);
    int innerVEdges = std::max(vRes - 2, 0);

    int nInterior = innerUEdges * innerVEdges;
    int nBoundary = (innerUEdges + innerVEdges) * 2 + nBoundaryEdges;

    return nInterior + nBoundary;
}

inline int
quad::countUniformCoords(int edgeRes) {
    return (edgeRes + 1) * (edgeRes + 1);
}

inline int
quad::countInteriorCoords(int uniformRes) {
    return countUniformCoords(uniformRes - 2);
}

inline int
quad::countInteriorCoords(int uRes, int vRes) {
    return std::max(uRes - 1, 1) * std::max(vRes - 1, 1);
}

inline int
quad::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(0.5f, 0.5f);
    return 1;
}

int
quad::getBoundaryEdgeCoords(int edge, int edgeRes, bool v0, bool v1,
                            Coord coords[]) {

    float dt = 1.0 / (float)edgeRes;

    float t0 = v0 ? 0.0f : dt;
    float t1 = v0 ? 1.0f : (1.0f - dt);

    int nCoords = edgeRes - 1 + v0 + v1;

    switch (edge) {
    case 0:  return getVIsoLineCoords(nCoords, t0,   0.0f,  dt, coords);
    case 1:  return getUIsoLineCoords(nCoords, 1.0f, t0,    dt, coords);
    case 2:  return getVIsoLineCoords(nCoords, t1,   1.0f, -dt, coords);
    case 3:  return getUIsoLineCoords(nCoords, 0.0f, t1,   -dt, coords);
    }
    return 0;
}

int
quad::getBoundaryCoords(int const edgeRates[], Coord coords[]) {

    int nCoords = 0;
    nCoords += getVIsoLineCoords(edgeRates[0], 0.0f, 0.0f,
                      1.0/(float)edgeRates[0], coords + nCoords);
    nCoords += getUIsoLineCoords(edgeRates[1], 1.0f, 0.0f,
                      1.0/(float)edgeRates[1], coords + nCoords);
    nCoords += getVIsoLineCoords(edgeRates[2], 1.0f, 1.0f,
                     -1.0/(float)edgeRates[2], coords + nCoords);
    nCoords += getUIsoLineCoords(edgeRates[3], 0.0f, 1.0f,
                     -1.0/(float)edgeRates[3], coords + nCoords);
    return nCoords;
}

int
quad::getInteriorRingCoords(int uRes, int vRes,
                            float u0, float v0, float du, float dv,
                            Coord coords[]) {

    int nCoords = 0;
    if ((uRes > 0) && (vRes > 0)) {
        float u1 = 1.0f - u0;
        float v1 = 1.0f - v0;

        nCoords += getVIsoLineCoords(uRes, u0, v0,  du, coords + nCoords);
        nCoords += getUIsoLineCoords(vRes, u1, v0,  dv, coords + nCoords);
        nCoords += getVIsoLineCoords(uRes, u1, v1, -du, coords + nCoords);
        nCoords += getUIsoLineCoords(vRes, u0, v1, -dv, coords + nCoords);
    } else if (uRes > 0) {
        nCoords += getVIsoLineCoords(uRes+1, u0, v0, du, coords);
    } else if (vRes > 0) {
        nCoords += getUIsoLineCoords(vRes+1, u0, v0, dv, coords);
    } else {
        return getCenterCoord(coords);
    }
    return nCoords;
}

inline int
quad::getInteriorCoords(int const uvRes[2], Coord coords[]) {

    int nIntRings = std::min((uvRes[0] / 2), (uvRes[1] / 2));
    if (nIntRings == 0) return 0;

    float du = 1.0 / (float)uvRes[0];
    float dv = 1.0 / (float)uvRes[1];
    float u  = du;
    float v  = dv;

    int uRes = uvRes[0] - 2;
    int vRes = uvRes[1] - 2;

    //
    //  Note that with separate U and V res, one can go negative so beware
    //  of making any assumptions -- defer to the function for the ring:
    //
    int nCoords = 0;
    for (int i=0; i < nIntRings; ++i, uRes -= 2, vRes -= 2, u += du, v += dv) {
        nCoords += getInteriorRingCoords(uRes, vRes, u, v, du, dv,
                                         &coords[nCoords]);
    }
    return nCoords;
}

int
quad::getInteriorRingFacets(int uRes, int vRes, int coord0, Facet facets[]) {

    assert((uRes >= 0) && (vRes >= 0));

    int totalInnerFacets = uRes * vRes;
    if (totalInnerFacets == 0) return 0;

    if (totalInnerFacets == 1) {
        facets[0] = Facet(coord0, coord0+1, coord0+2, coord0+3);
        return 1;
    }

    //
    //  Deal with case of single quad strip (rather than a succession of
    //  four strips) when there is no interior ring of vertices:
    //
    FacetStrip qStrip;
    qStrip.quadTopology = true;

    if ((uRes == 1) || (vRes == 1)) {
        qStrip.innerReversed = true;

        if (uRes > 1) {
            qStrip.outerFirst = coord0;
            qStrip.outerPrev  = coord0 + 2*uRes + 1;
            qStrip.innerFirst = qStrip.outerPrev - 1;
            qStrip.innerLast  = qStrip.outerPrev - uRes;

            return qStrip.connectUniformQuads(uRes, facets);
        } else {
            qStrip.outerFirst = coord0 + 1;
            qStrip.outerPrev  = coord0;
            qStrip.innerFirst = qStrip.outerFirst + 2*vRes;
            qStrip.innerLast  = qStrip.innerFirst - vRes + 1;

            return qStrip.connectUniformQuads(vRes, facets);
        }
    }

    //
    //  The general case -- one or more quads for each edge that are
    //  connected to the next interior ring of vertices:
    //
    int nFacets = 0;

    int uResInner = uRes - 2;
    int vResInner = vRes - 2;

    int outerRingStart = coord0;
    int innerRingStart = coord0 + 2 * (uRes + vRes);

    qStrip.outerFirst = outerRingStart;
    qStrip.outerPrev  = innerRingStart - 1;
    qStrip.innerReversed = false;
    qStrip.innerFirst    = innerRingStart;
    qStrip.innerLast     = innerRingStart + uResInner;
    nFacets += qStrip.connectUniformQuads(uRes-1, facets + nFacets);

    qStrip.outerFirst += uRes;
    qStrip.outerPrev   = qStrip.outerFirst - 1;
    qStrip.innerReversed = false;
    qStrip.innerFirst   += uResInner;
    qStrip.innerLast    += vResInner;
    nFacets += qStrip.connectUniformQuads(vRes-1, facets + nFacets);

    qStrip.outerFirst += vRes;
    qStrip.outerPrev   = qStrip.outerFirst - 1;
    qStrip.innerReversed = (vResInner == 0);
    qStrip.innerFirst   += vResInner;
    qStrip.innerLast    += uResInner * (qStrip.innerReversed ? -1 : 1);
    nFacets += qStrip.connectUniformQuads(uRes-1, facets + nFacets);

    qStrip.outerFirst += uRes;
    qStrip.outerPrev   = qStrip.outerFirst - 1;
    qStrip.innerReversed = (uResInner == 0);
    qStrip.innerFirst   += uResInner * (qStrip.innerReversed ? -1 : 1);
    qStrip.innerLast     = innerRingStart;
    nFacets += qStrip.connectUniformQuads(vRes-1, facets + nFacets);

    return nFacets;
}

int
quad::getBoundaryRingFacets(int const outerRes[], int uRes, int vRes,
                            int nBoundaryEdges, Facet facets[]) {

    uRes = std::max(uRes - 2, 0);
    vRes = std::max(vRes - 2, 0);

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    FacetStrip qStrip;
    qStrip.quadTopology  = true;

    qStrip.outerEdges = outerRes[0];
    qStrip.outerFirst = outerRingStart;
    qStrip.outerLast  = outerRingStart + outerRes[0];
    qStrip.innerReversed = false;
    qStrip.innerEdges    = uRes;
    qStrip.innerFirst    = innerRingStart;
    qStrip.innerLast     = innerRingStart + uRes;
    nFacets += qStrip.connectNonUniformFacets(facets + nFacets);

    qStrip.outerEdges = outerRes[1];
    qStrip.outerFirst = qStrip.outerLast;
    qStrip.outerLast += outerRes[1];
    qStrip.innerReversed = false;
    qStrip.innerEdges    = vRes;
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += vRes;
    nFacets += qStrip.connectNonUniformFacets(facets + nFacets);

    qStrip.outerEdges = outerRes[2];
    qStrip.outerFirst = qStrip.outerLast;
    qStrip.outerLast += outerRes[2];
    qStrip.innerReversed = (vRes == 0);
    qStrip.innerEdges    = uRes;
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += uRes * (qStrip.innerReversed ? -1 : 1);
    nFacets += qStrip.connectNonUniformFacets(facets + nFacets);

    qStrip.outerEdges = outerRes[3];
    qStrip.outerFirst = qStrip.outerLast;
    qStrip.outerLast  = 0;
    qStrip.innerReversed = (uRes == 0);
    qStrip.innerEdges    = vRes;
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast     = innerRingStart;
    nFacets += qStrip.connectNonUniformFacets(facets + nFacets);

    return nFacets;
}

int
quad::getNonUniformFacets(int const outerRes[], int const innerRes[],
                          int nBoundaryEdges, Facet facets[]){

    int uRes = innerRes[0];
    int vRes = innerRes[1];

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(outerRes, uRes, vRes, nBoundaryEdges,
                                        facets);

    //  Second, generate the remaining rings of interior facets:
    int nRings = (std::min(uRes,vRes) + 1) / 2;
    int coord0 = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        uRes = std::max(uRes - 2, 0);
        vRes = std::max(vRes - 2, 0);

        nFacets += getInteriorRingFacets(uRes, vRes, coord0, facets + nFacets);
        coord0  += 2 * (uRes + vRes);
    }
    return nFacets;
}

int
quad::getUniformFacets(int res, Facet facets[]) {

    //  The trivial case should have been handled by the caller:
    assert(res > 1);

    int nRings = (res + 1) / 2;

    int nFacets = 0;
    int coord0 = 0;
    for (int ring = 0; ring < nRings; ++ring, res -= 2) {
        nFacets += getInteriorRingFacets(res, res, coord0, facets + nFacets);
        coord0  += 4 * res;
    }
    return nFacets;
}


//
//  REMINDER TO SELF -- according to the OpenGL docs, the "inner" tess
//  rates are expected to reflect a tessellation of the entire face, i.e.
//  they are not the outer rates with 2 subtracted, but are the same as
//  the outer rates.  Their minimum is therefore 1 -- no inner vertices,
//  BUT any non-unit outer rate will trigger an interior point.
//
//  Note that triangles will need considerably different treatment in
//  some cases given the way we diverge from the OpenGL patterns, e.g.
//  the corner faces are not bisected in the uniform case but may need
//  to be when non-uniform.
//

//
//  Implementations for tri functions:
//
inline int
tri::countUniformFacets(int edgeRes) {
    return edgeRes * edgeRes;
}

inline int
tri::countNonUniformFacets(int nBoundaryEdges, int innerRes) {

    int nInnerEdges = std::max(innerRes - 3, 0);

    int nInterior = nInnerEdges ? countUniformFacets(nInnerEdges) : 0;
    int nBoundary = nInnerEdges * 3 + nBoundaryEdges;

    return nInterior + nBoundary;
}

inline int
tri::countUniformCoords(int edgeRes) {
    return edgeRes * (edgeRes + 1) / 2;
}

inline int
tri::countInteriorCoords(int edgeRes) {
    return countUniformCoords(edgeRes - 2);
}

inline int
tri::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(1.0f/3.0f, 1.0f/3.0f);
    return 1;
}

int
tri::getBoundaryEdgeCoords(int edge, int edgeRes, bool v0, bool v1,
                           Coord coords[]) {

    float dt = 1.0 / (float)edgeRes;

    float t0 = v0 ? 0.0f : dt;
    float t1 = v0 ? 1.0f : (1.0f - dt);

    int nCoords = edgeRes - 1 + v0 + v1;

    switch (edge) {
    case 0:  return getVIsoLineCoords(nCoords, t0,  0.0f, dt, coords);
    case 1:  return getUVLineCoords(  nCoords, t1,  t0,  -dt, dt, coords);
    case 2:  return getUIsoLineCoords(nCoords, 0.0, t1,  -dt, coords);
    }
    return 0;
}

int
tri::getBoundaryCoords(int const edgeRates[], Coord coords[]) {

    int nCoords = 0;
    nCoords += getVIsoLineCoords(edgeRates[0], 0.0f, 0.0f,
                      1.0/(float)edgeRates[0], coords + nCoords);
    nCoords += getUVLineCoords(edgeRates[1], 1.0f, 0.0f,
                     -1.0/(float)edgeRates[1],
                      1.0/(float)edgeRates[1], coords + nCoords);
    nCoords += getUIsoLineCoords(edgeRates[2], 0.0f, 1.0f,
                     -1.0/(float)edgeRates[2], coords + nCoords);
    return nCoords;
}

int
tri::getInteriorRingCoords(int edgeRes, float u0, float v0, float dt,
                      Coord coords[]) {
    assert(edgeRes);

    float u1 = 1.0f - u0*2.0f;
    float v1 = 1.0f - v0*2.0f;

    int nCoords = 0;
    nCoords += getVIsoLineCoords(edgeRes, u0, v0,  dt, coords + nCoords);
    nCoords += getUVLineCoords(  edgeRes, u1, v0, -dt, dt, coords + nCoords);
    nCoords += getUIsoLineCoords(edgeRes, u0, v1, -dt, coords + nCoords);
    return nCoords;
}

int
tri::getInteriorCoords(int edgeRes, Coord coords[]) {

    int nIntRings = edgeRes / 3;
    if (nIntRings == 0) return 0;

    float dt = 1.0 / (float)edgeRes;
    float u  = dt;
    float v  = dt;

    int ringRes = edgeRes - 3;

    int nCoords = 0;
    for (int i = 0; i < nIntRings; ++i, ringRes -= 3, u += dt, v += dt) {
        if (ringRes == 0) {
            nCoords += getCenterCoord(&coords[nCoords]);
        } else {
            nCoords += getInteriorRingCoords(ringRes, u, v, dt,
                                             &coords[nCoords]);
        }
    }
    return nCoords;
}

int
tri::getInteriorRingFacets(int edgeRes, int coord0, Facet facets[]) {

    //
    //  Deal with trivial cases with no inner vertices:
    //
    if (edgeRes < 1) {
        return 0;
    } else if (edgeRes == 1) {
        facets[0] = Facet(coord0, coord0+1, coord0+2);
        return 1;
    } else if (edgeRes == 2) {
        facets[0] = Facet(coord0+0, coord0+1, coord0+5);
        facets[1] = Facet(coord0+2, coord0+3, coord0+1);
        facets[2] = Facet(coord0+4, coord0+5, coord0+3);
        facets[3] = Facet(coord0+1, coord0+3, coord0+5);
        return 4;
    }

    //
    //  Generate facets for the 3 tri-strips for each edge:
    //
    int nFacets = 0;

    int outerRes = edgeRes;
    int innerRes = edgeRes - 3;

    int outerRingStart = coord0;
    int innerRingStart = coord0 + 3 * outerRes;

    FacetStrip tStrip;
    tStrip.quadTopology  = false;
    tStrip.innerReversed = false;

    tStrip.outerFirst = outerRingStart;
    tStrip.outerPrev  = innerRingStart - 1;
    tStrip.innerFirst = innerRingStart;
    tStrip.innerLast  = innerRingStart + innerRes;
    nFacets += tStrip.connectUniformTris(outerRes-1, facets + nFacets);

    tStrip.outerFirst += outerRes;
    tStrip.outerPrev   = tStrip.outerFirst - 1;
    tStrip.innerFirst += innerRes;
    tStrip.innerLast  += innerRes;
    nFacets += tStrip.connectUniformTris(outerRes-1, facets + nFacets);

    tStrip.outerFirst += outerRes;
    tStrip.outerPrev   = tStrip.outerFirst - 1;
    tStrip.innerFirst += innerRes;
    tStrip.innerLast   = innerRingStart;
    nFacets += tStrip.connectUniformTris(outerRes-1, facets + nFacets);

    return nFacets;
}

int
tri::getBoundaryRingFacets(int const outerRes[], int innerRes,
                           int nBoundaryEdges, Facet facets[]) {

    innerRes = std::max(innerRes - 3, 0);

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    FacetStrip tStrip;
    tStrip.quadTopology  = false;
    tStrip.innerReversed = false;
    tStrip.innerEdges    = innerRes;

    tStrip.outerEdges = outerRes[0];
    tStrip.outerFirst = outerRingStart;
    tStrip.outerLast  = outerRingStart + outerRes[0];
    tStrip.innerFirst = innerRingStart;
    tStrip.innerLast  = innerRingStart + innerRes;
    nFacets += tStrip.connectNonUniformFacets(facets + nFacets);

    tStrip.outerEdges  = outerRes[1];
    tStrip.outerFirst  = tStrip.outerLast;
    tStrip.outerLast  += outerRes[1];
    tStrip.innerFirst  = tStrip.innerLast;
    tStrip.innerLast  += innerRes;
    nFacets += tStrip.connectNonUniformFacets(facets + nFacets);

    tStrip.outerEdges = outerRes[2];
    tStrip.outerFirst = tStrip.outerLast;
    tStrip.outerLast  = 0;
    tStrip.innerFirst = tStrip.innerLast;
    tStrip.innerLast  = innerRingStart;
    nFacets += tStrip.connectNonUniformFacets(facets + nFacets);

    return nFacets;
}

int
tri::getUniformFacets(int edgeRes, Facet facets[]) {

    //  The trivial case should have been handled by the caller:
    assert(edgeRes > 1);

    int nRings = 1 + (edgeRes / 3);

    int nFacets = 0;
    int coord0  = 0;
    for (int ring = 0; ring < nRings; ++ring, edgeRes -= 3) {
        nFacets += getInteriorRingFacets(edgeRes, coord0, facets + nFacets);
        coord0  += 3 * edgeRes;
    }
    return nFacets;
}

int
tri::getNonUniformFacets(int const outerRes[], int innerRes,
                         int nBoundaryEdges, Facet facets[]) {

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(outerRes, innerRes, nBoundaryEdges,
                                        facets);

    //  Second, generate the remaining rings of interior facets:
    int nRings = 1 + (innerRes / 3);
    int coord0  = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        innerRes -= 3;

        nFacets += getInteriorRingFacets(innerRes, coord0, facets + nFacets);
        coord0  += 3 * innerRes;
    }
    return nFacets;
}


//
//  These utilities support quadrangulated polygons used for quad-based
//  subdivision schemes.
//

//
//  The formulae to enumerate points and facets for a uniform tessellation
//  reflect the differing topologies for the odd and even case:
//
inline int
qpoly::countUniformFacets(int N, int edgeRes) {

    int H = edgeRes / 2;
    return (edgeRes & 1) ? (H+1)* H * N + ((N == 3) ? 1 : N)
                         :   H  * H * N;
}

inline int
qpoly::countNonUniformFacets(int N, int nBoundaryEdges, int innerRes) {

    int nInnerEdges = std::max(innerRes - 2, 0);

    int nInterior = nInnerEdges ? countUniformFacets(N, nInnerEdges) : 0;
    int nBoundary = nInnerEdges * N + nBoundaryEdges;

    return nInterior + nBoundary;
}

inline int
qpoly::countUniformCoords(int N, int edgeRes) {

    int H = edgeRes / 2;
    return (edgeRes & 1) ? (H+1)* (H+1) * N + ((N == 3) ? 0 : 1)
                         :   H  * (H+1) * N + 1;
}

inline int
qpoly::countInteriorCoords(int N, int edgeRes) {

    return (edgeRes > 1) ? countUniformCoords(N, edgeRes - 2)
                         : ((N == 3) ? 0 : 1);
}

inline int
qpoly::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(0.5f, 0.5f);
    return 1;
}

inline int
qpoly::getRingEdgeCoords(int edgeRes, bool incFirst, bool incLast,
                        float uCorner0, float uCorner1,
                        float tStart, float dt, Coord coords[]) {

    //
    //  Determine number of coords in each half, excluding the ends.  The
    //  second half will get the extra when odd so that the sequence starts
    //  exactly on the boundary of the second sub-face (avoiding floating
    //  point error when accumulating to the boundary of the first):
    //
    int n0 = (edgeRes - 1) / 2;
    int n1 = (edgeRes - 1) - n0;

    int nCoords = 0;
    if (incFirst) {
        coords[nCoords++] = Coord(uCorner0 + tStart, tStart);
    }
    if (n0) {
        float u0 = uCorner0 + tStart + dt;
        float v0 = tStart;
        nCoords += getVIsoLineCoords(n0, u0, v0, dt, coords + nCoords);
    }
    if (n1) {
        float u1 = uCorner1 + tStart;
        float v1 = (edgeRes & 1) ? (0.5f - 0.5f * dt) : 0.5f;
        nCoords += getUIsoLineCoords(n1, u1, v1, -dt, coords + nCoords);
    }
    if (incLast) {
        coords[nCoords++] = Coord(uCorner1 + tStart, tStart);
    }
    return nCoords;
}

int
qpoly::getBoundaryEdgeCoords(int N, int edge,
                            int edgeRes, bool inc0, bool inc1,
                            Coord coords[]) {

    float u0 = (float)edge;
    float u1 = (edge < (N-1)) ? (u0 + 1.0f) : 0.0f;

    return getRingEdgeCoords(edgeRes, inc0, inc1, u0, u1,
                             0.0f, 1.0f / (float)edgeRes,
                             coords);
}

int
qpoly::getBoundaryCoords(int N, int const edgeRates[], Coord coords[]) {

    int nCoords = 0;

    float u0 = 0.0f;
    for (int i = 0; i < N; ++i, u0 += 1.0f) {
        float u1 = (i < (N-1)) ? (u0 + 1.0f) : 0.0f;

        nCoords += getRingEdgeCoords(edgeRates[i], true, false, u0, u1,
                                     0.0f, 1.0f / (float)edgeRates[i],
                                     coords + nCoords);
    }
    return nCoords;
}

int
qpoly::getInteriorRingCoords(int N, int edgeRes, float tStart, float dt,
                            Coord coords[]) {
    assert(edgeRes > 1);

    int nCoords = 0;

    float u0 = 0.0f;
    for (int i = 0; i < N; ++i, u0 += 1.0f) {
        float u1 = (i < (N-1)) ? (u0 + 1.0f) : 0.0f;

        nCoords += getRingEdgeCoords(edgeRes, true, false, u0, u1,
                                     tStart, dt,
                                     coords + nCoords);
    }
    return nCoords;
}

int
qpoly::getCenterRingCoords(int N, float tStart, Coord coords[]) {

    for (int i = 0; i < N; ++i) {
        coords[i] = Coord((float)i + tStart, tStart);
    }
    return (N == 3) ? N : (N + getCenterCoord(coords + N));
}

inline int
qpoly::getInteriorCoords(int N, int edgeRes, Coord coords[]) {

    int nIntRings = edgeRes / 2;
    if (nIntRings == 0) return 0;

    float dt = 1.0 / (float)edgeRes;
    float t  = dt;

    int ringRes = edgeRes - 2;

    int nCoords = 0;
    for (int i = 0; i < nIntRings; ++i, ringRes -= 2, t += dt) {
        if (ringRes == 0) {
            nCoords += getCenterCoord(&coords[nCoords]);
        } else if (ringRes == 1) {
            nCoords += getCenterRingCoords(N, t, &coords[nCoords]);
        } else {
            nCoords += getInteriorRingCoords(N, ringRes, t, dt,
                                             &coords[nCoords]);
        }
    }
    return nCoords;
}

int
qpoly::getCenterFacets(int N, int coord0, Facet facets[]) {

    return (N == 3) ? getSimpleFacet(3, coord0, facets)
                    : getTriFanFacets(N, coord0, facets);
}

int
qpoly::getInteriorRingFacets(int N, int edgeRes, int coord0, Facet facets[]) {

    //
    //  Deal with trivial cases with no inner vertices:
    //
    if (edgeRes < 1) return 0;

    if (edgeRes == 1) {
        return getCenterFacets(N, coord0, facets);
    }

    //
    //  Generate facets for the N quad-strips for each edge:
    //
    int outerRes  = edgeRes;
    int outerRing = coord0;

    int innerRes  = outerRes - 2;
    int innerRing = outerRing + N * outerRes;

    int nFacets = 0;

    FacetStrip qStrip;
    qStrip.quadTopology  = true;
    qStrip.innerReversed = false;

    for (int edge = 0; edge < N; ++edge) {
        qStrip.outerFirst = outerRing + edge * outerRes;
        qStrip.innerFirst = innerRing + edge * innerRes;

        qStrip.outerPrev = (edge > 0) ? (qStrip.outerFirst - 1) :
                                        (qStrip.innerFirst - 1);
        qStrip.innerLast = (edge < N-1) ? (qStrip.innerFirst + innerRes) :
                                          innerRing;

        nFacets += qStrip.connectUniformQuads(innerRes + 1, facets + nFacets);
    }
    return nFacets;
}

int
qpoly::getBoundaryRingFacets(int N, int const outerRes[], int innerRes,
                             int nBoundaryEdges, Facet facets[]) {

    innerRes = std::max(innerRes - 2, 0);

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    FacetStrip qStrip;
    qStrip.quadTopology  = true;
    qStrip.innerReversed = false;

    qStrip.outerFirst = outerRingStart;
    qStrip.outerEdges = outerRes[0];
    qStrip.outerLast  = outerRingStart + outerRes[0];

    qStrip.innerFirst = innerRingStart;
    qStrip.innerEdges = innerRes;
    qStrip.innerLast  = innerRingStart + innerRes;

    for (int edge = 0; edge < N; ++edge) {
        if (edge) {
            qStrip.outerEdges = outerRes[edge];

            qStrip.outerFirst = qStrip.outerLast;
            qStrip.innerFirst = qStrip.innerLast;
            if (edge < N-1) {
                qStrip.outerLast += qStrip.outerEdges;
                qStrip.innerLast += innerRes;
            } else {
                qStrip.outerLast = outerRingStart;
                qStrip.innerLast = innerRingStart;
            }
        }
        nFacets += qStrip.connectNonUniformFacets(facets + nFacets);
    }
    return nFacets;
}
    
int
qpoly::getUniformFacets(int N, int edgeRes, Facet facets[]) {

    //  The trivial (single facet) case should be handled externally:
    if (edgeRes == 1) {
        return getCenterFacets(N, 0, facets);
    }

    int nRings = (edgeRes + 1) / 2;

    int nFacets = 0;
    int coord0  = 0;
    for (int ring = 0; ring < nRings; ++ring, edgeRes -= 2) {
        nFacets += getInteriorRingFacets(N, edgeRes, coord0, facets + nFacets);
        coord0  += N * edgeRes;
    }
    return nFacets;
}

int
qpoly::getNonUniformFacets(int N, int const outerRes[], int innerRes,
                           int nBoundaryEdges, Facet facets[]){

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(N, outerRes, innerRes, nBoundaryEdges,
                                        facets);

    //  Second, generate the remaining rings of interior facets:
    int nRings = (innerRes + 1) / 2;
    int coord0  = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        innerRes = std::max(innerRes - 2, 0);

        nFacets += getInteriorRingFacets(N, innerRes, coord0, facets + nFacets);
        coord0  += N * innerRes;
    }
    return nFacets;
}


//
//
//
inline void
Tessellation::initialize(Parameterization p, int numRates, int const rates[],
                         Options /* options */) {

    //  Members related to the parameterization:
    _param = p;

    //  Members related to tessellation rates:
    int N = _param.GetFaceSize();
    if (N > 4) {
        _outerRatesDynamic.resize(N);
        _outerRates = &_outerRatesDynamic[0];
    } else {
        _outerRates = &_outerRatesLocal[0];
    }

    int numBoundaryEdges = 0;
    if (numRates < N) {
        _isUniform = true;

        std::fill(_outerRates, _outerRates + N, rates[0]);
        _innerRates[0] = rates[0];
        _innerRates[1] = rates[0];

        numBoundaryEdges = rates[0] * N;
    } else {
        _isUniform = true;  // will be marked false below if warranted
        for (int i = 0; i < N; ++i) {
            _outerRates[i] = rates[i];
            _isUniform &= (rates[i] == rates[0]);
            numBoundaryEdges += rates[i];
        }

        //  Assign or infer the inner rates:
        if (N != 4) {
            _innerRates[0] = (numRates > N) ? rates[N] : (numBoundaryEdges / N);
            _innerRates[1] = _innerRates[0];
        } else if (numRates > 4) {
            _innerRates[0] = rates[4];
            _innerRates[1] = rates[4 + (numRates > 5)];
        } else {
            _innerRates[0] = (rates[0] + rates[2]) / 2;
            _innerRates[1] = (rates[1] + rates[3]) / 2;
        }

        //  Test specified inner-rates to confirm still uniform:
        if (_isUniform && (numRates > N)) {
            _isUniform &= (_innerRates[0] == rates[0]);
            _isUniform &= (_innerRates[1] == rates[0]);
        }
    }

    //  Members related to the resulting pattern:
    _numBoundaryPoints = numBoundaryEdges;

    if (_isUniform) {
        int res = _outerRates[0];

        switch (_param.GetType()) {
        case Parameterization::QUAD:
            _numInteriorPoints = quad::countInteriorCoords(res);
            _numFacets = quad::countUniformFacets(res);
            break;
        case Parameterization::TRI:
            _numInteriorPoints = tri::countInteriorCoords(res);
            _numFacets = tri::countUniformFacets(res);
            break;
        case Parameterization::QPOLY:
            _numInteriorPoints = qpoly::countInteriorCoords(N, res);
            _numFacets = qpoly::countUniformFacets(N, res);
            break;
        }
    } else {
        int resOuterSum = numBoundaryEdges;

        int res0 = _innerRates[0];
        int res1 = _innerRates[1];

        switch (_param.GetType()) {
        case Parameterization::QUAD:
            _numInteriorPoints = quad::countInteriorCoords(res0, res1);
            _numFacets = quad::countNonUniformFacets(resOuterSum, res0, res1);
            break;
        case Parameterization::TRI:
            _numInteriorPoints = tri::countInteriorCoords(res0);
            _numFacets = tri::countNonUniformFacets(resOuterSum, res0);
            break;
        case Parameterization::QPOLY:
            _numInteriorPoints = qpoly::countInteriorCoords(N, res0);
            _numFacets = qpoly::countNonUniformFacets(N, resOuterSum, res0);
            break;
        }
        //  Given inner resolutions may lead to 0 interior points, but for a
        //  non-uniform tessellation we require at least 1:
        //
        //  WIP - this may be relaxed for some special cases in future to
        //  reduce the number of Coords/Facets for very low tessellations.
        if (_numInteriorPoints == 0) {
            _numInteriorPoints = 1;
        }
    }
    _numTotalPoints = _numBoundaryPoints + _numInteriorPoints;

    bool printNonUniform = false; // !_isUniform;
    if (printNonUniform) {
        printf("Tessellation::initialize(%d, numRates = %d):\n", N, numRates);
        printf("    is uniform          = %d\n", _isUniform);
        printf("        outer rates     =");
        for (int i = 0; i < N; ++i) printf(" %d", _outerRates[i]);
        printf("\n");
        printf("        inner rate(s)   = %d", _innerRates[0]);
        if (N == 4) printf(" %d\n", _innerRates[1]);
        printf("\n");
        printf("    num boundary points = %d\n", _numBoundaryPoints);
        printf("    num interior points = %d\n", _numInteriorPoints);
        printf("    num facets          = %d\n", _numFacets);
    }
}

//
//  Tessellation constructors and destructor:
//
Tessellation::Tessellation(Parameterization p, int uniformRate,
                           Options options) {

    initialize(p, 1, &uniformRate, options);
}

Tessellation::Tessellation(Parameterization p, int numRates, int const rates[],
                           Options options) {

    initialize(p, numRates, rates, options);
}

Tessellation::~Tessellation() {
}


//
//  Main methods to retrieve samples and facets:
//
int
Tessellation::GetBoundaryCoords(int edge, bool v0, bool v1, Coord coords[]) const {

    //  Remember - "edge coords" here excludes coords at the end vertices

    int res = _outerRates[edge];

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::getBoundaryEdgeCoords(edge, res, v0, v1, coords);
    case Parameterization::TRI:
        return tri::getBoundaryEdgeCoords(edge, res, v0, v1, coords);
    case Parameterization::QPOLY:
        return qpoly::getBoundaryEdgeCoords(GetFaceSize(), edge, res, v0, v1, coords);
    default:
        assert(0);
    }
    return -1;
}

int
Tessellation::GetBoundaryCoords(Coord coords[]) const {

    if (_numBoundaryPoints == GetFaceSize()) {
        return GetCornerCoords(coords);
    }

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::getBoundaryCoords(_outerRates, coords);
    case Parameterization::TRI:
        return tri::getBoundaryCoords(_outerRates, coords);
    case Parameterization::QPOLY:
        return qpoly::getBoundaryCoords(GetFaceSize(), _outerRates, coords);
    default:
        assert(0);
    }
    return -1;
}

int
Tessellation::GetInteriorCoords(Coord coords[]) const {

    if (_numInteriorPoints == 0) return 0;

    if (_numInteriorPoints == 1) {
        _param.GetCenterCoord(coords[0]);
        return 1;
    }

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::getInteriorCoords(_innerRates, coords);
    case Parameterization::TRI:
        return tri::getInteriorCoords(_innerRates[0], coords);
    case Parameterization::QPOLY:
        return qpoly::getInteriorCoords(GetFaceSize(), _innerRates[0], coords);
    default:
        assert(0);
    }
    return 0;
}

int
Tessellation::GetCoords(Coord coords[]) const {

    if (_numTotalPoints == GetFaceSize()) {
        return GetCornerCoords(coords);
    }

    int nCoords = GetBoundaryCoords(coords);
    nCoords += GetInteriorCoords(coords + nCoords);
    return nCoords;
}

int
Tessellation::GetFacets(Facet facets[]) const {

    if (_numFacets == 1) {
        return getSimpleFacet(GetFaceSize(), 0, facets);
    }
    int N = GetFaceSize();

    if (_isUniform) {
        int res = _outerRates[0];

        switch (_param.GetType()) {
        case Parameterization::QUAD:
            return quad::getUniformFacets(res, facets);
        case Parameterization::TRI:
            return tri::getUniformFacets(res, facets);
        case Parameterization::QPOLY:
            return qpoly::getUniformFacets(N, res, facets);
        default:
            assert(0);
        }
    } else {
        int const * outerRes = &_outerRates[0];
        int const * innerRes = &_innerRates[0];

int nFacets = 0;
        switch (_param.GetType()) {
        case Parameterization::QUAD:
            nFacets = quad::getNonUniformFacets(outerRes, innerRes,
                                             _numBoundaryPoints, facets);
            break;
        case Parameterization::TRI:
            nFacets = tri::getNonUniformFacets(outerRes, innerRes[0],
                                            _numBoundaryPoints, facets);
            break;
        case Parameterization::QPOLY:
            nFacets = qpoly::getNonUniformFacets(N, outerRes, innerRes[0],
                                              _numBoundaryPoints, facets);
            break;
        default:
            assert(0);
        }
assert(nFacets == _numFacets);
        return nFacets;
    }
    return -1;
}

void
Tessellation::TransformFacetIndices(Facet facets[], int commonOffset) {

    for (int i = 0; i < _numFacets; ++i) {
        Facet & f = facets[i];
        for (int j = 0; j < 4; ++j) {
            if (f[j] >= 0) {
                f[j] += commonOffset;
            }
        }
    }
}

void
Tessellation::TransformFacetIndices(Facet facets[], int boundaryOffset,
                                                   int interiorOffset) {

    for (int i = 0; i < _numFacets; ++i) {
        Facet & f = facets[i];
        for (int j = 0; j < 4; ++j) {
            if (f[j] >= 0) {
                if (f[j] > _numBoundaryPoints) {
                    f[j] += interiorOffset;
                } else {
                    f[j] += boundaryOffset;
                }
            }
        }
    }
}

void
Tessellation::TransformFacetIndices(Facet facets[], int const boundaryIndices[],
                                                   int interiorOffset) {

    for (int i = 0; i < _numFacets; ++i) {
        Facet & f = facets[i];
        for (int j = 0; j < 4; ++j) {
            if (f[j] >= 0) {
                if (f[j] > _numBoundaryPoints) {
                    f[j] += interiorOffset;
                } else {
                    f[j] = boundaryIndices[f[j]];
                }
            }
        }
    }
}

void
Tessellation::TransformFacetIndices(Facet facets[], int const boundaryIndices[],
                                                   int const interiorIndices[]) {

    for (int i = 0; i < _numFacets; ++i) {
        Facet & f = facets[i];
        for (int j = 0; j < 4; ++j) {
            if (f[j] >= 0) {
                if (f[j] > _numBoundaryPoints) {
                    f[j] = interiorIndices[f[j] - _numBoundaryPoints];
                } else {
                    f[j] = boundaryIndices[f[j]];
                }
            }
        }
    }
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
