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

#include "../bfr/tessellation.h"

#include <cstring>
#include <cstdio>

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
    setSimpleFacet(Facet facets[], int size, int startIndex = 0) {

        if (size == 3) {
            facets[0].Set(startIndex, startIndex+1, startIndex+2);
        } else {
            facets[0].Set(startIndex, startIndex+1, startIndex+2, startIndex+3);
        }
        return 1;
    }

    inline int
    setTriFanFacets(Facet facets[], int size, int startIndex = 0) {

        for (int i = 1; i <= size; ++i) {
            facets[i-1].Set(startIndex + (i - 1),
                            startIndex + ((i < size) ? i : 0),
                            startIndex + size);
        }
        return size;
    }

    inline int
    setTriFacet(Facet facets[], int t0, int t1, int t2) {

        facets[0].Set(t0, t1, t2);
        return 1;
    }

    inline int
    setQuadFacets(Facet facets[], int q0, int q1, int q2, int q3,
                  int triangulationSign = 0) {

        if (triangulationSign == 0) {
            // no triangulation
            facets[0].Set(q0, q1, q2, q3);
            return 1;
        } else if (triangulationSign > 0) {
            // triangulate along diagonal in direction of leading edge
            facets[0].Set(q0, q1, q2);
            facets[1].Set(q2, q3, q0);
            return 2;
        } else {
            // triangulate along diagonal opposing the leading edge
            facets[0].Set(q2, q3, q1);
            facets[1].Set(q0, q1, q3);
            return 2;
        }
    }

    //
    //  Useful struct for storing bounding indices and other topology of
    //  a strip of facets so points can be connected in various ways:
    //
    //  A strip of facets is defined between an outer and inner ring of
    //  points -- denoted as follows, where the "i" and "o" prefixes are
    //  used to designate points on the inner and outer rings:
    //
    //    oPrev  ---  iFirst  ... iFirst+/-i ...  iLast    --- oLast+1
    //      |                                                    |
    //    oFirst --- oFirst+1 ...  oFirst+j ... oFirst+N-1 --- oLast
    //
    //  Since these points form part of a ring, they will wrap around to
    //  the beginning of the ring for the last edge and so the sequence
    //  is not always sequential.  Transitions to the "first" and "last"
    //  of both the outer and inner rings are potentially discontinuous,
    //  which is why they are provided as separate members.
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
        FacetStrip() { std::memset(this, 0, sizeof(*this)); }

        int connectUniformQuads(  Facet facets[]) const;
        int connectUniformTris(   Facet facets[]) const;
        int connectNonUniformTris(Facet facets[]) const;

    public:
        //  Members defining how the strip should be used:
        unsigned int quadTopology    : 1;
        unsigned int quadTriangulate : 1;
        unsigned int innerReversed   : 1;

        unsigned int excludeFirst  : 1;
        unsigned int connectFirst  : 1;
        unsigned int connectLast   : 1;
        unsigned int includeLast   : 1;

        //  Members defining the dimensions of the strip -- the number
        //  of "inner edges" potentially excludes the two edges that
        //  connect the inner ring to the outer:
        int outerEdges;
        int innerEdges;

        //  Members containing indices for points noted above.  Since
        //  a strip may wrap around the concentric rings of points,
        //  pairs of points that may appear to have successive indices
        //  will not -- which is why these are assigned externally:
        int outerFirst, outerLast, outerPrev;
        int innerFirst, innerLast;
    };

    int
    FacetStrip::connectUniformQuads(Facet facets[]) const {

        assert(quadTopology);
        assert(innerEdges == (outerEdges - 2));
        //
        //  For connecting quads, the pattern is simplified as follows:
        //
        //      oPrev ---- iFirst  ...   iLast ---- oLast+1
        //        | 3      2 | 3         2 | 3       2 |
        //        | 0      1 | 0         1 | 0       1 |
        //      oFirst -- oFirst+1 ... oFirst+N-1 -- oLast
        //
        //  with the first and last quads not sharing any inner edges
        //  (between inner-first and inner-last) and potentially being
        //  split to include the triangle on the outer edge.
        //
        //  It is typical for the first quad to always be included and
        //  for the last to be excluded -- the last quad usually being
        //  included by the next strip in the ring (unless split).
        //
        int nFacets = 0;

        //  Split or assign the first quad (precedes inner edges):
        int out0 = outerFirst;
        int in0  = innerFirst;

        if (connectFirst) {
            nFacets += setTriFacet(facets + nFacets, out0, out0 + 1, in0);
        } else if (!excludeFirst) {
            nFacets += setQuadFacets(facets + nFacets,
                             out0, out0 + 1, in0, outerPrev, quadTriangulate);
        }

        //  Assign quads sharing the inner edges (last is a special case):
        int outI = outerFirst + 1;
        int inI  = innerFirst;

        if (innerEdges) {
            int triSign = quadTriangulate;
            int inDelta = innerReversed ? -1 : 1;

            for (int i = 1; i <= innerEdges; ++i, ++outI, inI += inDelta) {
                if (i > (innerEdges / 2)) triSign = - quadTriangulate;

                int outJ = outI + 1;
                int inJ  = (i < innerEdges) ? (inI + inDelta) : innerLast;

                nFacets += setQuadFacets(facets + nFacets,
                                 outI, outJ, inJ, inI, triSign);
            }
        }

        //  Split or assign the last quad (follows inner edges):
        int outN = outerLast;
        int inN  = innerLast;

        if (connectLast) {
            nFacets += setTriFacet(facets + nFacets, outI, outN, inN);
        } else if (includeLast) {
            nFacets += setQuadFacets(facets + nFacets,
                             outI, outN, outN+1, inN, -quadTriangulate);
        }
        return nFacets;
    }

    int
    FacetStrip::connectUniformTris(Facet facets[]) const {

        assert(!quadTopology);
        assert(!excludeFirst);
        assert(!includeLast);
        assert(!innerReversed);
        //
        //  Assign the set of tris for the "sawtooth" strip with N outer
        //  edges and N-3 inner edges of the inner ring:
        //
        //               1       3              2M-1
        //       oPrev --- iFirst -- i1  ...  ii --- iLast -- oLast+1
        //          / 2\1  0/  \    /  \       \1  0/ 2\    /  \.
        //         /0  1\2 /    \  /    \       \2 /0  1\  /    \.
        //    oFirst --- o1 ---- o2  ..  oi  ... oM --- oN-1 --- oLast
        //           0       2       4              2M
        //
        //  The first and last pair of tris may optionally be split by
        //  connecting the "first" or "last" points between the two rows
        //  (i.e. [oFirst, oFirst+1, iFirst]) which bisects the two
        //  triangles normally included.
        //
        //  Following the first pair (or single tri if split), a single
        //  leading triangle ([o1, o2, iFirst] above) is then assigned,
        //  followed by pairs of adjacent tris below each inner edge:
        //  the first of the pair based on the inner edge, the second on
        //  the outer edge.
        //
        int nFacets = 0;

        //  Split or assign the first pair of tris (precedes inner edges):
        int out0 = outerFirst;
        int in0  = innerFirst;

        if (connectFirst) {
            nFacets += setTriFacet(facets + nFacets, out0, out0+1, in0);
        } else {
            nFacets += setTriFacet(facets + nFacets, out0, out0+1, outerPrev);
            nFacets += setTriFacet(facets + nFacets, in0, outerPrev, out0+1);
        }

        //  Assign the next tri -- preceding the pairs for the inner edges:
        nFacets += setTriFacet(facets + nFacets, out0 + 1, out0 + 2, in0);

        //  Assign pair of tris below each inner edge (last is special):
        int outI = outerFirst + 2;
        int inI  = innerFirst;

        if (innerEdges) {
            for (int i = 1; i <= innerEdges; ++i, ++inI, ++outI) {
                int outJ = outI + 1;
                int inJ  = (i < innerEdges) ? (inI  + 1) : innerLast;

                nFacets += setTriFacet(facets + nFacets, inJ, inI, outI);
                nFacets += setTriFacet(facets + nFacets, outI, outJ, inJ);
            }
        }

        //  Split the last pair of tris (follows  inner edges):
        int outN = outerLast;
        int inN  = innerLast;

        if (connectLast) {
            nFacets += setTriFacet(facets + nFacets, outI, outN, inN);
        }
        return nFacets;
    }

    int
    FacetStrip::connectNonUniformTris(Facet facets[]) const {

        //
        //  General case:
        //
        //   oPrev -- iFirst  .  ...  i0+/-i  ...   .   iLast --*
        //        |   /       .                     .        \  |
        //        | /         |                     |         \ |
        //   oFirst -------- o0  ...   o0+i   ...  oN-1 ------ oLast
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
                nFacets += setTriFacet(facets + nFacets,
                                       cOuter0, cOuter1, cInner0);

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
                nFacets += setTriFacet(facets + nFacets,
                                       cInner1, cInner0, cOuter0);

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
class quad {
public:
    //  Public methods for counting coords and facets:
    static int CountUniformFacets(int edgeRes, bool triangulate);
    static int CountSegmentedFacets(int const uvRes[], bool triangulate);
    static int CountNonUniformFacets(int const outerRes[], int const uvRes[],
                                     bool triangulate);

    static int CountInteriorCoords(int edgeRes);
    static int CountInteriorCoords(int const uvRes[]);

    //  Public methods for identifying and assigning coords:
    static int GetCornerCoords(Coord coords[]);
    static int GetBoundaryEdgeCoords(int edge, int edgeRes,
                                     bool v0, bool v1, Coord coords[]);
    static int GetBoundaryCoords(int const edgeRates[], Coord coords[]);
    static int GetInteriorCoords(int const uvRes[2], Coord coords[]);

    //  Public methods for identifying and assigning facets:
    static int GetUniformFacets(int uniformRes,
                                Facet facets[], bool triangulate);
    static int GetSegmentedFacets(int const uvRes[],
                                  Facet facets[], bool triangulate);
    static int GetNonUniformFacets(int const outerRes[], int const innerRes[],
                                   int nBoundaryEdges,
                                   Facet facets[], bool triangulate);
private:
    //  Private methods used by those above:
    static int countUniformCoords(int edgeRes);

    static int getCenterCoord(Coord coords[]);
    static int getInteriorRingCoords(int   uRes,   int   vRes,
                                     float uStart, float vStart,
                                     float uDelta, float vDelta,
                                     Coord coords[]);

    static int getInteriorRingFacets(int uRes, int vRes, int indexOfFirstCoord,
                                     Facet facets[], bool triangulate);
    static int getBoundaryRingFacets(int const outerRes[],
                                     int uRes, int vRes, int nBoundaryEdges,
                                     Facet facets[], bool triangulate);
    static int getSingleStripFacets(int uRes, int vRes, int indexOfFirstCoord,
                                    Facet facets[], bool triangulate);
};

class tri {
public:
    //  Public methods for counting coords and facets:
    static int CountUniformFacets(int edgeRes);
    static int CountNonUniformFacets(int const outerRes[], int innerRes);

    static int CountInteriorCoords(int edgeRes);

    //  Public methods for identifying and assigning coords:
    static int GetCornerCoords(Coord coords[]);
    static int GetBoundaryEdgeCoords(int edge, int edgeRes,
                                     bool v0, bool v1, Coord coords[]);
    static int GetBoundaryCoords(int const edgeRates[], Coord coords[]);
    static int GetInteriorCoords(int edgeRes, Coord coords[]);

    //  Public methods for identifying and assigning facets:
    static int GetUniformFacets(int uniformRes, Facet facets[]);
    static int GetNonUniformFacets(int const outerRes[], int innerRes,
                                   int nBoundaryEdges, Facet facets[]);
private:
    //  Private methods used by those above:
    static int countUniformCoords(int edgeRes);

    static int getCenterCoord(Coord coords[]);
    static int getInteriorRingCoords(int   edgeRes,
                                     float uStart, float vStart,
                                     float tDelta,
                                     Coord coords[]);

    static int getInteriorRingFacets(int edgeRes, int indexOfFirstCoord,
                                     Facet facets[]);
    static int getBoundaryRingFacets(int const outerRes[], int innerRes,
                                     int nBoundaryEdges,
                                     Facet facets[]);
};

class qpoly {
public:
    //  Public methods for counting coords and facets:
    static int CountUniformFacets(int N, int edgeRes, bool triangulate);
    static int CountNonUniformFacets(int N, int const outerRes[], int innerRes,
                                     bool triangulate);

    static int CountInteriorCoords(int N, int edgeRes);

    //  Public methods for identifying and assigning coords (note these
    //  require a full Parameterization while others only need size N)
    static int GetCornerCoords(Parameterization P, Coord coords[]);
    static int GetBoundaryEdgeCoords(Parameterization P, int edge,
                                     int edgeRes, bool incFirst, bool incLast,
                                     Coord coords[]);
    static int GetBoundaryCoords(Parameterization P, int const edgeRates[],
                                 Coord coords[]);
    static int GetInteriorCoords(Parameterization P, int edgeRes,
                                 Coord coords[]);

    //  Public methods for identifying and assigning facets:
    static int GetUniformFacets(int N, int uniformRes,
                                Facet facets[], bool triangulate);
    static int GetNonUniformFacets(int N, int const outerRes[], int innerRes,
                                   int nBoundaryEdges,
                                   Facet facets[], bool triangulate);
private:
    //  Private methods used by those above:
    static int countUniformCoords(int N, int edgeRes);

    static int getCenterCoord(Coord coords[]);
    static int getCenterRingCoords(Parameterization P, float tStart,
                                   Coord coords[]);
    static int getRingEdgeCoords(Parameterization P, int edge, int edgeRes,
                                 bool incFirst, bool incLast,
                                 float tStart, float tDelta,
                                 Coord coords[]);
    static int getInteriorRingCoords(Parameterization P, int edgeRes,
                                     float tStart, float tDelta,
                                     Coord coords[]);

    static int getCenterFacets(int N, int indexOfFirstCoord, Facet facets[]);
    static int getInteriorRingFacets(int N, int edgeRes, int indexOfFirstCoord,
                                     Facet facets[], bool triangulate);
    static int getBoundaryRingFacets(int N, int const outerRes[], int innerRes,
                                     int nBoundaryEdges,
                                     Facet facets[], bool triangulate);
};

//
//  Implementations for quad functions:
//
inline int
quad::CountUniformFacets(int edgeRes, bool triangulate) {
    return (edgeRes * edgeRes) << triangulate;
}

inline int
quad::CountSegmentedFacets(int const uvRes[], bool triangulate) {
    //  WIP - may extend this later to account for outer rates...
    assert((uvRes[0] == 1) || (uvRes[1] == 1));
    return (uvRes[0] * uvRes[1]) << triangulate;
}

inline int
quad::CountNonUniformFacets(int const outerRes[], int const innerRes[],
                            bool triangulate) {

    int uRes = innerRes[0];
    int vRes = innerRes[1];
    assert((uRes > 1) && (vRes > 1));

    //  Count interior facets based on edges of inner ring:
    int innerUEdges = uRes - 2;
    int innerVEdges = vRes - 2;

    int nInterior = (innerUEdges * innerVEdges) << triangulate;

    //
    //  Accumulate boundary facets for each edge based on uniformity...
    //
    //  A uniform edge contributes a quad for each inner edge, plus one
    //  facet for the leading corner (quad if uniform, tri if not) and a
    //  tri for the trailing corner if it is not uniform.
    //
    //  A non-uniform edge contributes a tri for each of the inner edges
    //  and one for each of the outer edges.
    //
    bool uniformEdges[4];
    uniformEdges[0] = (outerRes[0] == uRes) && !triangulate;
    uniformEdges[1] = (outerRes[1] == vRes) && !triangulate;
    uniformEdges[2] = (outerRes[2] == uRes) && !triangulate;
    uniformEdges[3] = (outerRes[3] == vRes) && !triangulate;

    bool uniformCorners[4];
    uniformCorners[0] = (uniformEdges[0] && uniformEdges[3]);
    uniformCorners[1] = (uniformEdges[1] && uniformEdges[0]);
    uniformCorners[2] = (uniformEdges[2] && uniformEdges[1]);
    uniformCorners[3] = (uniformEdges[3] && uniformEdges[2]);

    int nBoundary = 0;
    nBoundary += uniformEdges[0] ? (innerUEdges + 1 + !uniformCorners[1]) :
                                   (innerUEdges + outerRes[0]);
    nBoundary += uniformEdges[1] ? (innerVEdges + 1 + !uniformCorners[2]) :
                                   (innerVEdges + outerRes[1]);
    nBoundary += uniformEdges[2] ? (innerUEdges + 1 + !uniformCorners[3]) :
                                   (innerUEdges + outerRes[2]);
    nBoundary += uniformEdges[3] ? (innerVEdges + 1 + !uniformCorners[0]) :
                                   (innerVEdges + outerRes[3]);
    return nInterior + nBoundary;
}

inline int
quad::countUniformCoords(int edgeRes) {
    return (edgeRes + 1) * (edgeRes + 1);
}

inline int
quad::CountInteriorCoords(int uniformRes) {
    return (uniformRes - 1) * (uniformRes - 1);
}

inline int
quad::CountInteriorCoords(int const uvRes[]) {
    return (uvRes[0] - 1) * (uvRes[1] - 1);
}

inline int
quad::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(0.5f, 0.5f);
    return 1;
}

int
quad::GetCornerCoords(Coord coords[]) {

    coords[0] = Coord(0.0f, 0.0f);
    coords[1] = Coord(0.0f, 1.0f);
    coords[2] = Coord(1.0f, 1.0f);
    coords[3] = Coord(1.0f, 0.0f);
    return 4;
}

int
quad::GetBoundaryEdgeCoords(int edge, int edgeRes, bool v0, bool v1,
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
quad::GetBoundaryCoords(int const edgeRates[], Coord coords[]) {

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
quad::GetInteriorCoords(int const uvRes[2], Coord coords[]) {

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
quad::getSingleStripFacets(int uRes, int vRes, int coord0,
                           Facet facets[], bool triangulate) {

    assert((uRes == 1) || (vRes == 1));

    FacetStrip qStrip;
    qStrip.quadTopology    = true;
    qStrip.quadTriangulate = triangulate;
    qStrip.connectFirst    = false;
    qStrip.connectLast     = false;
    qStrip.innerReversed   = true;
    qStrip.includeLast     = true;

    if (uRes > 1) {
        qStrip.outerEdges = uRes;
        qStrip.innerEdges = uRes - 2;

        //  Assign these successively around the strip:
        qStrip.outerFirst = coord0;
        qStrip.outerLast  = qStrip.outerFirst + uRes;
        qStrip.innerLast  = qStrip.outerLast  + 2;
        qStrip.innerFirst = qStrip.outerLast  + uRes;
        qStrip.outerPrev  = qStrip.innerFirst + 1;

        return qStrip.connectUniformQuads(facets);
    } else {
        qStrip.outerEdges = vRes;
        qStrip.innerEdges = vRes - 2;

        qStrip.outerPrev  = coord0;
        qStrip.outerFirst = coord0 + 1;
        qStrip.outerLast  = qStrip.outerFirst + vRes;
        qStrip.innerLast  = qStrip.outerLast  + 2;
        qStrip.innerFirst = qStrip.outerLast  + vRes;

        return qStrip.connectUniformQuads(facets);
    }
}

int
quad::getInteriorRingFacets(int uRes, int vRes, int coord0,
                            Facet facets[], bool triangulate) {

    assert((uRes >= 0) && (vRes >= 0));

    //
    //  Deal with some simple and special cases first:
    //
    int totalInnerFacets = uRes * vRes;
    if (totalInnerFacets == 0) return 0;

    if (totalInnerFacets == 1) {
        return setQuadFacets(facets, coord0, coord0+1, coord0+2, coord0+3,
                                triangulate);
    }

    //  The single interior strip is enclosed by a single ring:
    if ((uRes == 1) || (vRes == 1)) {
        return getSingleStripFacets(uRes, vRes, coord0, facets, triangulate);
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

    FacetStrip qStrip;
    qStrip.quadTopology    = true;
    qStrip.quadTriangulate = triangulate;
    qStrip.connectFirst    = false;
    qStrip.connectLast     = false;

    qStrip.outerEdges    = uRes;
    qStrip.outerFirst    = outerRingStart;
    qStrip.outerPrev     = innerRingStart - 1;
    qStrip.outerLast     = outerRingStart + uRes;
    qStrip.innerEdges    = uResInner;
    qStrip.innerReversed = false;
    qStrip.innerFirst    = innerRingStart;
    qStrip.innerLast     = innerRingStart + uResInner;
    nFacets += qStrip.connectUniformQuads(facets + nFacets);

    qStrip.outerEdges    = vRes;
    qStrip.outerFirst   += uRes;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast     = qStrip.outerFirst + vRes;
    qStrip.innerEdges    = vResInner;
    qStrip.innerReversed = false;
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += vResInner;
    nFacets += qStrip.connectUniformQuads(facets + nFacets);

    qStrip.outerEdges    = uRes;
    qStrip.outerFirst   += vRes;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast     = qStrip.outerFirst + uRes;
    qStrip.innerEdges    = uResInner;
    qStrip.innerReversed = (vResInner == 0);
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += uResInner * (qStrip.innerReversed ? -1 : 1);
    nFacets += qStrip.connectUniformQuads(facets + nFacets);

    qStrip.outerEdges    = vRes;
    qStrip.outerFirst   += uRes;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast     = outerRingStart;
    qStrip.innerEdges    = vResInner;
    qStrip.innerReversed = (uResInner == 0);
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast     = innerRingStart;
    nFacets += qStrip.connectUniformQuads(facets + nFacets);

    return nFacets;
}

int
quad::getBoundaryRingFacets(int const outerRes[], int uRes, int vRes,
                            int nBoundaryEdges,
                            Facet facets[], bool triangulate) {

    //  Identify edges and corners that should preserve uniform behavior:
    bool uniformEdges[4];
    uniformEdges[0] = (outerRes[0] == uRes);
    uniformEdges[1] = (outerRes[1] == vRes);
    uniformEdges[2] = (outerRes[2] == uRes);
    uniformEdges[3] = (outerRes[3] == vRes);

    bool uniformCorners[4];
    uniformCorners[0] = (uniformEdges[0] && uniformEdges[3]);
    uniformCorners[1] = (uniformEdges[1] && uniformEdges[0]);
    uniformCorners[2] = (uniformEdges[2] && uniformEdges[1]);
    uniformCorners[3] = (uniformEdges[3] && uniformEdges[2]);

    //  Initialize inner edge counts and the FacetStrip for local use:
    assert((uRes > 1) && (vRes > 1));
    int innerResU = uRes - 2;
    int innerResV = vRes - 2;

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    FacetStrip qStrip;
    qStrip.quadTopology    = true;
    qStrip.quadTriangulate = triangulate;

    //  Assign strip indices for the inner and outer rings:
    qStrip.outerEdges    = outerRes[0];
    qStrip.outerFirst    = outerRingStart;
    qStrip.outerPrev     = innerRingStart - 1;
    qStrip.outerLast     = outerRingStart + outerRes[0];
    qStrip.innerEdges    = innerResU;
    qStrip.innerReversed = false;
    qStrip.innerFirst    = innerRingStart;
    qStrip.innerLast     = innerRingStart + innerResU;
    if (uniformEdges[0]) {
        qStrip.connectFirst  = !uniformCorners[0];
        qStrip.connectLast   = !uniformCorners[1];
        nFacets += qStrip.connectUniformQuads(facets + nFacets);
    } else {
        nFacets += qStrip.connectNonUniformTris(facets + nFacets);
    }

    qStrip.outerEdges    = outerRes[1];
    qStrip.outerFirst    = qStrip.outerLast;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast    += outerRes[1];
    qStrip.innerEdges    = innerResV;
    qStrip.innerReversed = false;
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += innerResV;
    if (uniformEdges[1]) {
        qStrip.connectFirst  = !uniformCorners[1];
        qStrip.connectLast   = !uniformCorners[2];
        nFacets += qStrip.connectUniformQuads(facets + nFacets);
    } else {
        nFacets += qStrip.connectNonUniformTris(facets + nFacets);
    }

    qStrip.outerEdges    = outerRes[2];
    qStrip.outerFirst    = qStrip.outerLast;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast    += outerRes[2];
    qStrip.innerEdges    = innerResU;
    qStrip.innerReversed = (innerResV == 0);
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast    += innerResU * (qStrip.innerReversed ? -1 : 1);
    if (uniformEdges[2]) {
        qStrip.connectFirst  = !uniformCorners[2];
        qStrip.connectLast   = !uniformCorners[3];
        nFacets += qStrip.connectUniformQuads(facets + nFacets);
    } else {
        nFacets += qStrip.connectNonUniformTris(facets + nFacets);
    }

    qStrip.outerEdges    = outerRes[3];
    qStrip.outerFirst    = qStrip.outerLast;
    qStrip.outerPrev     = qStrip.outerFirst - 1;
    qStrip.outerLast     = 0;
    qStrip.innerEdges    = innerResV;
    qStrip.innerReversed = (innerResU == 0);
    qStrip.innerFirst    = qStrip.innerLast;
    qStrip.innerLast     = innerRingStart;
    if (uniformEdges[3]) {
        qStrip.connectFirst  = !uniformCorners[3];
        qStrip.connectLast   = !uniformCorners[0];
        nFacets += qStrip.connectUniformQuads(facets + nFacets);
    } else {
        nFacets += qStrip.connectNonUniformTris(facets + nFacets);
    }
    return nFacets;
}

int
quad::GetSegmentedFacets(int const innerRes[],
                         Facet facets[], bool triangulate) {

    //  WIP - may extend this later to account for differing outer rates
    //        resulting in a non-uniform strip of faces between the two
    //        opposing edges
    int uRes = innerRes[0];
    int vRes = innerRes[1];
    assert((uRes == 1) || (vRes == 1));

    return getSingleStripFacets(uRes, vRes, 0, facets, triangulate);
}

int
quad::GetNonUniformFacets(int const outerRes[], int const innerRes[],
                          int nBoundaryEdges,
                          Facet facets[], bool triangulate){

    int uRes = innerRes[0];
    int vRes = innerRes[1];
    assert((uRes > 1) && (vRes > 1));

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(outerRes, uRes, vRes, nBoundaryEdges,
                                        facets, triangulate);

    //  Second, generate the remaining rings of interior facets:
    int nRings = (std::min(uRes,vRes) + 1) / 2;
    int coord0 = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        uRes = std::max(uRes - 2, 0);
        vRes = std::max(vRes - 2, 0);

        nFacets += getInteriorRingFacets(uRes, vRes, coord0,
                                         facets + nFacets, triangulate);
        coord0  += 2 * (uRes + vRes);
    }
    return nFacets;
}

int
quad::GetUniformFacets(int res, Facet facets[], bool triangulate) {

    //  The trivial case should have been handled by the caller:
    assert(res > 1);

    int nRings = (res + 1) / 2;

    int nFacets = 0;
    int coord0 = 0;
    for (int ring = 0; ring < nRings; ++ring, res -= 2) {
        nFacets += getInteriorRingFacets(res, res, coord0,
                                         facets + nFacets, triangulate);
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
tri::CountUniformFacets(int edgeRes) {
    return edgeRes * edgeRes;
}

inline int
tri::CountNonUniformFacets(int const outerRes[], int innerRes) {

    assert(innerRes > 2);

    //  Count interior facets based on edges of inner ring:
    int nInnerEdges = innerRes - 3;

    int nInterior = nInnerEdges ? CountUniformFacets(nInnerEdges) : 0;

    //
    //  Note the number of boundary facets is not affected by the uniform
    //  behavior at corners when rates match -- in contrast to quads.  In
    //  both cases, two tris are generated from four points at the corner,
    //  just with a different edge bisecting that "quad".
    //
    int nBoundary = (nInnerEdges + outerRes[0]) +
                    (nInnerEdges + outerRes[1]) +
                    (nInnerEdges + outerRes[2]);

    return nInterior + nBoundary;
}

inline int
tri::countUniformCoords(int edgeRes) {
    return edgeRes * (edgeRes + 1) / 2;
}

inline int
tri::CountInteriorCoords(int edgeRes) {
    return countUniformCoords(edgeRes - 2);
}

inline int
tri::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(1.0f/3.0f, 1.0f/3.0f);
    return 1;
}

int
tri::GetCornerCoords(Coord coords[]) {

    coords[0] = Coord(0.0f, 0.0f);
    coords[1] = Coord(0.0f, 1.0f);
    coords[2] = Coord(1.0f, 0.0f);
    return 3;
}

int
tri::GetBoundaryEdgeCoords(int edge, int edgeRes, bool v0, bool v1,
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
tri::GetBoundaryCoords(int const edgeRates[], Coord coords[]) {

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
tri::GetInteriorCoords(int edgeRes, Coord coords[]) {

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
        return setTriFacet(facets, coord0, coord0+1, coord0+2);
    } else if (edgeRes == 2) {
        setTriFacet(facets + 0, coord0+0, coord0+1, coord0+5);
        setTriFacet(facets + 1, coord0+2, coord0+3, coord0+1);
        setTriFacet(facets + 2, coord0+4, coord0+5, coord0+3);
        setTriFacet(facets + 3, coord0+1, coord0+3, coord0+5);
        return 4;
    }

    //
    //  Generate facets for the 3 tri-strips for each edge:
    //
    int nFacets = 0;

    int outerEdges = edgeRes;
    int innerEdges = edgeRes - 3;

    int outerRingStart = coord0;
    int innerRingStart = coord0 + 3 * outerEdges;

    FacetStrip tStrip;
    tStrip.quadTopology  = false;
    tStrip.innerReversed = false;
    tStrip.innerEdges    = innerEdges;
    tStrip.outerEdges    = outerEdges;

    tStrip.outerFirst = outerRingStart;
    tStrip.outerLast  = outerRingStart + outerEdges;
    tStrip.outerPrev  = innerRingStart - 1;
    tStrip.innerFirst = innerRingStart;
    tStrip.innerLast  = innerRingStart + innerEdges;
    nFacets += tStrip.connectUniformTris(facets + nFacets);

    tStrip.outerFirst += outerEdges;
    tStrip.outerLast  += outerEdges;
    tStrip.outerPrev   = tStrip.outerFirst - 1;
    tStrip.innerFirst += innerEdges;
    tStrip.innerLast  += innerEdges;
    nFacets += tStrip.connectUniformTris(facets + nFacets);

    tStrip.outerFirst += outerEdges;
    tStrip.outerLast   = outerRingStart;
    tStrip.outerPrev   = tStrip.outerFirst - 1;
    tStrip.innerFirst += innerEdges;
    tStrip.innerLast   = innerRingStart;
    nFacets += tStrip.connectUniformTris(facets + nFacets);

    return nFacets;
}

int
tri::getBoundaryRingFacets(int const outerRes[], int innerRes,
                           int nBoundaryEdges, Facet facets[]) {

    //  Identify edges and corners that should preserve uniform behavior:
    bool uniformEdges[3];
    uniformEdges[0] = (outerRes[0] == innerRes);
    uniformEdges[1] = (outerRes[1] == innerRes);
    uniformEdges[2] = (outerRes[2] == innerRes);

    bool uniformCorners[3];
    uniformCorners[0] = (uniformEdges[0] && uniformEdges[2]);
    uniformCorners[1] = (uniformEdges[1] && uniformEdges[0]);
    uniformCorners[2] = (uniformEdges[2] && uniformEdges[1]);

    //  Initialize inner edge count and the FacetStrip for local use:
    assert(innerRes > 2);
    int innerEdges = innerRes - 3;

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    FacetStrip tStrip;
    tStrip.quadTopology  = false;
    tStrip.innerReversed = false;
    tStrip.innerEdges    = innerEdges;

    //  Assign the three strips of Facets:
    tStrip.outerEdges   = outerRes[0];
    tStrip.outerFirst   = outerRingStart;
    tStrip.outerLast    = outerRingStart + outerRes[0];
    tStrip.outerPrev    = innerRingStart - 1;
    tStrip.innerFirst   = innerRingStart;
    tStrip.innerLast    = innerRingStart + innerEdges;
    if (uniformEdges[0]) {
        tStrip.connectFirst = !uniformCorners[0];
        tStrip.connectLast  = !uniformCorners[1];
        nFacets += tStrip.connectUniformTris(facets + nFacets);
    } else {
        nFacets += tStrip.connectNonUniformTris(facets + nFacets);
    }

    tStrip.outerEdges   = outerRes[1];
    tStrip.outerFirst   = tStrip.outerLast;
    tStrip.outerLast   += outerRes[1];
    tStrip.outerPrev    = tStrip.outerFirst - 1;
    tStrip.innerFirst   = tStrip.innerLast;
    tStrip.innerLast   += innerEdges;
    if (uniformEdges[1]) {
        tStrip.connectFirst = !uniformCorners[1];
        tStrip.connectLast  = !uniformCorners[2];
        nFacets += tStrip.connectUniformTris(facets + nFacets);
    } else {
        nFacets += tStrip.connectNonUniformTris(facets + nFacets);
    }

    tStrip.outerEdges   = outerRes[2];
    tStrip.outerFirst   = tStrip.outerLast;
    tStrip.outerLast    = 0;
    tStrip.outerPrev    = tStrip.outerFirst - 1;
    tStrip.innerFirst   = tStrip.innerLast;
    tStrip.innerLast    = innerRingStart;
    if (uniformEdges[2]) {
        tStrip.connectFirst = !uniformCorners[2];
        tStrip.connectLast  = !uniformCorners[0];
        nFacets += tStrip.connectUniformTris(facets + nFacets);
    } else {
        nFacets += tStrip.connectNonUniformTris(facets + nFacets);
    }
    return nFacets;
}

int
tri::GetUniformFacets(int edgeRes, Facet facets[]) {

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
tri::GetNonUniformFacets(int const outerRes[], int innerRes,
                         int nBoundaryEdges, Facet facets[]) {

    assert(innerRes > 2);

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(outerRes, innerRes,
                                        nBoundaryEdges, facets);

    //  Second, generate the remaining rings of interior facets:
    int nRings = 1 + (innerRes / 3);
    int coord0  = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        innerRes -= 3;

        nFacets += getInteriorRingFacets(innerRes,
                                         coord0, facets + nFacets);
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
qpoly::CountUniformFacets(int N, int edgeRes, bool triangulate) {

    bool resIsOdd = (edgeRes & 1);

    int H = edgeRes / 2;

    int nQuads  = (H + resIsOdd) * H * N;
    int nCenter = resIsOdd ? ((N == 3) ? 1 : N) : 0;

    return (nQuads << triangulate) + nCenter;
}

inline int
qpoly::CountNonUniformFacets(int N, int const outerRes[], int innerRes,
                             bool triangulate) {

    assert(innerRes > 1);

    //  Count interior facets based on edges of inner ring:
    int nInnerEdges = innerRes - 2;

    int nInterior = 0;
    if (nInnerEdges) {
        nInterior = CountUniformFacets(N, nInnerEdges, triangulate);
    }

    //
    //  Accumulate boundary facets for uniform vs non-uniform edge.  Uniform
    //  has a quad for each inner edge, plus one facet for leading corner
    //  and a tri for the trailing corner if not uniform.  Non-uniform has
    //  a tri for each inner edge and each outer edge:
    //
    int nBoundary = 0;
    for (int i = 0; i < N; ++i) {
        if ((outerRes[i] == innerRes) && !triangulate) {
            nBoundary += nInnerEdges + 1 + (innerRes != outerRes[(i+1) % N]);
        } else {
            nBoundary += nInnerEdges + outerRes[i];
        }
    }
    return nInterior + nBoundary;
}

inline int
qpoly::countUniformCoords(int N, int edgeRes) {

    int H = edgeRes / 2;
    return (edgeRes & 1) ? (H+1)* (H+1) * N + ((N == 3) ? 0 : 1)
                         :   H  * (H+1) * N + 1;
}

inline int
qpoly::CountInteriorCoords(int N, int edgeRes) {

    assert(edgeRes > 1);
    return countUniformCoords(N, edgeRes - 2);
}

inline int
qpoly::getCenterCoord(Coord coords[]) {

    coords[0] = Coord(0.5f, 0.5f);
    return 1;
}

int
qpoly::GetCornerCoords(Parameterization P, Coord coords[]) {

    int N = P.GetFaceSize();
    for (int i = 0; i < N; ++i) {
        P.GetCornerCoord(i, &coords[i][0], &coords[i][1]);
    }
    return N;
}

inline int
qpoly::getRingEdgeCoords(Parameterization P, int edge, int edgeRes,
                         bool incFirst, bool incLast,
                         float tOrigin, float dt, Coord coords[]) {

    //
    //  Determine number of coords in each half, excluding the ends.  The
    //  second half will get the extra when odd so that the sequence starts
    //  exactly on the boundary of the second sub-face (avoiding floating
    //  point error when accumulating to the boundary of the first):
    //
    int n0 = (edgeRes - 1) / 2;
    int n1 = (edgeRes - 1) - n0;

    int nCoords = 0;
    if (incFirst || n0) {
        float u0, v0;
        P.GetCornerCoord(edge, &u0, &v0);

        //  u ranges from [tOrigin < 0.5] while v is constant
        if (incFirst) {
            coords[nCoords++] = Coord(u0 + tOrigin, v0 + tOrigin);
        }
        if (n0) {
            float u = u0 + tOrigin + dt;
            float v = v0 + tOrigin;
            nCoords += getVIsoLineCoords(n0, u, v, dt, coords + nCoords);
        }
    }
    if (n1 || incLast) {
        float u1, v1;
        P.GetCornerCoord((edge + 1) % P.GetFaceSize(), &u1, &v1);

        //  u is constant while v ranges from [0.5 > tOrigin] (even)
        if (n1) {
            float u = u1 + tOrigin;
            float v = v1 + ((edgeRes & 1) ? (0.5f - 0.5f * dt) : 0.5f);
            nCoords += getUIsoLineCoords(n1, u, v, -dt, coords + nCoords);
        }
        if (incLast) {
            coords[nCoords++] = Coord(u1 + tOrigin, v1 + tOrigin);
        }
    }
    return nCoords;
}

int
qpoly::GetBoundaryEdgeCoords(Parameterization P, int edge,
                            int edgeRes, bool inc0, bool inc1,
                            Coord coords[]) {

    return getRingEdgeCoords(P, edge, edgeRes, inc0, inc1,
                             0.0f, 1.0f / (float)edgeRes,
                             coords);
}

int
qpoly::GetBoundaryCoords(Parameterization P, int const edgeRates[],
                         Coord coords[]) {


    int N = P.GetFaceSize();

    int nCoords = 0;
    for (int i = 0; i < N; ++i) {
        nCoords += getRingEdgeCoords(P, i, edgeRates[i], true, false,
                                     0.0f, 1.0f / (float)edgeRates[i],
                                     coords + nCoords);
    }
    return nCoords;
}

int
qpoly::getInteriorRingCoords(Parameterization P, int edgeRes,
                             float tOrigin, float dt,
                             Coord coords[]) {
    assert(edgeRes > 1);

    int N = P.GetFaceSize();

    int nCoords = 0;
    for (int i = 0; i < N; ++i) {
        nCoords += getRingEdgeCoords(P, i, edgeRes, true, false,
                                     tOrigin, dt,
                                     coords + nCoords);
    }
    return nCoords;
}

int
qpoly::getCenterRingCoords(Parameterization P, float tOrigin, Coord coords[]) {

    int N = P.GetFaceSize();

    //  Just need the single corner point for each edge here:
    for (int i = 0; i < N; ++i) {
        float uCorner, vCorner;
        P.GetCornerCoord(i, &uCorner, &vCorner);
        coords[i] = Coord(uCorner + tOrigin, vCorner + tOrigin);
    }
    return (N == 3) ? N : (N + getCenterCoord(coords + N));
}

inline int
qpoly::GetInteriorCoords(Parameterization P, int edgeRes, Coord coords[]) {

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
            nCoords += getCenterRingCoords(P, t, &coords[nCoords]);
        } else {
            nCoords += getInteriorRingCoords(P, ringRes, t, dt,
                                             &coords[nCoords]);
        }
    }
    return nCoords;
}

int
qpoly::getCenterFacets(int N, int coord0, Facet facets[]) {

    return (N == 3) ? setSimpleFacet(facets, 3, coord0)
                    : setTriFanFacets(facets, N, coord0);
}

int
qpoly::getInteriorRingFacets(int N, int edgeRes, int coord0,
                             Facet facets[], bool triangulate) {

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
    qStrip.quadTopology    = true;
    qStrip.quadTriangulate = triangulate;
    qStrip.outerEdges      = outerRes;
    qStrip.innerEdges      = innerRes;
    qStrip.innerReversed   = false;
    qStrip.connectFirst    = false;
    qStrip.connectLast     = false;

    for (int edge = 0; edge < N; ++edge) {
        qStrip.outerFirst = outerRing + edge * outerRes;
        qStrip.innerFirst = innerRing + edge * innerRes;

        qStrip.outerPrev = (edge ? qStrip.outerFirst : innerRing) - 1;

        if (edge < N-1) {
            qStrip.outerLast = qStrip.outerFirst + outerRes;
            qStrip.innerLast = qStrip.innerFirst + innerRes;
        } else {
            qStrip.outerLast = outerRing;
            qStrip.innerLast = innerRing;
        }

        nFacets += qStrip.connectUniformQuads(facets + nFacets);
    }
    return nFacets;
}

int
qpoly::getBoundaryRingFacets(int N, int const outerRes[], int innerRes,
                             int nBoundaryEdges,
                             Facet facets[], bool triangulate) {

    int innerEdges = std::max(innerRes - 2, 0);

    int nFacets = 0;

    int outerRingStart = 0;
    int innerRingStart = nBoundaryEdges;

    //  Initialize properties of the strip that are fixed:
    FacetStrip qStrip;
    qStrip.quadTopology    = true;
    qStrip.quadTriangulate = triangulate;
    qStrip.innerReversed   = false;
    qStrip.innerEdges      = innerEdges;

    for (int edge = 0; edge < N; ++edge) {
        qStrip.outerEdges = outerRes[edge];

        //  Initialize the indices starting this strip:
        if (edge) {
            qStrip.outerFirst = qStrip.outerLast;
            qStrip.outerPrev  = qStrip.outerFirst - 1;
            qStrip.innerFirst = qStrip.innerLast;
        } else {
            qStrip.outerFirst = outerRingStart;
            qStrip.outerPrev  = innerRingStart - 1;
            qStrip.innerFirst = innerRingStart;
        }

        //  Initialize the indices ending this strip:
        if (edge < N-1) {
            qStrip.outerLast = qStrip.outerFirst + qStrip.outerEdges;
            qStrip.innerLast = qStrip.innerFirst + qStrip.innerEdges;
        } else {
            qStrip.outerLast = outerRingStart;
            qStrip.innerLast = innerRingStart;
        }

        //  Test rates at, before and after this edge for uniform behavior:
        if ((outerRes[edge] == innerRes) && (innerRes > 1)) {
            qStrip.connectFirst = (outerRes[(edge-1+N) % N] != innerRes);
            qStrip.connectLast  = (outerRes[(edge + 1) % N] != innerRes);

            nFacets += qStrip.connectUniformQuads(facets+nFacets);
        } else {
            nFacets += qStrip.connectNonUniformTris(facets + nFacets);
        }
    }
    return nFacets;
}
    
int
qpoly::GetUniformFacets(int N, int edgeRes,
                        Facet facets[], bool triangulate) {

    //  The trivial (single facet) case should be handled externally:
    if (edgeRes == 1) {
        return getCenterFacets(N, 0, facets);
    }

    int nRings = (edgeRes + 1) / 2;

    int nFacets = 0;
    int coord0  = 0;
    for (int ring = 0; ring < nRings; ++ring, edgeRes -= 2) {
        nFacets += getInteriorRingFacets(N, edgeRes, coord0,
                                         facets + nFacets, triangulate);
        coord0  += N * edgeRes;
    }
    return nFacets;
}

int
qpoly::GetNonUniformFacets(int N, int const outerRes[], int innerRes,
                           int nBoundaryEdges,
                           Facet facets[], bool triangulate){

    //  First, generate the ring of boundary facets separately:
    int nFacets = getBoundaryRingFacets(N, outerRes, innerRes, nBoundaryEdges,
                                        facets, triangulate);

    //  Second, generate the remaining rings of interior facets:
    int nRings = (innerRes + 1) / 2;
    int coord0  = nBoundaryEdges;

    for (int ring = 1; ring < nRings; ++ring) {
        innerRes = std::max(innerRes - 2, 0);

        nFacets += getInteriorRingFacets(N, innerRes, coord0,
                                         facets + nFacets, triangulate);
        coord0  += N * innerRes;
    }
    return nFacets;
}


//
//  Internal initialization methods:
//
void
Tessellation::initialize(Parameterization p,
        int numRates, int const rates[], Options options) {

    //  Initialize trivial members:
    _param = p;

    _triangulate = options.GetTriangulateQuadFacets();

    //  Initialize the full array of rates, returning sum of all edge rates
    int sumOfEdgeRates = initializeRates(numRates, rates);

    //  Initialize the inventory based on the Parameterization type:
    switch (_param.GetType()) {
    case Parameterization::QUAD:
        quadInitializeInventory(sumOfEdgeRates);
        break;
    case Parameterization::TRI:
        triInitializeInventory(sumOfEdgeRates);
        break;
    case Parameterization::QPOLY:
        qpolyInitializeInventory(sumOfEdgeRates);
        break;
    }

    //  Debugging output:
    bool printNonUniform = false; // !_isUniform;
    if (printNonUniform) {
        int N = _param.GetFaceSize();
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

int
Tessellation::initializeRates(int numRates, int const rates[]) {

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
    return numBoundaryEdges;
}

void
Tessellation::quadInitializeInventory(int sumOfEdgeRates) {

    int const * inner = &_innerRates[0];
    int const * outer = &_outerRates[0];

    if (_isUniform) {
        if (inner[0] > 1) {
            _numInteriorPoints = quad::CountInteriorCoords(inner[0]);
            _numFacets = quad::CountUniformFacets(inner[0], _triangulate);
        } else if (_triangulate) {
            _numInteriorPoints = 0;
            _numFacets = 2;
            _splitQuad = true;
        } else {
            _numInteriorPoints = 0;
            _numFacets = 1;
            _singleFace = true;
        }
    } else {
        //
        //  For quads another low-res case is recognized when there are
        //  no interior points, but the face has extra boundary points.
        //  Instead of introducing a center point, the face is considered
        //  to be "segmented" into other faces that cover it without the
        //  addition of any interior vertices.
        //
        //  This currently occurs for a pure 1 x M tessellation -- from
        //  which a quad strip is generated -- but could be extended to
        //  handle the 1 x M inner case with additional points on the
        //  opposing edges.
        //
        if ((inner[0] > 1) && (inner[1] > 1)) {
            _numInteriorPoints = quad::CountInteriorCoords(_innerRates);
            _numFacets = quad::CountNonUniformFacets(_outerRates, _innerRates,
                                                     _triangulate);
        } else if ((outer[0] == inner[0]) && (inner[0] == outer[2]) &&
                   (outer[1] == inner[1]) && (inner[1] == outer[3])) {
            _numInteriorPoints = 0;
            _numFacets = quad::CountSegmentedFacets(_innerRates, _triangulate);
            _segmentedFace = true;
        } else {
            _numInteriorPoints = 1;
            _numFacets = sumOfEdgeRates;
            _triangleFan = true;
        }
    }
    _numBoundaryPoints = sumOfEdgeRates;
    _numTotalPoints    = _numBoundaryPoints + _numInteriorPoints;
}

void
Tessellation::triInitializeInventory(int sumOfEdgeRates) {

    int res = _innerRates[0];

    if (_isUniform) {
        if (res > 1) {
            _numInteriorPoints = tri::CountInteriorCoords(res);
            _numFacets = tri::CountUniformFacets(res);
        } else {
            _numInteriorPoints = 0;
            _numFacets = 1;
            _singleFace = true;
        }
    } else {
        if (res > 2) {
            _numInteriorPoints = tri::CountInteriorCoords(res);
            _numFacets = tri::CountNonUniformFacets(_outerRates, res);
        } else {
            _numInteriorPoints = 1;
            _numFacets = sumOfEdgeRates;
            _triangleFan = true;
        }
    }
    _numBoundaryPoints = sumOfEdgeRates;
    _numTotalPoints    = _numBoundaryPoints + _numInteriorPoints;
}

void
Tessellation::qpolyInitializeInventory(int sumOfEdgeRates) {

    int N   = _param.GetFaceSize();
    int res = _innerRates[0];

    if (_isUniform) {
        if (res > 1) {
            _numInteriorPoints = qpoly::CountInteriorCoords(N, res);
            _numFacets = qpoly::CountUniformFacets(N, res, _triangulate);
        } else if (N == 3) {
            _numInteriorPoints = 0;
            _numFacets = 1;
            _singleFace  = true;
        } else {
            _numInteriorPoints = 1;
            _numFacets = N;
            _triangleFan = true;
        }
    } else {
        if (res > 1) {
            _numInteriorPoints = qpoly::CountInteriorCoords(N, res);
            _numFacets = qpoly::CountNonUniformFacets(N, _outerRates, res,
                                                     _triangulate);
        } else {
            _numInteriorPoints = 1;
            _numFacets = sumOfEdgeRates;
            _triangleFan = true;
        }
    }
    _numBoundaryPoints = sumOfEdgeRates;
    _numTotalPoints    = _numBoundaryPoints + _numInteriorPoints;
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


//
//  Main methods to retrieve samples and facets:
//
int
Tessellation::GetCornerCoords(Coord coords[]) const {

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::GetCornerCoords(coords);
    case Parameterization::TRI:
        return tri::GetCornerCoords(coords);
    case Parameterization::QPOLY:
        return qpoly::GetCornerCoords(_param, coords);
    default:
        assert(0);
    }
    return -1;
}

int
Tessellation::GetBoundaryCoords(int edge, bool v0, bool v1,
                                Coord coords[]) const {

    //  Remember - "edge coords" here excludes coords at the end vertices

    int res = _outerRates[edge];

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::GetBoundaryEdgeCoords(edge, res, v0, v1, coords);
    case Parameterization::TRI:
        return tri::GetBoundaryEdgeCoords(edge, res, v0, v1, coords);
    case Parameterization::QPOLY:
        return qpoly::GetBoundaryEdgeCoords(_param, edge, res, v0, v1, coords);
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
        return quad::GetBoundaryCoords(_outerRates, coords);
    case Parameterization::TRI:
        return tri::GetBoundaryCoords(_outerRates, coords);
    case Parameterization::QPOLY:
        return qpoly::GetBoundaryCoords(_param, _outerRates, coords);
    default:
        assert(0);
    }
    return -1;
}

int
Tessellation::GetInteriorCoords(Coord coords[]) const {

    if (_numInteriorPoints == 0) return 0;

    if (_numInteriorPoints == 1) {
        _param.GetCenterCoord(&coords[0][0], &coords[0][1]);
        return 1;
    }

    switch (_param.GetType()) {
    case Parameterization::QUAD:
        return quad::GetInteriorCoords(_innerRates, coords);
    case Parameterization::TRI:
        return tri::GetInteriorCoords(_innerRates[0], coords);
    case Parameterization::QPOLY:
        return qpoly::GetInteriorCoords(_param, _innerRates[0], coords);
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

    int N = GetFaceSize();

    if (_singleFace) {
        return setSimpleFacet(facets, N);
    }
    if (_triangleFan) {
        return setTriFanFacets(facets, _numFacets);
    }
    if (_splitQuad) {
        return setQuadFacets(facets, 0, 1, 2, 3, _triangulate);
    }

int nFacets = 0;
    switch (_param.GetType()) {
    case Parameterization::QUAD:
        if (_isUniform) {
            nFacets = quad::GetUniformFacets(_innerRates[0],
                                facets, _triangulate);
        } else if (_segmentedFace) {
            nFacets = quad::GetSegmentedFacets(_innerRates,
                                facets, _triangulate);
        } else {
            nFacets = quad::GetNonUniformFacets(_outerRates, _innerRates,
                                _numBoundaryPoints, facets, _triangulate);
        }
        break;
    case Parameterization::TRI:
        if (_isUniform) {
            nFacets = tri::GetUniformFacets(_innerRates[0], facets);
        } else {
            nFacets = tri::GetNonUniformFacets(_outerRates, _innerRates[0],
                                _numBoundaryPoints, facets);
        }
        break;
    case Parameterization::QPOLY:
        if (_isUniform) {
            nFacets = qpoly::GetUniformFacets(N, _innerRates[0],
                                facets, _triangulate);
        } else {
            nFacets = qpoly::GetNonUniformFacets(N, _outerRates, _innerRates[0],
                                _numBoundaryPoints, facets, _triangulate);
        }
        break;
    default:
        assert(0);
    }
if (nFacets != _numFacets) {
    printf("Expecting %d facets -- assigned %d\n", _numFacets, nFacets);
}
assert(nFacets == _numFacets);
if (_triangulate) {
for (int i = 0; i < nFacets; ++i) {
    assert(_triangulate && (facets[i][3] < 0));
}
}
    return nFacets;
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
