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

#ifndef OPENSUBDIV3_BFR_TESSELLATION_H
#define OPENSUBDIV3_BFR_TESSELLATION_H

#include "../version.h"

#include "../bfr/parameterization.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Tessellation is a simple class that provides topological information
//  for a specified tessellation pattern of a given parameterization.
//
class Tessellation {
public:
    //
    //  Options configure a Tessellation to determine the nature of its
    //  results and to specify the structure of the coordinate and facet
    //  index buffers that its methods will populate.
    //
    //  The sizes and strides of the target buffers should be specified
    //  explicitly as they are not inferred by the presence of other
    //  options.
    //
    class Options {
    public:
        Options() : _preserveQuads(false), _facetSize4(false),
                    _coordStride(0), _facetStride(0) { }

        //  Choice of triangulation (default) or preservation of quads
        //  with quad-based subdivision:
        void PreserveQuads(bool on) { _preserveQuads = on; }
        bool PreserveQuads() const  { return _preserveQuads; }

        //  Option for the number of indices associated with each facet
        //  (i.e. the number of indices set by the Tessellation) which can
        //  be 3 or 4 (default is 3 and remains so unless 4 specified):
        void SetFacetSize(int numIndices) { _facetSize4 = (numIndices == 4); }
        int  GetFacetSize() const         { return 3 + (int)_facetSize4; }

        //  Option for stride between facets within a larger set:
        void SetFacetStride(int stride)  { _facetStride = (short) stride; }
        int  GetFacetStride() const      { return _facetStride; }

        //  Option for stride between (u,v) pairs within a larger set:
        void SetCoordStride(int stride) { _coordStride = (short) stride; }
        int  GetCoordStride() const     { return _coordStride; }

    private:
        unsigned int _preserveQuads : 1;
        unsigned int _facetSize4    : 1;

        short _coordStride;
        short _facetStride;
    };

public:
    //
    //  Constructors require a Parameterization of a face, a set of one or
    //  more tessellation rates, and a standard set of options.
    //
    //  A simple constructor provides a single uniform tessellatin rate
    //  applicable to all faces.  For non-uniform tessellations, a more
    //  general constructor can provide separate tessellation rates for
    //  each edge and/or one or more inner tessellation rates (two for
    //  quads only, one for all others). Certain subsets of these rates
    //  can be specified -- leaving others to be inferred.
    //
    //  A non-uniform Tessellation of a face with N edges interprets the
    //  given number of tessellation rates as follows -- for all faces
    //  (regardless of N):
    //
    //      numRates ==  1:   uniform inner rate, outer similarly uniform
    //      numRates ==  N:   explicit outer rates per edge, inner inferred
    //      numRates == N+1:  explicit outer rates, uniform inner rate
    //
    //  and for quads only (N == 4, with its two independent inner rates):
    //
    //      numRates ==  2:   explict inner rates, outer rates inferred
    //      numRates == N+2:  explicit outer rates, explicit inner rates
    //
    //  Note that the values for outer and inner rates follow conventions
    //  elsewhere, e.g. a uniform tessellation rate of X for a triangle
    //  corresponds to the more explicit specification of X as the outer
    //  rate for each of its edges and X for the inner rate.
    //
    //  Like other classes, constructors can produce invalid instances if
    //  given obviously invalid arguments, e.g. an invalid Parameterization,
    //  non-positive tessellation rates, etc.
    //
    Tessellation(Parameterization const & p, int uniformRate,
                 Options options = Options());
    Tessellation(Parameterization const & p, int numRates, int const rates[],
                 Options options = Options());
    ~Tessellation();

    bool IsValid() const { return _isValid; }

    //
    //  General queries:
    //
    Parameterization GetParameterization() const { return _param; }

    int GetFaceSize() const { return _param.GetFaceSize(); }

    int GetRates(int rates[]) const;

    bool IsUniform() const { return _isUniform; }

    //
    //  Queries to determine the number of sample points involved in the
    //  tessellation pattern -- overall or for various features:
    //
    int GetNumCoords() const { return _numInteriorPoints + _numBoundaryPoints; }

    int GetNumBoundaryCoords() const { return _numBoundaryPoints; }
    int GetNumInteriorCoords() const { return _numInteriorPoints; }

    int GetNumEdgeCoords(int edge) const { return _outerRates[edge] - 1; }

    //
    //  Methods to identify coordinates of sample points for all or specific
    //  features of the parameterization.  All such methods return the number
    //  of coordinates returned, so the above methods returning the size only
    //  are not necessary if buffers for the resulting coords have already
    //  been adequately sized:
    //
    template <typename REAL>
    int GetCoords(REAL coordBuffer[]) const;

    template <typename REAL>
    int GetBoundaryCoords(REAL coordBuffer[]) const;
    template <typename REAL>
    int GetInteriorCoords(REAL coordBuffer[]) const;

    template <typename REAL>
    int GetVertexCoord(int vertex, REAL coordBuffer[]) const;
    template <typename REAL>
    int GetEdgeCoords( int edge,   REAL coordBuffer[]) const;

    //
    //  Methods to query the number and values of facets, and a few methods
    //  to tranform (offset or remap) facet indices for various uses:
    //
    int GetNumFacets() const { return _numFacets; }
    int GetFacetSize() const { return _facetSize; }

    int GetFacets(int facetIndexBuffer[]) const;

    void TransformFacetIndices(int facetIndexBuffer[],
                               int commonOffset);
    void TransformFacetIndices(int facetIndexBuffer[],
                               int boundaryOffset, int interiorOffset);
    void TransformFacetIndices(int facetIndexBuffer[],
                               int const boundaryIndices[],
                               int       interiorOffset);
    void TransformFacetIndices(int facetIndexBuffer[],
                               int const boundaryIndices[],
                               int const interiorIndices[]);

private:
    //  Private initialization methods:
    bool validateArguments(Parameterization const & p,
                    int nRates, int const rates[], Options const & options);

    void initialize(Parameterization const & p,
                    int nRates, int const rates[], Options const & options);

    void initializeDefaults();
    int  initializeRates(int nRates, int const rates[]);
    void initializeInventoryForParamTri(int sumOfOuterRates);
    void initializeInventoryForParamQuad(int sumOfOuterRates);
    void initializeInventoryForParamQPoly(int sumOfOuterRates);

private:
    //  Private members:
    Parameterization _param;

    unsigned short _isValid       :  1;
    unsigned short _isUniform     :  1;
    unsigned short _triangulate   :  1;
    unsigned short _singleFace    :  1;
    unsigned short _segmentedFace :  1;
    unsigned short _triangleFan   :  1;
    unsigned short _splitQuad     :  1;

    short _facetSize;
    int   _facetStride;
    int   _coordStride;

    int _numGivenRates;
    int _numBoundaryPoints;
    int _numInteriorPoints;
    int _numFacets;

    int  _innerRates[2];
    int* _outerRates;
    int  _outerRatesLocal[4];
};

//
//  Inline implementations:
//
template <typename REAL>
inline int
Tessellation::GetVertexCoord(int vertex, REAL coord[]) const {
    _param.GetVertexCoord(vertex, coord);
    return 1;
}

template <typename REAL>
inline int
Tessellation::GetCoords(REAL coordBuffer[]) const {
    int nCoords = GetBoundaryCoords(coordBuffer);
    nCoords += GetInteriorCoords(coordBuffer + nCoords * _coordStride);
    return nCoords;
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_TESSELLATION */
