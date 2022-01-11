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

#ifndef OPENSUBDIV3_BFR_TYPES_H
#define OPENSUBDIV3_BFR_TYPES_H

#include "../version.h"

#include "../vtr/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  WORK IN PROGRESS...
//
//  Simple (temporary?) types used with evaluation and tessellation:
//      - interfaces often require dealing with arrays of these
//      - explicit types may be removed in favor of more primitive types
//      - use of float[] and int[] or templates may be preferable
//
struct Coord {
    Coord() { }
    Coord(float u, float v) { uv[0] = u, uv[1] = v; }

    float const & operator[](int i) const { return uv[i]; }
    float       & operator[](int i)       { return uv[i]; }

    float uv[2];  // will need template versions of this <typename REAL>
};

struct Facet {
    Facet() { }
    Facet(int a, int b, int c, int d = -1) { v[0] = a, v[1] = b, v[2] = c, v[3] = d; }

    int const & operator[](int i) const { return v[i]; }
    int       & operator[](int i)       { return v[i]; }

    int v[4];
};


//
//  Typedefs for indices that are inherited from the Vtr level -- eventually
//  these primitive Vtr types may be declared at a lower, more public level.
//
typedef Vtr::Index       Index;
typedef Vtr::LocalIndex  LocalIndex;

typedef Vtr::IndexArray       IndexArray;
typedef Vtr::LocalIndexArray  LocalIndexArray;

typedef Vtr::ConstIndexArray       ConstIndexArray;
typedef Vtr::ConstLocalIndexArray  ConstLocalIndexArray;

inline bool IndexIsValid(Index index) { return Vtr::IndexIsValid(index); }

static const Index INDEX_INVALID = Vtr::INDEX_INVALID;
static const int   VALENCE_LIMIT = Vtr::VALENCE_LIMIT;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_TYPES_H */
