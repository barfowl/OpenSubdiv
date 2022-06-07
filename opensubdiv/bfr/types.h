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

#include "../far/types.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Typedefs for indices and limits used either directly with or for
//  purposes similar to those of Far classes:
//
//  WIP - Bfr methods never expose invalid indices, so no need for this test
//      - and if only limit definitions remain, consider <bfr/limits.h>
template <typename INDEX>
inline bool IndexIsValid(INDEX index) { return (index >= 0); }

//
//  Limits on vertex valence and face size:
//  WIP - use of suffix/prefix here warrants review
//      - "limit surface" used much more in Bfr, so avoiding "limit"
//
static const int MAX_VALENCE   = Far::VALENCE_LIMIT;
static const int MAX_FACE_SIZE = Far::VALENCE_LIMIT;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;
} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_TYPES_H */
