//
//   Copyright 2019 DreamWorks Animation LLC.
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

#include <opensubdiv/far/topologyRefinerFactory.h>
#include <opensubdiv/far/topologyLevel.h>


//
//  Local typedef of TopologyRefinerFactory<TopologyLevel> for convenience:
//
typedef OpenSubdiv::OPENSUBDIV_VERSION::Far::TopologyRefinerFactory
            <OpenSubdiv::OPENSUBDIV_VERSION::Far::TopologyLevel>   CustomRefinerFactory;


//
//  Required specializations for TopologyRefinerFactory<TopologyLevel>:
//
namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {
namespace Far {

template <>
bool
TopologyRefinerFactory<TopologyLevel>::resizeComponentTopology(
    TopologyRefiner & refiner, TopologyLevel const & level);

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignComponentTopology(
    TopologyRefiner & refiner, TopologyLevel const & level);

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignComponentTags(
    TopologyRefiner & refiner, TopologyLevel const & level);

template <>
bool
TopologyRefinerFactory<TopologyLevel>::assignFaceVaryingTopology(
    TopologyRefiner & refiner, TopologyLevel const & level);

template <>
void
TopologyRefinerFactory<TopologyLevel>::reportInvalidTopology(
    TopologyError errCode, char const * msg, TopologyLevel const & level);

} // end namespace Far
} // end namespace OPENSUBDIV_VERSION
} // end namespace OpenSubdiv
