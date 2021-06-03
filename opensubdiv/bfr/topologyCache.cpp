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

#include "../far/patchTree.h"

#include "../bfr/topologyCache.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Trivial constructor and destructor:
//
TopologyCache::TopologyCache() : _map() {
}

TopologyCache::~TopologyCache() {

    clear();
}

//
//  The cache owns the data that is assigned to it, so destroy all entries
//  when explicitly cleared or the destructor is called:
//
void
TopologyCache::clear() {

    for (map_type::iterator it = _map.begin(); it != _map.end(); ++it) {
        delete it->second;
    }
    _map.clear();
}

//
//  These definitions are rough stand-ins given the current loose definition
//  of the "key" type -- currently an unsigned long int (64-bit intended)...
//
TopologyCache::data_type const *
TopologyCache::find(Key const & key) const {

    assert(key.IsValid());

    map_type::const_iterator mapIt = _map.find(key.hashBits);
    return (mapIt == _map.end()) ? 0 : mapIt->second;
}

TopologyCache::data_type const *
TopologyCache::add(Key const & key, data_type const * data) {

    assert(key.IsValid());

    map_type::const_iterator mapIt = _map.find(key.hashBits);
    if (mapIt != _map.end()) return mapIt->second;

    _map[key.hashBits] = data;
    return data;
}

//
//  Virtual methods -- intended to be overridden for thread-safety:
//
TopologyCache::data_type const *
TopologyCache::Find(Key const & key) const {

    return find(key);
}

TopologyCache::data_type const *
TopologyCache::Add(Key const & key, data_type const * data) {

    return add(key, data);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv
