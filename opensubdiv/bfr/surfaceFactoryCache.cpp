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

#include "../bfr/surfaceFactoryCache.h"
#include "../far/patchTree.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {

//
//  Trivial constructor and destructor:
//
SurfaceFactoryCache::SurfaceFactoryCache() : _mapBits(), _mapHash() {
}

SurfaceFactoryCache::~SurfaceFactoryCache() {

    clear();
}

//
//  The cache owns the data that is assigned to it, so destroy all entries
//  when explicitly cleared or the destructor is called:
//
void
SurfaceFactoryCache::clear() {

    clear(&_mapBits);
    clear(&_mapHash);
}

void
SurfaceFactoryCache::clear(map_type * mapPtr) {

    map_type & map = *mapPtr;

    for (map_type::iterator it = map.begin(); it != map.end(); ++it) {
        delete it->second;
    }
    mapPtr->clear();
}

//
//  These definitions do not yet account for the two types of keys --
//  which in turn require a map corresponding to each type:
//
SurfaceFactoryCache::data_type const *
SurfaceFactoryCache::find(Key const & key) const {

    assert(key.IsValid());

    map_type const & map = (key.GetFormat() == Key::BITFIELDS)
                         ? _mapBits : _mapHash;

    map_type::const_iterator mapIt = map.find(key.GetValue());
    return (mapIt == map.end()) ? 0 : mapIt->second;
}

SurfaceFactoryCache::data_type const *
SurfaceFactoryCache::add(Key const & key, data_type const * data) {

    assert(key.IsValid());

    map_type & map = (key.GetFormat() == Key::BITFIELDS)
                   ? _mapBits : _mapHash;

    map_type::const_iterator mapIt = map.find(key.GetValue());
    if (mapIt != map.end()) return mapIt->second;

    map[key.GetValue()] = data;
    return data;
}

//
//  Virtual methods -- intended to be overridden for thread-safety:
//
SurfaceFactoryCache::data_type const *
SurfaceFactoryCache::Find(Key const & key) const {

    return find(key);
}

SurfaceFactoryCache::data_type const *
SurfaceFactoryCache::Add(Key const & key, data_type const * data) {

    return add(key, data);
}

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv
