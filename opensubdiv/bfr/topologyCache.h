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

#ifndef OPENSUBDIV3_BFR_TOPOLOGY_CACHE_H
#define OPENSUBDIV3_BFR_TOPOLOGY_CACHE_H

#include "../version.h"

#include "../bfr/types.h"

#include <map>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
class PatchTree;
}

namespace Bfr {

//
//  TopologyCache is a container for storing/caching instances of the
//  internal representation of complex patches (currently via PatchTrees)
//  so that they can be quickly identified and retrieved by some topology
//  hashing mechanism.
//
//  Initial/expected use requires simple searches of and additions to the
//  Cache by the LimitSurfaceFactory or its helpers.  Longer term, with
//  the possibility of instances of caches being shared between meshes and
//  factories, additional options and/or methods may be warranted to limit
//  what is cached or to prune the cache if it gets too large.
//
class TopologyCache {
public:
    TopologyCache();
    virtual ~TopologyCache();

    size_t Size() const { return _map.size(); }

protected:
    friend class IrregularPatchBuilder;

    struct Key {
        Key() : hashBits(0) { }

        bool IsValid() const { return hashBits != 0; }

        //  WIP - an enum is planned here, in addition to the int, to
        //        to distinguish different groups of topology
        //      - "simple hashing" just fills a 64-bit field for common cases
        //      - "complex hashing" will use a hashing function applied to
        //        buffers assembling creasing information, sizes of incident
        //        faces, etc. to capture the full topology
        //      - so two separate maps are planned for these two categories
        unsigned long hashBits;
    };

    typedef Key            key_type;
    typedef Far::PatchTree data_type;

    virtual data_type const * Find(key_type const & key) const;
    virtual data_type const * Add(key_type const & key, data_type const * data);

    data_type const * find(key_type const & key) const;
    data_type const * add(key_type const & key, data_type const * data);

private:
    void clear();

private:
    typedef unsigned long map_key_type;

    typedef std::map<map_key_type, data_type const *>  map_type;

    map_type   _map;
};

//
//  Template for simple thread-safe subclasses of TopologyCache:
//
//  Separate read and write locks are provided to support mutex types
//  allowing shared (read) or exclusive (write) access.
//
template <class MUTEX_TYPE, class SCOPED_READ_LOCK_TYPE,
                            class SCOPED_WRITE_LOCK_TYPE>
class ThreadSafeTopologyCache : public TopologyCache {
public:
    ThreadSafeTopologyCache() : TopologyCache() { }
    ~ThreadSafeTopologyCache() { }

protected:
    data_type const * Find(key_type const & key) const {
        SCOPED_READ_LOCK_TYPE lockGuard(_mutex);
        return find(key);
    }

    data_type const * Add(key_type const & key, data_type const * data) {
        SCOPED_WRITE_LOCK_TYPE lockGuard(_mutex);
        return add(key, data);
    }

private:
    MUTEX_TYPE mutable _mutex;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_TOPOLOGY_CACHE_H */
