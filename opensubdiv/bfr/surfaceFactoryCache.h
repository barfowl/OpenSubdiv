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

#ifndef OPENSUBDIV3_BFR_SURFACE_FACTORY_CACHE_H
#define OPENSUBDIV3_BFR_SURFACE_FACTORY_CACHE_H

#include "../version.h"

#include "../bfr/types.h"

#include <map>
#include <cstdint>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
class PatchTree;
}

namespace Bfr {

//
//  SurfaceFactoryCache is a container for storing/caching instances of the
//  internal representation of complex patches (currently via PatchTrees)
//  used by the SurfaceFactory so that they can be quickly identified and
//  retrieved for reuse.
//
//  Initial/expected use requires simple searches of and additions to the
//  cache by the SurfaceFactory or its Builders.  Longer term, with the
//  possibility of instances of caches being shared between meshes and
//  factories, additional options and/or methods may be warranted to limit
//  what is cached or to prune the cache if it gets too large.
//
class SurfaceFactoryCache {
public:
    SurfaceFactoryCache();
    virtual ~SurfaceFactoryCache();

    size_t Size() const { return _mapBits.size() + _mapHash.size(); }

protected:
    //  Access restricted to the Factory, its Builders, etc.
    friend class SurfaceFactory;

    //  Forward declaration of the Key type
    class Key;

    //  WIP - use of STL-style type names for containers is questionable
    typedef Key            key_type;
    typedef Far::PatchTree data_type;

protected:
    //
    //  Potential overrides by subclasses for thread-safety:
    //
    virtual data_type const * Find(key_type const & key) const;
    virtual data_type const * Add(key_type const & key, data_type const * data);

    //
    //  Common implementation used by all subclasses:
    //
    data_type const * find(key_type const & key) const;
    data_type const * add(key_type const & key, data_type const * data);

protected:
    //
    //  Keys associated with unique topologies in the cache consist of an
    //  integer value that may be computed in at least two different ways:
    //  the most common, simple topologies are encoded into a simple set
    //  of bitfields, while those more complex require a hashing function.
    //
    class Key {
    public:
        typedef std::uint64_t IntType;

        enum Format { INVALID, BITFIELDS, HASHED };

    public:
        Key() : _value(0), _format(INVALID) { }

        bool IsValid() const { return (_format != INVALID); }

        Format  GetFormat() const { return _format; }
        IntType GetValue()  const { return _value; }

        void SetFormat(Format format) { _format = format; }
        void SetValue(IntType value)  { _value  = value; }

    private:
        IntType _value;
        Format  _format;
    };

private:
    typedef std::map<Key::IntType, data_type const *>  map_type;

    void clear(map_type * map);
    void clear();

private:
    map_type _mapBits;
    map_type _mapHash;
};

//
//  Template for simple thread-safe subclasses of SurfaceFactoryCache:
//
//  Separate read and write locks are provided to support mutex types
//  allowing shared (read) or exclusive (write) access.
//
template <class MUTEX_TYPE, class READ_LOCK_GUARD_TYPE,
                            class WRITE_LOCK_GUARD_TYPE>
class SurfaceFactoryCacheThreaded : public SurfaceFactoryCache {
public:
    SurfaceFactoryCacheThreaded() : SurfaceFactoryCache() { }
    virtual ~SurfaceFactoryCacheThreaded() { }

protected:
    virtual data_type const * Find(key_type const & key) const {
        READ_LOCK_GUARD_TYPE lockGuard(_mutex);
        return find(key);
    }

    virtual data_type const * Add(key_type const & key, data_type const * data){
        WRITE_LOCK_GUARD_TYPE lockGuard(_mutex);
        return add(key, data);
    }

private:
    MUTEX_TYPE mutable _mutex;
};

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_SURFACE_FACTORY_CACHE_H */
