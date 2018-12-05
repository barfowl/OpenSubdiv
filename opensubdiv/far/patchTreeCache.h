//
//   Copyright 2018 DreamWorks Animation LLC
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

#ifndef OPENSUBDIV3_FAR_PATCH_TREE_CACHE_H
#define OPENSUBDIV3_FAR_PATCH_TREE_CACHE_H

#include "../version.h"

#include "../far/patchTree.h"

#include <map>

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {

//
//  PatchTreeCache is a container for storing/caching PatchTrees that can be
//  quickly identified and retrieved by some topology hashing mechanism.
//
//  All of this is essentially a place holder for now...
//
class PatchTreeCache {
public:
    PatchTreeCache() { }
    ~PatchTreeCache() { }

    //  Exposing the actual container used internally is unwise, so just
    //  provide simple/minimal public interface:
    //
    PatchTree const * Find(PatchTree::Key const & key) const;

    void Add(PatchTree::Key const & key, PatchTree const * tree);

private:
    typedef std::map<unsigned long, PatchTree const *> PatchTreeMap;

    PatchTreeMap _map;
};

//
//  These stubs are not functional -- they exist to illustrate potential usage
//  within the Factory classes.
//
inline PatchTree const *
PatchTreeCache::Find(PatchTree::Key const & key) const {
    if (key.hashBits) {
        PatchTreeMap::const_iterator treeIt = _map.find(key.hashBits);
        return (treeIt == _map.end()) ? 0 : treeIt->second;
    }
    return 0;
}

inline void
PatchTreeCache::Add(PatchTree::Key const & key, PatchTree const * tree) {
    if (key.hashBits) {
        _map[key.hashBits] = tree;
    }
}

} // end namespace Far

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_FAR_PATCH_TREE_CACHE_H */
