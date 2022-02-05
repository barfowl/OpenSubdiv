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
#include <shared_mutex>

#include <opensubdiv/bfr/surfaceFactory.h>
#include <opensubdiv/bfr/surfaceFactoryCache.h>
#include <opensubdiv/far/topologyRefiner.h>

//
//  Definition of a subclass of SurfaceFactory for Far::TopologyRefiner:
//
//  A subclass is free to define its own construction interface (given its
//  unique mesh type) and to extend its public interface in any way that
//  suits the mesh.
//
//  Given each representation typically has its own way of representing
//  primvars, using explicit primvar types in construction or other
//  queries is likely -- especially face-varying primvars, whose topology
//  is unique.  For example, it may be useful to have the constructor
//  specify a single face-varying primvar to be used for UVs when more
//  than one are available.
//
//  Unfortunately, the Far::TopologyRefiner can use integers for its face-
//  varying channels, which the SurfaceFactory can use directly, so a more
//  explicit association of primvars with integers is not necessary here.
//
class SubclassOfSurfaceFactory : public OpenSubdiv::Bfr::SurfaceFactory {
public:
    typedef OpenSubdiv::Far::TopologyRefiner TopologyRefiner;
    typedef OpenSubdiv::Bfr::Index           Index;

public:
    //
    //  Subclass-specific constructor:
    //
    SubclassOfSurfaceFactory(TopologyRefiner const & mesh,
                             Options options = Options());
    virtual ~SubclassOfSurfaceFactory();

    //
    //  Additional subclass-specific public methods:
    //
    TopologyRefiner const & GetMesh() const { return _mesh; }

    //
    //  Convenience queries to verify bounds of integer arguments used by
    //  the SurfaceFactory, i.e. face indices and face-varying IDs:
    //
    int GetNumFaces() const;
    int GetNumFVarChannels() const;

protected:
    //
    //  Virtual methods to satisfy topological requirements:
    //
    bool isFaceHole( Index faceIndex) const;
    int  getFaceSize(Index faceIndex) const;

    int getFaceVertexIndices(   Index faceIndex,
                                Index vertexIndices[]) const;
    int getFaceFVarValueIndices(Index faceIndex, FVarID fvarID,
                                Index fvarValueIndices[]) const;

    int populateFaceVertexDescriptor(Index faceIndex, int faceVertex,
                            OpenSubdiv::Bfr::VertexDescriptor * vd) const;

    int getFaceVertexIncidentFaceVertexIndices(
                            Index faceIndex, int faceVertex,
                            Index vertexIndices[]) const;
    int getFaceVertexIncidentFaceFVarValueIndices(
                            Index faceIndex, int faceVertex, FVarID fvarID,
                            Index fvarValueIndices[]) const;

protected:
    //
    //  Virtual method to access the local topology cache:
    //
    //  The virtual method to retrieve a reference to a local cache will
    //  be invoked when/if necessary. Recall that an externally defined
    //  shared cache can be specified in the Options and takes precedence.
    //
    OpenSubdiv::Bfr::SurfaceFactoryCache * getInternalCache() const {
        return & _localCache;
    }

private:
    //
    //  Internal supporting method to gather indices -- either vertex or
    //  face-varying -- since both are accessed similarly:
    //
    int getFaceVaryingChannel(FVarID fvarID) const;

    int getFaceVertexPointIndices(Index faceIndex, int faceVertex,
                                  Index indices[], int vtxOrFVarChannel) const;

private:
    //
    //  Typically a subclass adds member variables for an instance of a
    //  mesh and an instance of a local cache:
    //
    TopologyRefiner const & _mesh;

    //  The ownership of the local cache is deferred to the subclass in
    //  part so the subclass can choose one of its preferred type --
    //  depending on the level of thread-safety required.
    //
    //  Bfr::SurfaceFactoryCache is a base class that allows for simple
    //  declaration of thread-safe subclasses via templates. If not
    //  requiring the cache to be thread-safe, using the base class is
    //  sufficient (as is done here).  Use of threading extensions in
    //  more recent compilers allows for separate read and write locks,
    //  e.g.:
    //
    //  typedef Bfr::ThreadSafeSurfaceFactoryCache
    //              < std::shared_mutex, std::shared_lock<std::shared_mutex>,
    //                                   std::unique_lock<std::shared_mutex> >
    //              LocalFactoryCacheType;
    //
    typedef OpenSubdiv::Bfr::SurfaceFactoryCache LocalFactoryCacheType;

    //  Remember to declare the cache as mutable:
    LocalFactoryCacheType mutable _localCache;
};


//
//  Simple inline extensions to the public interface:
//
inline int
SubclassOfSurfaceFactory::GetNumFaces() const {
    return _mesh.GetLevel(0).GetNumFaces();
}

inline int
SubclassOfSurfaceFactory::GetNumFVarChannels() const {
    return _mesh.GetNumFVarChannels();
}
