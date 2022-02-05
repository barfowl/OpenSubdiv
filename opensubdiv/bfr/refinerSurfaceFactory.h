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

#ifndef OPENSUBDIV3_BFR_REFINER_SURFACE_FACTORY_H
#define OPENSUBDIV3_BFR_REFINER_SURFACE_FACTORY_H

#include "../version.h"

#include "../bfr/surfaceFactory.h"
#include "../bfr/surfaceFactoryCache.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class TopologyRefiner;
    class PatchBuilder;  //  WIP - should be temporary
}

namespace Bfr {

//
//  Intermediate subclass of SurfaceFactory using Far::TopologyRefiner as
//  the connected mesh representation.
//
//  This subclass provides additional public interface methods specific to
//  TopologyRefiner along with most requirements of the base class related
//  to the TopologyRefiner. The requirement not completed here is that of
//  providing a local SurfaceFactoryCache -- that is deferred to a template
//  below so that a wide range of clients desiring a thread-safe cache can
//  easily declare a subclass for a preferred thread-safe type.
//
class RefinerSurfaceFactoryBase : public SurfaceFactory {
public:
    //
    //  Subclass-specific constructor:
    //
    RefinerSurfaceFactoryBase(Far::TopologyRefiner const & mesh,
                               Options options = Options());
    virtual ~RefinerSurfaceFactoryBase();

    //
    //  Additional subclass-specific public methods:
    //
    Far::TopologyRefiner const & GetMesh() const { return _mesh; }

    //  Convenience queries to verify bounds of face indices and face-
    //  varying channel indices:
    int GetNumFaces() const { return _numFaces; }
    int GetNumFVarChannels() const { return _numFVarChannels; }

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
                                     VertexDescriptor * vertexDescriptor) const;

    int getFaceVertexIncidentFaceVertexIndices(
                            Index faceIndex, int faceVertex,
                            Index vertexIndices[]) const;
    int getFaceVertexIncidentFaceFVarValueIndices(
                            Index faceIndex, int faceVertex, FVarID fvarID,
                            Index fvarValueIndices[]) const;

    //
    //  Optional overrides for accelerating regular patches:
    //
    bool getFaceNeighborhoodVertexIndicesIfRegular(
                            Index faceIndex,
                            Index vertexIndices[]) const;

    bool getFaceNeighborhoodFVarValueIndicesIfRegular(
                            Index faceIndex,
                            FVarID fvarID, Index fvarValueIndices[]) const;

private:
    //
    //  Additional supporting methods:
    //
    int getFaceVaryingChannel(FVarID fvarID) const;

    int getFaceVertexPointIndices(Index faceIndex, int faceVertex,
                                  Index indices[], int vtxOrFVarChannel) const;

    int getFacePatchPointIndices(Index faceIndex,
                                 Index indices[], int vtxOrFVarChannel) const;

private:
    //  Additional members for the subclass:
    Far::TopologyRefiner const & _mesh;

    int _numFaces;
    int _numFVarChannels;
};


//
//  Template for concrete subclasses with the addition of management of an
//  internal cache. This makes it possible for clients to simply declare a
//  subclass that manages an internal thread-safe SurfaceFactoryCache using
//  their preferred thread-safe type.
//
template <class CACHE_TYPE = SurfaceFactoryCache>
class RefinerSurfaceFactoryCached : public RefinerSurfaceFactoryBase {
public:
    RefinerSurfaceFactoryCached(Far::TopologyRefiner const & mesh,
                                Options options = Options()) :
            RefinerSurfaceFactoryBase(mesh, options),
            _localCache() { }
    ~RefinerSurfaceFactoryCached() { }

protected:
    SurfaceFactoryCache * getInternalCache() const { return & _localCache; }

private:
    CACHE_TYPE mutable _localCache;
};

//  WIP - naming is uncertain here, this typedef may eventually be removed
typedef RefinerSurfaceFactoryCached<SurfaceFactoryCache> RefinerSurfaceFactory;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_REFINER_SURFACE_FACTORY_H */
