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

#ifndef OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H
#define OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H

#include "../version.h"

#include "../bfr/limitSurfaceFactory.h"
#include "../bfr/topologyCache.h"


namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Far {
    class TopologyRefiner;
}

namespace Bfr {

//
//  Intermediate subclass of LimitSurfaceFactory using Far::TopologyRefiner
//  as the connected mesh class.
//
//  This subclass provides additional interface specific to TopologyRefiner
//  and all requirements of the base class with the exception of the local
//  TopologyCache -- that is deferred to a template below so that clients
//  can easily declare subclasses using a preferred thread-safe cache type.
//
//  Creating this intermediate abstract class allows us to template for the
//  TopologyCache without templating the entire subclass implementation.
//
class RefinerLimitSurfaceFactoryBase : public LimitSurfaceFactory {
public:
    //
    //  Subclass-specific constructor:
    //
    RefinerLimitSurfaceFactoryBase(Far::TopologyRefiner const & mesh,
                               Options options = Options());
    virtual ~RefinerLimitSurfaceFactoryBase();

    //  Additional subclass-specific public queries:
    Far::TopologyRefiner const & GetMesh() const { return _mesh; }

    int GetNumFaces() const { return _numFaces; }

    int GetNumFVarChannels() const { return _numFVarChannels; }

public:
    //  WIP - temporary public methods for development and debugging
    //      - all of these "unsupported" cases will eventually be
    //        supported and the associated methods removed.
    bool IsFaceUnsupported(Index faceIndex) const;

    bool HasUnsupportedFaces() const;
    int  GetNumUnsupportedFaces() const;

    int  GetNumNonManifoldFaces() const;
    int  GetNumVal2InteriorFaces() const;

protected:
    //
    //  Virtual methods to satisfy topological requirements:
    //
    bool isFaceHole( Index faceIndex) const;
    int  getFaceSize(Index faceIndex) const;

    int getFaceVertexIndices(   Index faceIndex,
                                Index vertexIndices[]) const;
    int getFaceFVarValueIndices(Index faceIndex,
                                Index fvarValueIndices[], int fvarID) const;

    int populateFaceVertexTopology(Index faceIndex, int faceVertex,
                                   VertexTopology & vertexTopology) const;

    int getFaceVertexIncidentFaceVertexIndices(
                            Index faceIndex, int faceVertex,
                            Index vertexIndices[]) const;
    int getFaceVertexIncidentFaceFVarValueIndices(
                            Index faceIndex, int faceVertex,
                            Index fvarValueIndices[], int fvarID) const;

private:
    //
    //  Additional supporting methods:
    //
    int getFaceVertexIndices(Index faceIndex, int faceVertex,
                             Index indices[], int vertexOrFVarChannel) const;

private:
    //  Additional members for the subclass:
    Far::TopologyRefiner const & _mesh;

    int _numFaces;
    int _numFVarChannels;
};


//
//  Template for concrete subclasses with the addition of management of an
//  internal cache. This makes it easy for clients to declare subclasses
//  for thread-safe types of TopologyCache as a simple typedef:
//
template <class CACHE_TYPE = TopologyCache>
class RefinerLimitSurfaceFactoryCached: public RefinerLimitSurfaceFactoryBase {
public:
    RefinerLimitSurfaceFactoryCached(Far::TopologyRefiner const & mesh,
                                     Options options = Options()) :
            RefinerLimitSurfaceFactoryBase(mesh, options),
            _localCache() { }
    ~RefinerLimitSurfaceFactoryCached() { }

protected:
    TopologyCache * getInternalTopologyCache() const { return & _localCache; }

private:
    CACHE_TYPE mutable _localCache;
};

//  WIP - naming is uncertain here, this typedef may eventually be removed
typedef RefinerLimitSurfaceFactoryCached<TopologyCache>
        RefinerLimitSurfaceFactory;

} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_REFINER_LIMIT_SURFACE_FACTORY_H */
