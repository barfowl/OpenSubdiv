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

#ifndef OPENSUBDIV3_BFR_FACE_BUILDERS_H
#define OPENSUBDIV3_BFR_FACE_BUILDERS_H

#include "../version.h"

#include "../bfr/faceDescriptors.h"
#include "../far/patchDescriptor.h"

namespace OpenSubdiv {
namespace OPENSUBDIV_VERSION {

namespace Bfr {
namespace internal {

//
//  This header includes internal subclasses of the various FaceDescriptor
//  classes that are used to help construct the limit surface for a face.
//
//  FaceBuilders derive from FaceDescriptors in order to access all data
//  members.  FaceBuilders may augment the class with additional members,
//  but at the very least, they provide additional methods to facilitate
//  limit surface construction.
//
//  WIP - the "is a" versus "has a" nature of FaceBuilders relative to
//  FaceDescriptors is under consideration.  Other changes are being
//  considered in light of the merger of the manifold and non-manifold
//  cases into one.
//  

//
//  Builder deriving from the RegularFaceDescriptor:
//
class RegularFaceBuilder : public RegularFaceDescriptor {
public:
    typedef Far::PatchDescriptor::Type PatchType;

    RegularFaceBuilder() : RegularFaceDescriptor() { }
    virtual ~RegularFaceBuilder() { }

public:
    //
    //  Methods to identify the control vertices and provide the boundary
    //  mask and patch points (remapped to local indices, not the CVs):
    //
    int GetFaceSize()  const { return _faceSize; }
    int GetPatchSize() const { return _patchSize; }

    PatchType GetPatchType() const;
    int GetBoundaryMask() const { return _boundaryMaskSet ? _boundaryMask : -1; }

    int GetFaceVertexIndices(int vertIndices[]) const;
    int GetPatchPointIndices(int vertIndices[]) const;

    //  Methods to retrieve indices for face-varying patches:
    bool HasFVarIndices() const { return _fvarPatchSet; }

    int GetFaceFVarValueIndices(int fvarIndices[]) const;
    int GetFVarPatchPointIndices(int fvarIndices[]) const;

    //  WIP - not sure we will be keeping these...  Intended to re-express
    //  the full set of patch points into a set of unique and oriented
    //  (0-ring, 1-ring) control vertices with retrieval methods for both
    //  the control vertices and respective patch points:
    int GetNumControlVertices() const { return _numCVs; }
    int GetControlVertexIndices(int vertexIndices[]) const;
    int GetLocalPatchPoints(int localPatchPoints[]) const;
};


//
//  Builder deriving from the ManifoldFaceDescriptor:
//
class ManifoldFaceBuilder : public ManifoldFaceDescriptor {
public:
    ManifoldFaceBuilder() : ManifoldFaceDescriptor() { }
    virtual ~ManifoldFaceBuilder() { }

public:
    //
    //  Methods to identify the control vertices and control faces (in
    //  local terms) of the local neighborhood of the face:
    //
    int GetFaceSize() const { return _faceSize; }
    int GetFaceVertexIndices(int vertIndices[]) const;

    int GetNumControlVertices() const { return _numVerts; }
    int GetNumControlFaces() const { return _numFaces; }
    int GetNumControlFaceVertices() const { return _numFaceVerts; }

    int GetControlVertexIndices(int vertIndices[], int rotation = 0) const;
    int GetLocalFaceVertices(int localFaceVertIndices[], int sizes[]) const;

    bool HasFVarIndices() const { return _fvarIndicesSet; }
    int GetFaceFVarValueIndices(int fvarIndices[]) const;
int GetControlFVarValueIndices(int fvarIndices[], int rotation = 0) const;

    TopologyCache::Key ComputeTopologyKey() const;

    //  Implementation pending...
    bool IsRegular(RegularFaceDescriptor &) const { return false; }
};


//
//  Builder deriving from the NonManifoldFaceDescriptor:
//
class NonManifoldFaceBuilder : public NonManifoldFaceDescriptor {
public:
    NonManifoldFaceBuilder() : NonManifoldFaceDescriptor() { }
    virtual ~NonManifoldFaceBuilder() { }

public:
    //
    //  Methods identify the control vertices and more (plan on replacing
    //  current form of the NonManifoldFaceDescriptor)...
    //
    int GetFaceSize() const {
        return (int) _data.faceSizes[0];
    }
    int GetFaceVertexIndices(int vtxIndices[]) const {
        int N = GetFaceSize();
        //  Vertices of face are guaranteed to be first control vertices:
        std::memcpy(vtxIndices, &_controlVertices[0], N * sizeof(int));
        return N;
    }

    int GetNumControlVertices() const {
        return (int) _controlVertices.size();
    }
    int GetControlVertexIndices(int vtxIndices[]) const {
        int N = GetNumControlVertices();
        std::memcpy(vtxIndices, &_controlVertices[0], N * sizeof(int));
        return N;
    }

    bool HasFVarIndices() const {
        return (_data.faceFVarValues.size() > 0);
    }
    int GetFaceFVarValueIndices(int fvarIndices[]) const {
        int N = GetFaceSize();
        std::memcpy(fvarIndices, &_data.faceFVarValues[0], N * sizeof(int));
        return N;
    }
    int GetControlFVarValueIndices(int fvarIndices[]) const;

    //
    //  Topology for NonManifoldFaceDescriptors cannot be shared (due to the
    //  lack of consistent orientation) so topology Keys are empty:
    //
    TopologyCache::Key ComputeTopologyKey() const {
        return TopologyCache::Key();
    }

    //
    //  For future use -- failure is temporary, should always succeed in
    //  future, but a rewrite of the General descriptor is necessary...
    //
    bool GetManifoldSubset(ManifoldFaceDescriptor &) const { return false; }
};

} // end namespace internal
} // end namespace Bfr

} // end namespace OPENSUBDIV_VERSION
using namespace OPENSUBDIV_VERSION;

} // end namespace OpenSubdiv

#endif /* OPENSUBDIV3_BFR_FACE_BUILDERS_H */
