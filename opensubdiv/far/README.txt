

Changes in this branch represent a very rough prototype that organizes the limit
surface patches into trees associated with each base face of a mesh, rather than
assembled in PatchTables.

Additions include two groups of classes new to Far:

    - PatchTree, PatchTreeFactory and PatchTreeCache
    - BasePatch and BasePatchFactory

along with a new far/tutorial_10 that illustrates their use.


The Far::PatchTree is a low-level class that represents the limit surface for a
face of a particular topological configuration.  They are defined in terms of the
minimal number of control points required and are not tied to any particular
topological representation.  PatchTrees can be constructed independently or shared
for all faces in a mesh that identified with similar topology.

The Far::BasePatch is the public class that clients are intended to use to
construct and evaluate the limit surface associated with a face of their mesh.
The BasePatch is implemented with one or more PatchTrees (possibly more in the case
of face-varying data or N-sided faces) and identifies the subset of vertices
that constitute the control points for the PatchTree.  Clients are expected to
describe the topology for each face in their BaseFaceFactory, and that topolgoical
description is used to identify or construct the required PatchTree (using the
optional PatchTreeCache or PatchTreeFactory).

The new far/tutorial_10 is a simple tessellator that requires a .obj file on the
command line, constructs a Far::BasePatch for each face, tessellates each face
uniformly, and prints the resulting tessellation to standard output in .obj format.


The primary purpose of this branch was to illustrate the construction of a
PatchTree using the Far::PatchBuilder class, which supports all types of both quad
and triangular patches with little effort.  There are other ways to organize the
data in a PatchTree -- the organization implemented is just one possibility and not
necessarily preferred.  There are other extensions such as rotations or patches at
interior nodes of the tree that also warrant consideration but are not addressed
here.  These do not significanly impact the usage of the PatchBuilder in
constructing the tree.

The secondary purpose in building the rest of the classes around the PatchTree is
to raise issues for how this functionality will eventually be presented through
the public interface and/or assembled into efficient aggregates targeted to
specific platforms.  Many classes here are simple place holders or have
implementations that are temporary -- most notably the topology description for a
face, the associated hashing key computed from it, and cache which uses that key.


