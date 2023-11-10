#pragma once

#include "geometrycentral/combinatorial-maps/combinatorial_map_element_types.h"
#include "geometrycentral/utilities/mesh_data.h"
#include "geometrycentral/utilities/utilities.h"

#include "geometrycentral/surface/manifold_surface_mesh.h"

#include <array>
#include <deque>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <set>
#include <vector>

// NOTE: ipp includes at bottom of file

namespace geometrycentral {
namespace combinatorial_map {

// Typedefs and forward declarations
template <size_t D, typename T>
using VertexData = surface::MeshData<Vertex<D>, T>;
template <size_t D, typename T>
using DartData = surface::MeshData<Dart<D>, T>;

// ==========================================================
// ================    Combinatorial Map   ==================
// ==========================================================

template <size_t D>
class CombinatorialMap {

public:
  CombinatorialMap(const std::vector<std::vector<size_t>>& cells);

  virtual ~CombinatorialMap();


  // Number of mesh elements of each type
  size_t nVertices() const;
  size_t nDarts() const;

  // Methods for range-based for loops
  // Example: for(Vertex v : mesh.vertices()) { ... }
  VertexSet<D> vertices();
  DartSet<D> darts();


  // Methods for accessing elements by index
  // only valid when the  mesh is compressed
  Dart<D> dart(size_t index);

  VertexData<D, size_t> getVertexIndices();
  DartData<D, size_t> getDartIndices();

  size_t nConnectedComponents(); // compute number of connected components [O(n)]
  // virtual bool isManifold(); // Combinatorial maps must be manifold
  // virtual bool isEdgeManifold();
  // virtual bool isOriented(); // Combinatorial maps must be oriented
  void printStatistics() const; // print info about element counts to std::cout

  // std::vector<std::vector<size_t>> getFaceVertexList();

  template <size_t E>
  std::vector<std::vector<size_t>> getCellVertexList();

  std::unique_ptr<CombinatorialMap> copy() const;
  virtual std::unique_ptr<CombinatorialMap> copyToCombinatorialMap() const;
  // std::unique_ptr<ManifoldCombinatorialMap> toManifoldMesh();

  // Compress the mesh
  bool isCompressed() const;
  void compress();

  // == Mutation routines

  // Flip an edge. Edge is rotated clockwise. Return true if the edge was actually flipped (one can only flip
  // manifold, interior, triangular edges which are not incident on degree-1 vertices). Does _not_ create any new
  // elements, or cause the mesh to become decompressed.
  bool flip(Dart<D> d);

  // == Callbacks that will be invoked on mutation to keep containers/iterators/etc valid.

  // Expansion callbacks
  // Argument is the new size of the element list. Elements up to this index may now be used (but _might_ not be
  // in use immediately).
  std::list<std::function<void(size_t)>> vertexExpandCallbackList;
  std::list<std::function<void(size_t)>> dartExpandCallbackList;

  // Compression callbacks
  // Argument is a permutation to a apply, such that d_new[i] = d_old[p[i]]. THe length of the permutation is hte size
  // of the new index space. Any elements with p[i] == INVALID_IND are unused in the new index space.
  std::list<std::function<void(const std::vector<size_t>&)>> vertexPermuteCallbackList;
  std::list<std::function<void(const std::vector<size_t>&)>> dartPermuteCallbackList;

  // Mesh delete callbacks
  // (this unfortunately seems to be necessary; objects which have registered their callbacks above
  // need to know not to try to de-register them if the mesh has been deleted)
  std::list<std::function<void()>> meshDeleteCallbackList;

  // Check capacity. Needed when implementing expandable containers for mutable meshes to ensure the contain can
  // hold a sufficient number of elements before the next resize event.
  size_t nDartsCapacity() const;

  // == Debugging, etc

  // Performs a sanity checks on dart structure; throws on fail
  void validateConnectivity();

protected:
  // Constructor used by subclasses
  CombinatorialMap();

  // Construct directly from internal arrays
  CombinatorialMap(const std::array<std::vector<size_t>, D>& dartMap);

  // = Core arrays which hold the connectivity
  // Note: it should always be true that heFace.size() == nDartsCapacityCount, but any elements after
  // nDartsFillCount will be valid indices (in the std::vector sense), but contain uninitialized data. Similarly,
  // any std::vector<> indices corresponding to deleted elements will hold meaningless values.
  std::array<std::vector<size_t>, D> dartMap;

  size_t dartPartner(size_t iD, size_t dim) const;
  std::vector<size_t> dVertexArr; // dart.vertex()
  std::vector<size_t> vDartArr;   // vertex.dart()

  // Auxilliary arrays which cache other useful information

  // Track element counts (can't rely on rawVertices.size() after deletions have made the list sparse). These are the
  // actual number of valid elements, not the size of the buffer that holds them.
  size_t nVerticesCount = 0;
  size_t nDartsCount = 0;

  // == Track the capacity and fill size of our buffers.
  // These give the capacity of the currently allocated buffer.
  // Note that this is _not_ defined to be std::vector::capacity(), it's the largest size such that arr[i] is legal (aka
  // arr.size()).
  size_t nVerticesCapacityCount = 0; // will always be even if implicit twin
  size_t nDartsCapacityCount = 0;    // will always be even if implicit twin

  // These give the number of filled elements in the currently allocated buffer. This will also be the maximal index of
  // any element (except the weirdness of boundary loop faces). As elements get marked dead, nVerticesCount decreases
  // but nVertexFillCount does not (etc), so it denotes the end of the region in the buffer where elements have been
  // stored.
  size_t nVerticesFillCount = 0;
  size_t nDartsFillCount = 0; // must always be even if implicit twin

  // The mesh is _compressed_ if all of the index spaces are dense. E.g. if thare are |V| vertices, then the vertices
  // are densely indexed from 0 ... |V|-1 (and likewise for the other elements). The mesh can become not-compressed as
  // deletions mark elements with tombstones--this is how we support constant time deletion.
  // Call compress() to re-index and return to usual dense indexing.
  bool isCompressedFlag = true;

  uint64_t modificationTick = 1; // Increment every time the mesh is mutated in any way. Used to track staleness.

  // Hide copy and move constructors, we don't wanna mess with that
  CombinatorialMap(const CombinatorialMap& other) = delete;
  CombinatorialMap& operator=(const CombinatorialMap& other) = delete;
  CombinatorialMap(CombinatorialMap&& other) = delete;
  CombinatorialMap& operator=(CombinatorialMap&& other) = delete;

  // Used to resize the halfedge mesh. Expands and shifts vectors as necessary.
  Dart<D> getNewDart(bool isInterior);

  // Detect dead elements
  bool vertexIsDead(size_t iV) const;
  bool dartIsDead(size_t iD) const;

  // Deletes leave tombstones, which can be cleaned up with compress().
  // Note that these routines merely mark the element as dead. The caller should hook up connectivity to exclude these
  // elements before invoking.
  void deleteElement(Dart<D> d); // can't use for implicit twin

  // Compression helpers
  void compressDarts();

  // = =Helpers for mutation methods and similar things

  void initializeDartNeighbors();
  void copyInternalFields(CombinatorialMap& target) const;

  // replace values of i in arr with oldToNew[i] (skipping INVALID_IND)
  void updateValues(std::vector<size_t>& arr, const std::vector<size_t>& oldToNew);


  // Elements need direct access in to members to traverse
  friend class Vertex<D>;
  friend struct VertexRangeF<D>;
  friend class Dart<D>;
  friend struct DartRangeF<D>;
};

} // namespace combinatorial_map
} // namespace geometrycentral

// clang-format off
// preserve ordering
// #include "geometrycentral/combinatorial-maps/dart_logic_templates.ipp"
#include "geometrycentral/combinatorial-maps/combinatorial_map.ipp"
#include "geometrycentral/combinatorial-maps/combinatorial_map_element_types.ipp"
// clang-format on
