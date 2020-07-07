#pragma once

namespace geometrycentral {
namespace combinatorial_map {

template <size_t D>
CombinatorialMap<D>::CombinatorialMap() {}

// Methods for getting number of mesh elements
template <size_t D>
inline size_t CombinatorialMap<D>::nDarts() const {
  return nDartsCount;
}

template <size_t D>
inline size_t CombinatorialMap<D>::nVertices() const {
  return nVerticesCount;
}

// Capacities
template <size_t D>
inline size_t CombinatorialMap<D>::nDartsCapacity() const {
  return nDartsCapacityCount;
}

// Connectivity
template <size_t D>
inline size_t CombinatorialMap<D>::dartPartner(size_t iD, size_t dim) const {
  return dartMap[dim][iD];
}

// template <size_t D>
// inline size_t CombinatorialMap<D>::heNextIncomingNeighbor(size_t iD)  const {
//   return usesImplicitTwin() ? heTwinImplicit(heNextArr[iD]) : heVertInNextArr[iD];
// }

// template <size_t D>
// inline size_t CombinatorialMap<D>::heNextOutgoingNeighbor(size_t iD) const {
//   return usesImplicitTwin() ? heNextArr[heTwinImplicit(iD)] : heVertOutNextArr[iD];
// }

template <size_t D>
Dart<D> CombinatorialMap<D>::getNewDart(bool isInterior) {

  // The boring case, when no resize is needed
  if (nDartsFillCount < nDartsCapacityCount) {
    // No work needed
  }
  // The intesting case, where vectors resize
  else {
    size_t newDartCapacity = std::max(nDartsCapacityCount * 2, (size_t)1);

    // Resize internal arrays
    for (size_t iD = 0; iD < D; ++iD) {
      dartMap[iD].resize(newDartCapacity);
    }
    dVertexArr.resize(newDartCapacity);

    nDartsCapacityCount = newDartCapacity;

    // Invoke relevant callback functions
    for (auto& f : dartExpandCallbackList) {
      f(newDartCapacity);
    }
  }

  nDartsFillCount++;
  nDartsCount++;

  modificationTick++;
  return Dart<D>(this, nDartsFillCount - 1);
}

template <size_t D>
inline bool CombinatorialMap<D>::vertexIsDead(size_t iV) const {
  return vDartArr[iV] == INVALID_IND;
}

template <size_t D>
inline bool CombinatorialMap<D>::dartIsDead(size_t iD) const {
  return dartMap[0][iD] == INVALID_IND;
}

// Methods for iterating over mesh elements w/ range-based for loops ===========

template <size_t D>
inline VertexSet<D> CombinatorialMap<D>::vertices() {
  return VertexSet<D>(this, 0, nVerticesFillCount);
}

template <size_t D>
inline DartSet<D> CombinatorialMap<D>::darts() {
  return DartSet<D>(this, 0, nDartsFillCount);
}

// Methods for accessing elements by index =====================================
// Note that these are only valid when the mesh is compressed.

template <size_t D>
inline Dart<D> CombinatorialMap<D>::dart(size_t index) {
  return Dart<D>(this, index);
}

template <size_t D>
VertexData<D, size_t> CombinatorialMap<D>::getVertexIndices() {

  VertexData<D, size_t> indices(*this);
  size_t i = 0;
  for (Vertex<D> v : vertices()) {
    indices[v] = i;
    i++;
  }
  return indices;
}

template <size_t D>
DartData<D, size_t> CombinatorialMap<D>::getDartIndices() {
  DartData<D, size_t> indices(*this);
  size_t i = 0;
  for (Dart<D> dart : darts()) {
    indices[dart] = i;
    i++;
  }
  return indices;
}

template <size_t D>
template <size_t E>
std::vector<std::vector<size_t>> CombinatorialMap<D>::getCellVertexList() {
  std::vector<std::vector<size_t>> cellVertexList;
  // VertexData<D, size_t> vIdx = getVertexIndices();

  DartData<D, char> visited(*this, false);
  DartData<D, char> onStack(*this, false);

  for (Dart<D> d : darts()) {
    if (visited[d]) continue;

    std::vector<size_t> cellVertices;
    std::deque<Dart<D>> dartsToVisit;
    dartsToVisit.push_back(d);
    onStack[d] = true;

    while (!dartsToVisit.empty()) {
      Dart<D> curr = dartsToVisit.back();
      dartsToVisit.pop_back();

      size_t currIdx = curr.vertex().getIndex();
      if (std::find(cellVertices.begin(), cellVertices.end(), currIdx) == cellVertices.end()) {
        cellVertices.push_back(currIdx);
      }

      visited[curr] = true;

      // You need to go in descending order to orient tets properly
      for (size_t iD = D; iD > 0; --iD) {
        if (iD != E) {
          Dart<D> neighbor = curr.partner(iD - 1);
          if (!visited[neighbor] && !onStack[neighbor]) {
            dartsToVisit.push_back(neighbor);
            onStack[neighbor] = true;
          }
        }
      }
    }
    cellVertexList.push_back(cellVertices);
  }
  return cellVertexList;
}

// Misc utility methods =====================================

template <size_t D>
inline bool CombinatorialMap<D>::isCompressed() const {
  return isCompressedFlag;
}

template <size_t D>
CombinatorialMap<D>::~CombinatorialMap() {
  for (auto& f : meshDeleteCallbackList) {
    f();
  }
}

template <size_t D>
std::unique_ptr<CombinatorialMap<D>> CombinatorialMap<D>::copy() const {
  return copyToCombinatorialMap();
}

template <size_t D>
std::unique_ptr<CombinatorialMap<D>> CombinatorialMap<D>::copyToCombinatorialMap() const {
  CombinatorialMap<D>* newMesh = new CombinatorialMap<D>();
  copyInternalFields(*newMesh);
  return std::unique_ptr<CombinatorialMap<D>>(newMesh);
}

template <size_t D>
void CombinatorialMap<D>::copyInternalFields(CombinatorialMap<D>& target) const {
  // == Copy _all_ the fields!

  // Raw data buffers (underlying std::vectors duplicate storage automatically)
  // TODO: does this still do a deep copy now that this is an std::array?
  target.dartMap = dartMap;

  // counts and flags
  target.nDartsCount = nDartsCount;
  target.nDartsCapacityCount = nDartsCapacityCount;

  target.isCompressedFlag = isCompressedFlag;

  // Note: _don't_ copy callbacks lists! New mesh has new callbacks
}


// Builds a halfedge mesh
template <>
CombinatorialMap<2>::CombinatorialMap(const std::vector<std::vector<size_t>>& polygons) {
  surface::ManifoldSurfaceMesh mesh(polygons);

  nVerticesCount = mesh.nVertices();
  nDartsCount = mesh.nHalfedges();
  nVerticesCapacityCount = nVerticesCount;
  nDartsCapacityCount = nDartsCount;
  nVerticesFillCount = nVerticesCount;
  nDartsFillCount = nDartsCount;

  // TODO: copy over arrays?
  surface::VertexData<size_t> vIdx = mesh.getVertexIndices();
  surface::HalfedgeData<size_t> hIdx = mesh.getHalfedgeIndices();
  dartMap[0].reserve(mesh.nHalfedges());
  dartMap[1].reserve(mesh.nHalfedges());
  dVertexArr.reserve(mesh.nHalfedges());
  vDartArr.reserve(mesh.nVertices());
  for (surface::Halfedge he : mesh.halfedges()) {
    dartMap[0].push_back(hIdx[he.next()]);
    dartMap[1].push_back(hIdx[he.twin()]);
    dVertexArr.push_back(vIdx[he.vertex()]);
  }
  for (surface::Vertex v : mesh.vertices()) {
    vDartArr.push_back(hIdx[v.halfedge()]);
  }
}

// Builds a tet mesh
template <>
CombinatorialMap<3>::CombinatorialMap(const std::vector<std::vector<size_t>>& tets) {
  nVerticesCount = 0;
  for (const std::vector<size_t>& tet : tets) {
    GC_SAFETY_ASSERT(tet.size() == 4, "CombinatorialMap<3> can only construct tet meshes from a list of cell vertices");
    for (size_t i : tet) {
      nVerticesCount = std::max(nVerticesCount, i);
    }
  }
  nVerticesCount++; // 0-based means count is max + 1

  vDartArr = std::vector<size_t>(nVerticesCount, INVALID_IND);

  std::map<std::array<size_t, 3>, size_t> createdDarts;

  auto shift = [&](std::array<size_t, 3> key) -> std::array<size_t, 3> { return {key[1], key[2], key[0]}; };

  auto createdDartLookup = [&](std::array<size_t, 3> key) -> size_t {
    auto keyIter = createdDarts.find(key);
    if (keyIter != createdDarts.end()) {
      return keyIter->second;
    }
    keyIter = createdDarts.find(shift(key));
    if (keyIter != createdDarts.end()) {
      return dartMap[0][dartMap[0][keyIter->second]];
    }
    keyIter = createdDarts.find(shift(shift(key)));
    if (keyIter != createdDarts.end()) {
      return dartMap[0][keyIter->second];
    }
    createdDarts[key] = INVALID_IND;
    return INVALID_IND;
  };

  // === Walk the tets, creating darts. Hook up dartMap[0] and dartMap[1] pointers (halfedges on tet surfaces), but
  // don't hook up dartMap[2] yet (gluing tets together).

  for (size_t iTet = 0; iTet < tets.size(); iTet++) {
    const std::vector<size_t>& tet = tets[iTet];

    // The oriented faces of tet {0, 1, 2, 3} are given by {{0, 1, 2}, {0, 2, 3}, {1, 3, 2}, {0, 3, 1}}
    // We index the tet's halfedges as 0 1 2, 3 4 5, 6 7 8, 9 10 11
    // The next array is 1 2 0, 4 5 3, 7 8 6, 10 11 9
    // The twin array is 11 8 3, 2 7 9, 10 4 1, 5 6 0
    const std::array<std::array<size_t, 3>, 4> tetFaceIndices{
        std::array<size_t, 3>{0, 1, 2}, std::array<size_t, 3>{0, 2, 3}, std::array<size_t, 3>{1, 3, 2},
        std::array<size_t, 3>{0, 3, 1}};
    const std::array<std::array<size_t, 3>, 4> tetFaces{
        std::array<size_t, 3>{tet[0], tet[1], tet[2]}, std::array<size_t, 3>{tet[0], tet[2], tet[3]},
        std::array<size_t, 3>{tet[1], tet[3], tet[2]}, std::array<size_t, 3>{tet[0], tet[3], tet[1]}};
    const std::array<size_t, 12> next{1, 2, 0, 4, 5, 3, 7, 8, 6, 10, 11, 9};
    const std::array<size_t, 12> twin{11, 8, 3, 2, 7, 9, 10, 4, 1, 5, 6, 0};

    std::array<size_t, 12> createdDarts;
    for (size_t iDart = 0; iDart < 12; ++iDart) {
      size_t newDart = getNewDart(false).getIndex();
      createdDarts[iDart] = newDart;

      size_t iV = tetFaces[iDart / 3][iDart % 3];
      dVertexArr[newDart] = iV;
      vDartArr[iV] = newDart;
    }

    for (size_t iDart = 0; iDart < 12; ++iDart) {
      dartMap[0][createdDarts[iDart]] = createdDarts[next[iDart]];
      dartMap[1][createdDarts[iDart]] = createdDarts[twin[iDart]];
    }

    for (size_t iF = 0; iF < 4; ++iF) {
      const std::array<size_t, 3> face = tetFaces[iF];
      size_t twinFaceDart = createdDartLookup(face);
      if (twinFaceDart == INVALID_IND) {
        // if the opposite face has not been created, set the appropriate pointers to empty
        for (size_t iDart : tetFaceIndices[iF]) {
          dartMap[2][createdDarts[iDart]] = INVALID_IND;
        }
      } else {
        // if the opposite face has already created, hook up the appropriate pointers
        dartMap[2][createdDarts[tetFaceIndices[iF][0]]] = twinFaceDart;
        dartMap[2][createdDarts[tetFaceIndices[iF][2]]] = dartMap[0][twinFaceDart];
        dartMap[2][createdDarts[tetFaceIndices[iF][1]]] = dartMap[0][dartMap[0][twinFaceDart]];
      }
    }
  }

  // TODO: do something about boundary

  nVerticesCapacityCount = nVerticesCount;
  nDartsCapacityCount = nDartsCount;
  nVerticesFillCount = nVerticesCount;
  nDartsFillCount = nDartsCount;
} // namespace combinatorial_map

} // namespace combinatorial_map
} // namespace geometrycentral
