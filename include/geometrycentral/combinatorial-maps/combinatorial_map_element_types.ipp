#pragma once

// Implementations for combinatorial_map_mesh_types.h

// Make the element types hashable (this _should_ be doable for just the parent class, but I couldn't sort out how)
namespace std {
template <size_t D>
struct hash<geometrycentral::combinatorial_map::Dart<D>> {
  std::size_t operator()(const geometrycentral::combinatorial_map::Dart<D>& e) const {
    return std::hash<size_t>{}(e.getIndex());
  }
};
} // namespace std

namespace geometrycentral {
namespace combinatorial_map {

// ==========================================================
// ================        Dart        ==================
// ==========================================================

// Constructors
template <size_t D>
inline Dart<D>::Dart() {}

template <size_t D>
inline Dart<D>::Dart(CombinatorialMap<D>* mesh_, size_t ind_) : Element<Dart<D>, CombinatorialMap<D>>(mesh_, ind_) {}

// Navigators
template <size_t D>
inline Dart<D> Dart<D>::partner(size_t d) const {
  size_t partnerInd = this->mesh->dartPartner(this->ind, d);
  return (partnerInd == INVALID_IND) ? *this : Dart<D>(this->mesh, this->mesh->dartPartner(this->ind, d));
};

template <size_t D>
inline Vertex<D> Dart<D>::vertex() const {
  return Vertex<D>(this->mesh, this->mesh->dVertexArr[this->ind]);
};

template <size_t D>
inline bool Dart<D>::isDead() const {
  return this->mesh->dartIsDead(this->ind);
}

// Range iterators
template <size_t D>
inline bool VertexRangeF<D>::elementOkay(const CombinatorialMap<D>& mesh, size_t ind) {
  return !mesh.vertexIsDead(ind);
}

// Range iterators
template <size_t D>
inline bool DartRangeF<D>::elementOkay(const CombinatorialMap<D>& mesh, size_t ind) {
  return !mesh.dartIsDead(ind);
}

// ==========================================================
// ================        Vertex        ==================
// ==========================================================

// Constructors
template <size_t D>
inline Vertex<D>::Vertex() {}

template <size_t D>
inline Vertex<D>::Vertex(CombinatorialMap<D>* mesh_, size_t ind_)
    : Element<Vertex<D>, CombinatorialMap<D>>(mesh_, ind_) {}

// Navigators
template <size_t D>
inline Dart<D> Vertex<D>::dart() const {
  return Dart<D>(this->mesh, this->mesh->vDartArr[this->ind]);
};

template <size_t D>
inline bool Vertex<D>::isDead() const {
  return this->mesh->dartIsDead(this->ind);
}


} // namespace combinatorial_map


template <>
struct ElementSetType<combinatorial_map::Vertex<2>> {
  typedef combinatorial_map::VertexSet<2> type;
};
template <>
struct ElementSetType<combinatorial_map::Vertex<3>> {
  typedef combinatorial_map::VertexSet<3> type;
};
template <>
struct ElementSetType<combinatorial_map::Dart<2>> {
  typedef combinatorial_map::DartSet<2> type;
};
template <>
struct ElementSetType<combinatorial_map::Dart<3>> {
  typedef combinatorial_map::DartSet<3> type;
};

template <>
inline size_t nElements<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->nVertices();
}
template <>
inline size_t nElements<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->nVertices();
}
template <>
inline size_t nElements<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->nDarts();
}
template <>
inline size_t nElements<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->nDarts();
}

template <>
inline size_t elementCapacity<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->nVertices();
}
template <>
inline size_t elementCapacity<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->nVertices();
}
template <>
inline size_t elementCapacity<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->nDarts();
}
template <>
inline size_t elementCapacity<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->nDarts();
}

template <>
inline size_t dataIndexOfElement<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh,
                                                               combinatorial_map::Vertex<2> e) {
  return e.getIndex();
}
template <>
inline size_t dataIndexOfElement<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh,
                                                               combinatorial_map::Vertex<3> e) {
  return e.getIndex();
}
template <>
inline size_t dataIndexOfElement<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh,
                                                             combinatorial_map::Dart<2> e) {
  return e.getIndex();
}
template <>
inline size_t dataIndexOfElement<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh,
                                                             combinatorial_map::Dart<3> e) {
  return e.getIndex();
}


template <>
inline combinatorial_map::VertexSet<2>
iterateElements<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->vertices();
}
template <>
inline combinatorial_map::VertexSet<3>
iterateElements<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->vertices();
}
template <>
inline combinatorial_map::DartSet<2>
iterateElements<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->darts();
}
template <>
inline combinatorial_map::DartSet<3>
iterateElements<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->darts();
}

template <>
inline std::list<std::function<void(size_t)>>&
getExpandCallbackList<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->vertexExpandCallbackList;
}
template <>
inline std::list<std::function<void(size_t)>>&
getExpandCallbackList<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->vertexExpandCallbackList;
}
template <>
inline std::list<std::function<void(size_t)>>&
getExpandCallbackList<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->dartExpandCallbackList;
}
template <>
inline std::list<std::function<void(size_t)>>&
getExpandCallbackList<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->dartExpandCallbackList;
}

template <>
inline std::list<std::function<void(const std::vector<size_t>&)>>&
getPermuteCallbackList<combinatorial_map::Vertex<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->vertexPermuteCallbackList;
}
template <>
inline std::list<std::function<void(const std::vector<size_t>&)>>&
getPermuteCallbackList<combinatorial_map::Vertex<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->vertexPermuteCallbackList;
}
template <>
inline std::list<std::function<void(const std::vector<size_t>&)>>&
getPermuteCallbackList<combinatorial_map::Dart<2>>(combinatorial_map::CombinatorialMap<2>* mesh) {
  return mesh->dartPermuteCallbackList;
}
template <>
inline std::list<std::function<void(const std::vector<size_t>&)>>&
getPermuteCallbackList<combinatorial_map::Dart<3>>(combinatorial_map::CombinatorialMap<3>* mesh) {
  return mesh->dartPermuteCallbackList;
}

// template <size_t D>
// inline size_t elementCapacity<combinatorial_map::Vertex<D>>(combinatorial_map::CombinatorialMap<D>* mesh) {
//   return mesh->nVerticesCapacity();
// }

// template <size_t D>
// inline size_t elementCapacity<combinatorial_map::Dart<D>>(combinatorial_map::CombinatorialMap<D>* mesh) {
//   return mesh->nDartsCapacity();
// }

} // namespace geometrycentral
