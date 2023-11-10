#pragma once

#include "geometrycentral/utilities/element.h"
#include "geometrycentral/utilities/element_iterators.h"
#include "geometrycentral/utilities/mesh_data.h"
#include "geometrycentral/utilities/utilities.h"

#include <cstddef>
#include <iostream>
#include <list>
#include <typeindex>
#include <unordered_set>

namespace geometrycentral {
namespace combinatorial_map {

// === Types and inline methods for the dart mesh pointer and datatypes
template <size_t D>
class CombinatorialMap;

template <size_t D>
class Vertex;
template <size_t D>
class Dart;


template <size_t D, size_t E>
struct DartOrbitNavigator;

// ==========================================================
// ================        Dart        ==================
// ==========================================================

template <size_t D>
class Dart : public Element<Dart<D>, CombinatorialMap<D>> {
public:
  // Constructors
  Dart();                                      // construct an empty (null) element
  Dart(CombinatorialMap<D>* mesh, size_t ind); // construct pointing to the i'th element of that type on a mesh.
  // Dart(const Dynamic Element<Dart>& e); // construct from a dynamic element of matching type

  // Navigators
  Vertex<D> vertex() const;
  Dart<D> partner(size_t d) const;

  bool isDead() const;
};

// ==========================================================
// ================        Vertex        ==================
// ==========================================================

template <size_t D>
class Vertex : public Element<Vertex<D>, CombinatorialMap<D>> {
public:
  // Constructors
  Vertex();                                      // construct an empty (null) element
  Vertex(CombinatorialMap<D>* mesh, size_t ind); // construct pointing to the i'th element of that type on a mesh.

  // Navigators
  Dart<D> dart() const;

  bool isDead() const;
};


// == Range iterators

// All vertices
template <size_t D>
struct VertexRangeF {
  static bool elementOkay(const CombinatorialMap<D>& mesh, size_t ind);
  typedef Vertex<D> Etype;
  typedef CombinatorialMap<D> ParentMeshT;
};
template <size_t D>
using VertexSet = RangeSetBase<VertexRangeF<D>>;

// All darts
template <size_t D>
struct DartRangeF {
  static bool elementOkay(const CombinatorialMap<D>& mesh, size_t ind);
  typedef Dart<D> Etype;
  typedef CombinatorialMap<D> ParentMeshT;
};

template <size_t D>
using DartSet = RangeSetBase<DartRangeF<D>>;
} // namespace combinatorial_map

// Declare specializations of the logic templates. This is important, because these need to be declared before any of
// the templates using them are instantiated.

// template<size_t D> inline size_t nElements<combinatorial_map::Dart<D>>(combinatorial_map::CombinatorialMap<D>* mesh);

// template<size_t D> inline size_t dataIndexOfElement<combinatorial_map::Dart<D>
// >(combinatorial_map::CombinatorialMap<D>* mesh, combinatorial_map::Dart<D> e         ); template<size_t D> struct
// ElementSetType<combinatorial_map::Dart<D>      >   { typedef combinatorial_map::DartSet<D>     type; };
// template<size_t D> inline combinatorial_map::DartSet<D>       iterateElements<combinatorial_map::Dart<D>
// >(combinatorial_map::CombinatorialMap<D>* mesh); template<size_t D> inline std::list<std::function<void(size_t)>>&
// getExpandCallbackList<combinatorial_map::Dart<D>    >(combinatorial_map::CombinatorialMap<D>* mesh); template<size_t
// D> inline std::list<std::function<void(const std::vector<size_t>&)>>&
// getPermuteCallbackList<combinatorial_map::Dart<D>     >(combinatorial_map::CombinatorialMap<D>* mesh);
// template<size_t D> inline std::string typeShortName<combinatorial_map::Dart<D>>();

} // namespace geometrycentral
