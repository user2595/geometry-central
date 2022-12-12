#include "geometrycentral/surface/boundary_first_flattening.h"

namespace geometrycentral {
namespace surface {

VertexData<Vector2> parameterizeBFF(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo) {
  BFF bff(mesh, geo);
  return bff.flatten();
}

VertexData<Vector2> parameterizeBFFfromScaleFactors(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo,
                                                    const VertexData<double>& boundaryScaleFactors) {
  BFF bff(mesh, geo);
  return bff.flattenFromScaleFactors(boundaryScaleFactors);
}

VertexData<Vector2> parameterizeBFFfromExteriorAngles(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo,
                                                      const VertexData<double>& exteriorAngles) {
  BFF bff(mesh, geo);
  return bff.flattenFromExteriorAngles(exteriorAngles);
}

BFF::BFF(ManifoldSurfaceMesh& mesh_, IntrinsicGeometryInterface& geo_) : mesh(mesh_), geo(geo_) { init(); }

BFF::BFF(ManifoldSurfaceMesh& mesh_, IntrinsicGeometryInterface& geo_, const std::vector<Edge>& cut_)
    : mesh(mesh_), geo(geo_), cut(cut_) {
  init();
}

void BFF::init() {

  wIdx = indexWedges(mesh, cut, &nWedges, &nInteriorWedges, &nBoundaryWedges);

  // GC_SAFETY_ASSERT(mesh.eulerCharacteristic() == 2 && mesh.nBoundaryLoops() == 1,
  //                  "Input to BFF must be a topological disk");

  iIdx = CornerData<int>(mesh, -1);
  bIdx = CornerData<int>(mesh, -1);

  isInterior = Vector<bool>(nWedges);
  for (Corner c : mesh.corners()) {
    size_t iW = wIdx[c];
    if (iW < nInteriorWedges) {
      iIdx[c] = iW;
      isInterior(iW) = true;
    } else {
      bIdx[c] = iW - nInteriorWedges;
      isInterior(iW) = false;
    }
  }

  L = constructWedgeCotanLaplacian(mesh, geo, wIdx, nWedges);
  shiftDiagonal(L, 1e-8);

  Ldecomp = blockDecomposeSquare(L, isInterior);

  Lii = Ldecomp.AA;
  Lib = Ldecomp.AB;
  Lbb = Ldecomp.BB;

  // TODO: extract this factorization from a full factorization of L
  Liisolver.reset(new PositiveDefiniteSolver<double>(Lii));

  Omegai = Vector<double>::Constant(nInteriorWedges, 2 * M_PI);
  Omegab = Vector<double>::Constant(nBoundaryWedges, M_PI);
  geo.requireCornerAngles();
  for (Corner c : mesh.corners()) {
    size_t iW = wIdx[c];
    if (isInterior(iW)) {
      Omegai(iIdx[c]) -= geo.cornerAngles[c];
    } else {
      Omegab(bIdx[c]) -= geo.cornerAngles[c];
    }
  }
  geo.unrequireCornerAngles();
}

CornerData<Vector2> BFF::flattenWithCuts() {
  // Impose 0 scaling on boundary
  Vector<double> uBdy = Vector<double>::Zero(nBoundaryWedges);

  // Compute complementary data
  Vector<double> kBdy = dirichletToNeumann(uBdy);

  // Flatten
  return flattenWithCutsFromBoth(uBdy, kBdy);
}

CornerData<Vector2> BFF::flattenWithCutsFromScaleFactors(const VertexData<double>& uData) {
  // Extract boundary values
  Vector<double> uBdy(nBoundaryWedges);
  for (Corner c : mesh.corners()) {
    if (!isInterior(bIdx[c])) uBdy(bIdx[c]) = uData[c.vertex()];
  }

  // Compute complementary data
  Vector<double> kBdy = dirichletToNeumann(uBdy);

  // Flatten
  return flattenWithCutsFromBoth(uBdy, kBdy);
}

CornerData<Vector2> BFF::flattenWithCutsFromScaleFactors(const CornerData<double>& uData) {
  // Extract boundary values
  Vector<double> uBdy(nBoundaryWedges);
  for (Corner c : mesh.corners()) {
    if (!isInterior(bIdx[c])) uBdy(bIdx[c]) = uData[c];
  }

  // Compute complementary data
  Vector<double> kBdy = dirichletToNeumann(uBdy);

  // Flatten
  return flattenWithCutsFromBoth(uBdy, kBdy);
}

CornerData<Vector2> BFF::flattenWithCutsFromExteriorAngles(const CornerData<double>& kData) {
  // Extract boundary values
  Vector<double> kBdy(nBoundaryWedges);
  for (Corner c : mesh.corners()) {
    if (bIdx[c] >= 0) kBdy(bIdx[c]) = kData[c];
  }

  GC_SAFETY_ASSERT(abs(kBdy.sum() - 2 * M_PI) < 1e-3,
                   "BFF error: target exterior angles must sum to 2 pi, but the input sums to " +
                       std::to_string(kBdy.sum()));

  // Compute complementary data
  Vector<double> uBdy = neumannToDirichlet(kBdy);

  // FlattenWithCuts
  return flattenWithCutsFromBoth(uBdy, kBdy);
}

CornerData<Vector2> BFF::flattenWithCutsFromBoth(const Vector<double>& uBdy, const Vector<double>& kBdy) {

  Vector<double> boundaryX, boundaryY;
  std::tie(boundaryX, boundaryY) = tuple_cat(computeBoundaryPositions(uBdy, kBdy));

  Vector<double> interiorX = Liisolver->solve(-Lib * boundaryX);
  Vector<double> interiorY = Liisolver->solve(-Lib * boundaryY);

  CornerData<Vector2> parm(mesh);
  for (Corner c : mesh.corners()) {
    if (isInterior(wIdx[c])) {
      size_t iW = iIdx[c];
      parm[c] = Vector2{interiorX(iW), interiorY(iW)};
    } else {
      size_t iW = bIdx[c];
      parm[c] = Vector2{boundaryX(iW), boundaryY(iW)};
    }
  }

  return parm;
}

VertexData<Vector2> BFF::flatten() {
  VertexData<double> u(mesh, 0);
  return flattenFromScaleFactors(u);
}

VertexData<Vector2> BFF::flattenFromScaleFactors(const VertexData<double>& uData) {
  // Extract boundary values
  Vector<double> uBdy, ignore;
  decomposeVector(Ldecomp, uData.toVector(), ignore, uBdy);

  // Compute complementary data
  Vector<double> kBdy = dirichletToNeumann(uBdy);

  // Flatten
  return flattenFromBoth(uBdy, kBdy);
}

VertexData<Vector2> BFF::flattenFromExteriorAngles(const VertexData<double>& kData) {
  // Extract boundary values
  Vector<double> kBdy, ignore;
  decomposeVector(Ldecomp, kData.toVector(), ignore, kBdy);

  GC_SAFETY_ASSERT(abs(kBdy.sum() - 2 * M_PI) < 1e-3,
                   "BFF error: target exterior angles must sum to 2 pi, but the input sums to " +
                       std::to_string(kBdy.sum()));

  // Compute complementary data
  Vector<double> uBdy = neumannToDirichlet(kBdy);

  // Flatten
  return flattenFromBoth(uBdy, kBdy);
}

VertexData<Vector2> BFF::flattenFromBoth(const Vector<double>& uBdy, const Vector<double>& kBdy) {
  GC_SAFETY_ASSERT(cut.empty(), "BFF can only generate VertexData textures if there are no cuts. To parameterize cut "
                                "meshes, use BFF::flattenWithCuts");

  Vector<double> boundaryX, boundaryY;
  std::tie(boundaryX, boundaryY) = tuple_cat(computeBoundaryPositions(uBdy, kBdy));

  Vector<double> interiorX = Liisolver->solve(-Lib * boundaryX);
  Vector<double> interiorY = Liisolver->solve(-Lib * boundaryY);

  VertexData<Vector2> parm(mesh);
  for (Vertex v : mesh.vertices()) {
    if (isInterior(wIdx[v.corner()])) {
      size_t iW = iIdx[v.corner()];
      parm[v] = Vector2{interiorX(iW), interiorY(iW)};
    } else {
      size_t iW = bIdx[v.corner()];
      parm[v] = Vector2{boundaryX(iW), boundaryY(iW)};
    }
  }

  return parm;
}

Vector<double> BFF::dirichletToNeumann(const Vector<double>& uBdy) {
  return Omegab - (Lib.transpose() * Liisolver->solve(Omegai - Lib * uBdy)) - Lbb * uBdy;
}

Vector<double> BFF::neumannToDirichlet(const Vector<double>& kBdy) {
  // Convert Neumann data to Dirichlet data by solving the Poisson equation and reading off values
  ensureHaveLSolver();
  Vector<double> rhs = reassembleVector(Ldecomp, Omegai, Vector<double>(Omegab - kBdy));
  Vector<double> fullSolution = -Lsolver->solve(rhs);
  Vector<double> uBdy, ignore;
  decomposeVector(Ldecomp, fullSolution, ignore, uBdy);
  double uMean = uBdy.mean(); // Ensure that u has mean 0
  for (int i = 0; i < uBdy.size(); i++) uBdy(i) -= uMean;
  return uBdy;
}

std::array<Vector<double>, 2> BFF::computeBoundaryPositions(const Vector<double>& uBdy, const Vector<double>& kBdy) {

  geo.requireEdgeLengths();

  double phi = 0;

  // Iterate over boundary like here:
  // https://github.com/GeometryCollective/boundary-first-flattening/blob/master/mesh/include/CutIterator.inl#L14
  // We assume that the boundary has one connected component

  EdgeData<bool> onCut;
  if (!cut.empty()) {
    onCut = EdgeData<bool>(mesh, false);
    for (Edge e : cut) onCut[e] = true;
  }

  auto onCutBoundary = [&](Halfedge he) -> bool {
    return he.edge().isBoundary() || (!cut.empty() && onCut[he.edge()]);
  };

  Halfedge firstHe = (!cut.empty()) ? cut[0].halfedge() : mesh.boundaryLoop(0).halfedge().twin();
  GC_SAFETY_ASSERT(firstHe.isInterior(), "???");
  GC_SAFETY_ASSERT(onCutBoundary(firstHe), "Not on cut???");

  auto nextAlongCut = [&](Halfedge curr) -> Halfedge {
    // if (cut.empty()) return curr.twin().next().twin();
    Halfedge he = curr.next();
    while (!onCutBoundary(he)) {
      he = he.twin().next(); // loop around one ring clockwise
      if (he == curr.next()) break;
    }
    if (!onCutBoundary(he)) {
      std::cout << "??? can't find cut?" << std::endl;
    }
    return he;
  };

  std::vector<Eigen::Triplet<double>> Ntriplets;
  Vector<double> targetLength(nBoundaryWedges);
  DenseMatrix<double> T(2, nBoundaryWedges);

  Halfedge curr = firstHe;
  do {
    int iW = bIdx[curr.corner()];
    GC_SAFETY_ASSERT(iW >= 0, "invalid boundary vertex index");

    targetLength(iW) =
        geo.edgeLengths[curr.edge()] * exp(0.5 * (uBdy(bIdx[curr.corner()]) + uBdy(bIdx[curr.next().corner()])));

    T(0, iW) = cos(phi);
    T(1, iW) = sin(phi);

    Ntriplets.emplace_back(iW, iW, geo.edgeLengths[curr.edge()]);

    phi += kBdy(bIdx[curr.next().corner()]);

    curr = nextAlongCut(curr);
  } while (curr != firstHe);

  SparseMatrix<double> Ninv(nBoundaryWedges, nBoundaryWedges);
  Ninv.setFromTriplets(std::begin(Ntriplets), std::end(Ntriplets));

  Vector<double> roundedLength =
      targetLength - Ninv * T.transpose() * (T * Ninv * T.transpose()).inverse() * T * targetLength;

  std::array<Vector<double>, 2> bdyPositions{Vector<double>(nBoundaryWedges), Vector<double>(nBoundaryWedges)};
  double x = 0, y = 0;
  curr = firstHe;
  do {
    size_t iW = bIdx[curr.corner()];

    bdyPositions[0](iW) = x;
    bdyPositions[1](iW) = y;

    x += roundedLength(iW) * T(0, iW);
    y += roundedLength(iW) * T(1, iW);

    curr = nextAlongCut(curr);
  } while (curr != firstHe);

  return bdyPositions;
}

void BFF::ensureHaveLSolver() {
  if (!Lsolver) {
    Lsolver.reset(new PositiveDefiniteSolver<double>(L));
  }
}

SparseMatrix<double> constructWedgeCotanLaplacian(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo,
                                                  const CornerData<size_t>& wIdx, size_t nWedges) {
  std::vector<Eigen::Triplet<double>> T;

  geo.requireHalfedgeCotanWeights();
  for (Halfedge ij : mesh.interiorHalfedges()) {
    size_t i = wIdx[ij.corner()];
    size_t j = wIdx[ij.next().corner()];
    double weight = geo.halfedgeCotanWeights[ij];

    T.emplace_back(i, i, weight);
    T.emplace_back(i, j, -weight);
    T.emplace_back(j, i, -weight);
    T.emplace_back(j, j, weight);
  }
  geo.unrequireHalfedgeCotanWeights();

  SparseMatrix<double> L(nWedges, nWedges);
  L.setFromTriplets(T.begin(), T.end());

  return L;
}

CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut) {
  size_t nWedges, nInteriorWedges, nBoundaryWedges;
  return indexWedges(mesh, cut, &nWedges, &nInteriorWedges, &nBoundaryWedges);
}

CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut, size_t* nWedges) {
  size_t nInteriorWedges, nBoundaryWedges;
  return indexWedges(mesh, cut, nWedges, &nInteriorWedges, &nBoundaryWedges);
}

// https://github.com/GeometryCollective/boundary-first-flattening/blob/master/project/src/Bff.cpp#L755
CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut, size_t* nWedges,
                               size_t* nInteriorWedges, size_t* nBoundaryWedges) {
  VertexData<bool> vertexOnCut(mesh, false);
  EdgeData<bool> edgeOnCut(mesh, false);
  for (Edge e : cut) {
    vertexOnCut[e.halfedge().tailVertex()] = true;
    vertexOnCut[e.halfedge().tipVertex()] = true;
    edgeOnCut[e] = true;
  }

  size_t iW = 0;
  CornerData<size_t> wedgeIndices(mesh);

  // Index true interior vertices
  for (Vertex v : mesh.vertices()) {
    if (!v.isBoundary() && !vertexOnCut[v]) {
      for (Corner c : v.adjacentCorners()) wedgeIndices[c] = iW;
      iW++;
    }
  }
  *nInteriorWedges = iW;

  // Index boundary (but uncut) vertices
  for (Vertex v : mesh.vertices()) {
    if (v.isBoundary() && !vertexOnCut[v]) {
      for (Corner c : v.adjacentCorners()) wedgeIndices[c] = iW;
      iW++;
    }
  }

  // Index vertices on cut
  for (Edge e : cut) {
    for (Halfedge he : e.adjacentHalfedges()) {
      Halfedge curr = he;
      do {
        curr = curr.twin().next();
        wedgeIndices[curr.corner()] = iW;
      } while (!edgeOnCut[curr.edge()]);
      iW++;
    }
  }

  *nWedges = iW;
  *nBoundaryWedges = *nWedges - *nInteriorWedges;

  return wedgeIndices;
}

} // namespace surface
} // namespace geometrycentral
