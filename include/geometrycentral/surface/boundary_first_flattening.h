#pragma once

#include "geometrycentral/numerical/linear_algebra_utilities.h"
#include "geometrycentral/numerical/linear_solvers.h"
#include "geometrycentral/surface/intrinsic_geometry_interface.h"
#include "geometrycentral/surface/manifold_surface_mesh.h"

namespace geometrycentral {
namespace surface {

VertexData<Vector2> parameterizeBFF(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo);
VertexData<Vector2> parameterizeBFFfromScaleFactors(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo,
                                                    const VertexData<double>& boundaryScaleFactors);
VertexData<Vector2> parameterizeBFFfromExteriorAngles(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geo,
                                                      const VertexData<double>& exteriorAngles);

class BFF {
public:
  BFF(ManifoldSurfaceMesh& mesh_, IntrinsicGeometryInterface& geo_);
  BFF(ManifoldSurfaceMesh& mesh_, IntrinsicGeometryInterface& geo_, const std::vector<Edge>& cut_);

  VertexData<Vector2> flatten();
  VertexData<Vector2> flattenFromScaleFactors(const VertexData<double>& uBdy);
  VertexData<Vector2> flattenFromExteriorAngles(const VertexData<double>& kBdy);
  VertexData<Vector2> flattenFromBoth(const Vector<double>& uBdy, const Vector<double>& kBdy);

  CornerData<Vector2> flattenWithCuts();
  CornerData<Vector2> flattenWithCutsFromScaleFactors(const VertexData<double>& uBdy);
  CornerData<Vector2> flattenWithCutsFromScaleFactors(const CornerData<double>& uBdy);
  CornerData<Vector2> flattenWithCutsFromExteriorAngles(const CornerData<double>& kBdy);
  CornerData<Vector2> flattenWithCutsFromBoth(const Vector<double>& uBdy, const Vector<double>& kBdy);

  Vector<double> dirichletToNeumann(const Vector<double>& uBdy);
  Vector<double> neumannToDirichlet(const Vector<double>& kBdy);

  std::array<Vector<double>, 2> computeBoundaryPositions(const Vector<double>& uBdy, const Vector<double>& kBdy);


protected:
  ManifoldSurfaceMesh& mesh;
  IntrinsicGeometryInterface& geo;
  void init();

  //=== Used if there are cuts
  std::vector<Edge> cut;
  CornerData<size_t> wIdx;
  CornerData<int> iIdx, bIdx;
  size_t nWedges, nInteriorWedges, nBoundaryWedges;

  SparseMatrix<double> L, Lii, Lib, Lbb;
  std::unique_ptr<PositiveDefiniteSolver<double>> Liisolver;
  std::unique_ptr<PositiveDefiniteSolver<double>> Lsolver;
  Vector<double> Omegai, Omegab;

  Vector<bool> isInterior;

  BlockDecompositionResult<double> Ldecomp;

  void ensureHaveLSolver();
};

SparseMatrix<double> constructWedgeCotanLaplacian(ManifoldSurfaceMesh& mesh, IntrinsicGeometryInterface& geom,
                                                  const CornerData<size_t>& wIdx, size_t nWedges);

// Index wedges on cut mesh. Uncut interior vertices come first, followed by uncut boundary vertices, followed by wedges
// on the cut
// Wedges corresponding to uncut interior vertices are called "interior wedges", and all other wedges (on the original
// boundary or along the cut) are called "boundary wedges"
CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut);
CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut, size_t* nWedges);
CornerData<size_t> indexWedges(ManifoldSurfaceMesh& mesh, const std::vector<Edge>& cut, size_t* nWedges,
                               size_t* nInteriorWedges, size_t* nBoundaryWedges);

} // namespace surface
} // namespace geometrycentral
