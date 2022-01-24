#include "geometrycentral/numerical/linear_solvers.h"

#include "geometrycentral/numerical/linear_algebra_utilities.h"

#ifdef GC_HAVE_SUITESPARSE
#include "geometrycentral/numerical/suitesparse_utilities.h"
#endif

namespace geometrycentral {

template <typename T>
struct PSDSolverInternals {
#ifdef GC_HAVE_SUITESPARSE
  CholmodContext context;
  cholmod_sparse* cMat = nullptr;
  cholmod_factor* factorization = nullptr;
#else
  Eigen::SimplicialLDLT<SparseMatrix<T>> solver;
#endif
};

template <typename T>
PositiveDefiniteSolver<T>::~PositiveDefiniteSolver() {
#ifdef GC_HAVE_SUITESPARSE
  if (internals->cMat != nullptr) {
    cholmod_l_free_sparse(&internals->cMat, internals->context);
    internals->cMat = nullptr;
  }
  if (internals->factorization != nullptr) {
    cholmod_l_free_factor(&internals->factorization, internals->context);
  }
#endif
}

template <typename T>
PositiveDefiniteSolver<T>::PositiveDefiniteSolver(SparseMatrix<T>& mat)
    : LinearSolver<T>(mat), internals(new PSDSolverInternals<T>()) {


  // Check some sanity
  if (this->nRows != this->nCols) {
    throw std::logic_error("Matrix must be square");
  }
  size_t N = this->nRows;
#ifndef GC_NLINALG_DEBUG
  checkFinite(mat);
  checkHermitian(mat);
#endif

  mat.makeCompressed();

  // Suitesparse version
#ifdef GC_HAVE_SUITESPARSE

  // Convert suitesparse format
  if (internals->cMat != nullptr) {
    cholmod_l_free_sparse(&internals->cMat, internals->context);
  }
  internals->cMat = toCholmod(mat, internals->context, SType::SYMMETRIC);

  // Factor
  internals->context.setSimplicial(); // must use simplicial for LDLt
  internals->context.setLDL();        // ensure we get an LDLt internals->factorization
  internals->factorization = cholmod_l_analyze(internals->cMat, internals->context);
  bool success = (bool)cholmod_l_factorize(internals->cMat, internals->factorization, internals->context);

  if (!success) {
    throw std::runtime_error("failure in cholmod_l_factorize");
  }
  if (internals->context.context.status == CHOLMOD_NOT_POSDEF) {
    throw std::runtime_error("matrix is not positive definite");
  }


  // Eigen version
#else
  internals->solver.compute(mat);
  if (internals->solver.info() != Eigen::Success) {
    std::cerr << "Solver internals->factorization error: " << internals->solver.info() << std::endl;
    throw std::invalid_argument("Solver internals->factorization failed");
  }
#endif
};

template <typename T>
Vector<T> PositiveDefiniteSolver<T>::solve(const Vector<T>& rhs) {
  Vector<T> out;
  solve(out, rhs);
  return out;
}

template <typename T>
void PositiveDefiniteSolver<T>::solve(Vector<T>& x, const Vector<T>& rhs) {

  size_t N = this->nRows;

  // Check some sanity
  if ((size_t)rhs.rows() != N) {
    throw std::logic_error("Vector is not the right length");
  }
#ifndef GC_NLINALG_DEBUG
  checkFinite(rhs);
#endif


  // Suitesparse version
#ifdef GC_HAVE_SUITESPARSE

  // Convert input to suitesparse format
  cholmod_dense* inVec = toCholmod(rhs, internals->context);

  // Solve
  cholmod_dense* outVec = cholmod_l_solve(CHOLMOD_A, internals->factorization, inVec, internals->context);

  // Convert back
  toEigen(outVec, internals->context, x);

  // Free
  cholmod_l_free_dense(&outVec, internals->context);
  cholmod_l_free_dense(&inVec, internals->context);

  // Eigen version
#else
  // Solve
  x = internals->solver.solve(rhs);
  if (internals->solver.info() != Eigen::Success) {
    std::cerr << "Solver error: " << internals->solver.info() << std::endl;
    throw std::invalid_argument("Solve failed");
  }
#endif
}

template <typename T>
std::vector<T> PositiveDefiniteSolver<T>::solveSparse(const Vector<T>& rhs,
                                                      const std::vector<size_t>& queryCoordinates) {

  size_t N = this->nRows;

  // Check some sanity
  if ((size_t)rhs.rows() != N) {
    throw std::logic_error("Vector is not the right length");
  }
#ifndef GC_NLINALG_DEBUG
  checkFinite(rhs);
#endif

  std::vector<T> result;

  // Suitesparse version
#ifdef GC_HAVE_SUITESPARSE

  //== Create sparse row vector with sparsity pattern given by queryCoordinates
  size_t Nentries = queryCoordinates.size();
  size_t Ncols = 1;
  size_t Nrows = rhs.rows();

  cholmod_sparse* rhs_sparsity =
      cholmod_l_allocate_sparse(Nrows, Ncols, Nentries, true, true, 0, CHOLMOD_REAL, internals->context);
  cholmod_sparse* soln_sparsity =
      cholmod_l_allocate_sparse(Nrows, Ncols, Nentries, true, true, 0, CHOLMOD_REAL, internals->context);

  // Pull out useful pointers
  double* values = (double*)rhs_sparsity->x;
  SuiteSparse_long* rowIndices = (SuiteSparse_long*)rhs_sparsity->i;
  SuiteSparse_long* colStart = (SuiteSparse_long*)rhs_sparsity->p;

  // Fill in matrix
  for (size_t iEntry = 0; iEntry < Nentries; iEntry++) {
    values[iEntry] = 1;
    rowIndices[iEntry] = queryCoordinates[iEntry];
  }
  colStart[0] = 0;
  colStart[Ncols] = Nentries;

  // Pull out useful pointers
  values = (double*)soln_sparsity->x;
  rowIndices = (SuiteSparse_long*)soln_sparsity->i;
  colStart = (SuiteSparse_long*)soln_sparsity->p;

  // Fill in matrix
  for (size_t iEntry = 0; iEntry < Nentries; iEntry++) {
    values[iEntry] = 1;
    rowIndices[iEntry] = queryCoordinates[iEntry];
  }
  colStart[0] = 0;
  colStart[Ncols] = Nentries;

  // Convert input to suitesparse format
  cholmod_dense* inVec = toCholmod(rhs, internals->context);

  // Solve
  cholmod_dense *X = NULL, *Y = NULL, *E = NULL;
  cholmod_l_solve2(CHOLMOD_A, internals->factorization, inVec, rhs_sparsity, &X, &soln_sparsity, &Y, &E,
                   internals->context);

  // Type wizardry. This type is 'double' if T == 'float', and T otherwise
  // Needed because cholmod always uses double precision
  // (from toEigen in suitesparse_utilities.cpp)
  typedef typename std::conditional<std::is_same<T, float>::value, double, T>::type SCALAR_TYPE;
  SCALAR_TYPE* solution = (SCALAR_TYPE*)X->x;
  for (size_t iC : queryCoordinates) {
    result.push_back(solution[iC]);
  }
  // cholmod_dense* outVec = cholmod_l_solve(CHOLMOD_A, internals->factorization, inVec, internals->context);

  // Convert back
  // toEigen(outVec, internals->context, x);

  // Free
  cholmod_l_free_dense(&X, internals->context);
  cholmod_l_free_dense(&Y, internals->context);
  cholmod_l_free_dense(&E, internals->context);
  // cholmod_l_free_dense(&outVec, internals->context);
  cholmod_l_free_dense(&inVec, internals->context);
  cholmod_l_free_sparse(&rhs_sparsity, internals->context);
  cholmod_l_free_sparse(&soln_sparsity, internals->context);

  // Eigen version
#else
  Vector<T> fullSolution;
  solve(rhs, fullSolution);
  for (size_t iC : queryCoordinates) {
    result.push_back(fullSolution[iC]);
  }
#endif
  return result;
}

template <typename T>
Vector<T> solvePositiveDefinite(SparseMatrix<T>& A, const Vector<T>& rhs) {
  PositiveDefiniteSolver<T> s(A);
  return s.solve(rhs);
}


// Explicit instantiations
template class PositiveDefiniteSolver<double>;
template class PositiveDefiniteSolver<float>;
template class PositiveDefiniteSolver<std::complex<double>>;

template Vector<float> solvePositiveDefinite<float>(SparseMatrix<float>& A, const Vector<float>& rhs);
template Vector<double> solvePositiveDefinite<double>(SparseMatrix<double>& A, const Vector<double>& rhs);
template Vector<std::complex<double>>
solvePositiveDefinite<std::complex<double>>(SparseMatrix<std::complex<double>>& A,
                                            const Vector<std::complex<double>>& rhs);


} // namespace geometrycentral
