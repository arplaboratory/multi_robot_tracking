#ifndef _HUNGARIAN_ALG_H_
#define _HUNGARIAN_ALG_H_

#include <vector>
#include <iostream>
#include <limits>
#include <time.h>
#include <assert.h>


// http://community.topcoder.com/tc?module=Static&d1=tutorials&d2=hungarianAlgorithm
typedef float track_t;
typedef std::vector<int> assignments_t;
typedef std::vector<track_t> distMatrix_t;

class AssignmentProblemSolver
{
private:
  // --------------------------------------------------------------------------
  // Computes the optimal assignment (minimum overall costs) using Munkres algorithm.
  // --------------------------------------------------------------------------
  void assignmentoptimal(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);
  void buildassignmentvector(assignments_t& assignment, bool *starMatrix, size_t nOfRows, size_t nOfColumns);
  void computeassignmentcost(const assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows);
  void step2a(assignments_t& assignment, track_t *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
  void step2b(assignments_t& assignment, track_t *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
  void step3(assignments_t& assignment, track_t *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
  void step4(assignments_t& assignment, track_t *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim, size_t row, size_t col);
  void step5(assignments_t& assignment, track_t *distMatrix, bool *starMatrix, bool *newStarMatrix, bool *primeMatrix, bool *coveredColumns, bool *coveredRows, size_t nOfRows, size_t nOfColumns, size_t minDim);
  // --------------------------------------------------------------------------
  // Computes a suboptimal solution. Good for cases with many forbidden assignments.
  // --------------------------------------------------------------------------
  void assignmentsuboptimal1(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);
  // --------------------------------------------------------------------------
  // Computes a suboptimal solution. Good for cases with many forbidden assignments.
  // --------------------------------------------------------------------------
  void assignmentsuboptimal2(assignments_t& assignment, track_t& cost, const distMatrix_t& distMatrixIn, size_t nOfRows, size_t nOfColumns);

public:
  enum TMethod
  {
    optimal,
    many_forbidden_assignments,
    without_forbidden_assignments
  };

  void Solve(const std::vector<float> distMatrixIn, uint nOfRows, uint nOfColumns, std::vector<int>& assignment, TMethod Method = optimal);

};



#endif
