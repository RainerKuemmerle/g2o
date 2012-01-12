// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include "batch_stats.h"
#include <cstring>

namespace g2o {
  using namespace std;

  G2OBatchStatistics * globalStats=0;

  #ifndef PTHING
  #define PTHING(s) \
    #s << "= " << (st.s) << "\t "
  #endif

  G2OBatchStatistics::G2OBatchStatistics(){
    // zero all.
    memset (this, 0, sizeof(G2OBatchStatistics));
  }

  std::ostream& operator << (std::ostream& os , const G2OBatchStatistics& st)
  {
    os << PTHING(iteration);

    os << PTHING( numVertices ); // how many vertices are involved
    os << PTHING( numEdges ); // hoe many edges
    os << PTHING(  chi2 );  // total chi2
    
    /** timings **/
    // nonlinear part
    os << PTHING(  timeResiduals );  
    os << PTHING(  timeLinearize );   // jacobians
    os << PTHING(  timeQuadraticForm ); // construct the quadratic form in the graph
    
    // block_solver (constructs Ax=b, plus maybe schur);
    os << PTHING(  timeSchurrComplement ); // compute schurr complement (0 if not done);
    
    // linear solver (computes Ax=b); );
    os << PTHING(  timeSymbolicDecomposition ); // symbolic decomposition (0 if not done);
    os << PTHING(  timeNumericDecomposition ); // numeric decomposition  (0 if not done);
    os << PTHING(  timeLinearSolution );             // total time for solving Ax=b
    os << PTHING(  iterationsLinearSolver );  // iterations of PCG
    os << PTHING(  timeUpdate ); // oplus
    os << PTHING(  timeIteration ); // total time );

    os << PTHING( levenbergIterations );
    os << PTHING( timeLinearSolver);

    os << PTHING(hessianDimension);
    os << PTHING(hessianPoseDimension);
    os << PTHING(hessianLandmarkDimension);
    os << PTHING(choleskyNNZ);
    os << PTHING(timeMarginals);

    return os;
  };

}
