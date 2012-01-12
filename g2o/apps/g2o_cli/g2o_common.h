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

#ifndef G2O_COMMON_H
#define G2O_COMMON_H

namespace g2o {

  class DlWrapper;
  class DlSolverWrapper;

  /**
   * will also look for -typeslib in (argc, argv) and load that types
   */
  void loadStandardTypes(DlWrapper& dlWrapper, int argc = 0, char** argv = 0);

  /**
   * will also look for -solverlib in (argc, argv) and load that solver
   */
  void loadStandardSolver(DlWrapper& dlSolverWrapper, int argc = 0, char** argv = 0);

} // end namespace

#endif
