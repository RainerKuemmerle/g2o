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

#ifndef G2O_SOLVER_PROPERTY_H
#define G2O_SOLVER_PROPERTY_H

#include <string>

namespace g2o {

/**
 * \brief describe the properties of a solver
 */
struct SolverProperty
{
  std::string name;           ///< name of the solver, e.g., var
  std::string desc;           ///< short description of the solver
  std::string type;           ///< type of solver, e.g., "CSparse Cholesky", "PCG"
  bool requiresMarginalize;   ///< whether the solver requires marginalization of landmarks
  int poseDim;                ///< dimension of the pose vertices (-1 if variable)
  int landmarkDim;            ///< dimension of the landmar vertices (-1 if variable)
  SolverProperty() :
    name(), desc(), type(), requiresMarginalize(false), poseDim(-1), landmarkDim(-1)
  {
  }
  SolverProperty(const std::string& name_, const std::string& desc_, const std::string& type_, bool requiresMarginalize_, int poseDim_, int landmarkDim_) :
    name(name_), desc(desc_), type(type_), requiresMarginalize(requiresMarginalize_), poseDim(poseDim_), landmarkDim(landmarkDim_)
  {
  }
};

} // end namespace

#endif
