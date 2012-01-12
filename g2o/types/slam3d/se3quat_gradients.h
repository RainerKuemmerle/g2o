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

#ifndef G2O_SE3QUAT_GRADIENTS_H
#define G2O_SE3QUAT_GRADIENTS_H

#include <Eigen/Core>

namespace g2o {

using namespace Eigen;

void jacobian_3d_qman ( Matrix< double, 6 , 6> &  Ji , Matrix< double, 6 , 6> &  Jj,
    const double&  z11 , const double&  z12 , const double&  z13 , const double&  z14 ,
  const double&  z21 , const double&  z22 , const double&  z23 , const double&  z24 ,
  const double&  z31 , const double&  z32 , const double&  z33 , const double&  z34 ,
  const double&  xab11 , const double&  xab12 , const double&  xab13 , const double&  xab14 ,
  const double&  xab21 , const double&  xab22 , const double&  xab23 , const double&  xab24 ,
  const double&  xab31 , const double&  xab32 , const double&  xab33 , const double&  xab34 );

} // end namespace

#endif
