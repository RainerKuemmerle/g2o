// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, H. Strasdat, W. Burgard
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

#ifndef G2O_BASE_EDGE_H
#define G2O_BASE_EDGE_H

#include <iostream>
#include <limits>

#include <Eigen/Core>

#include "optimizable_graph.h"

namespace g2o {

  using namespace Eigen;

  template <int D, typename E>
  class BaseEdge : public OptimizableGraph::Edge
  {
    public:

      static const int Dimension = D;
      typedef E Measurement;
      typedef Matrix<double, D, 1> ErrorVector;
      typedef Matrix<double, D, D> InformationType;

      BaseEdge() : OptimizableGraph::Edge()
      {
        _dimension = D;
      }

      virtual ~BaseEdge() {}

      virtual double chi2() const 
      {
        return _error.dot(information()*_error);
      }

      virtual void robustifyError()
      {
        double nrm = sqrt(_error.dot(information()*_error));
        double w = sqrtOfHuberByNrm(nrm,_huberWidth);
        _error *= w;
      }

      virtual const double* errorData() const { return _error.data();}
      virtual double* errorData() { return _error.data();}
      const ErrorVector& error() const { return _error;}
      ErrorVector& error() { return _error;}

      //! information matrix of the constraint
      const InformationType& information() const { return _information;}
      InformationType& information() { return _information;}
      void setInformation(const InformationType& information) { _information = information;}

      virtual const double* informationData() const { return _information.data();}
      virtual double* informationData() { return _information.data();}

      //! accessor functions for the measurement represented by the edge
      const Measurement& measurement() const { return _measurement;}
      virtual void setMeasurement(const Measurement& m) { _measurement = m;}

      virtual int rank() const {return _dimension;}

      virtual void initialEstimate(const OptimizableGraph::VertexSet&, OptimizableGraph::Vertex*)
      {
        std::cerr << "inititialEstimate() is not implemented, please give implementation in your derived class" << std::endl;
      }

    protected:

      Measurement _measurement;
      InformationType _information;
      ErrorVector _error;
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // end namespace g2o

#endif
