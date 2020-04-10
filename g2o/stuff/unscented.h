// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_UNSCENTED_
#define G2O_UNSCENTED_

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace g2o {
  
  template <class SampleType>
  struct SigmaPoint {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SigmaPoint(const SampleType& sample, number_t wi, number_t wp):
      _sample(sample), _wi(wi), _wp(wp){}
    SigmaPoint(): _wi(0), _wp(0) {}
    SampleType _sample;
    number_t _wi;
    number_t _wp;
  };
  
  
  template <class SampleType, class CovarianceType>
  bool sampleUnscented(std::vector<SigmaPoint <SampleType>, Eigen::aligned_allocator<SigmaPoint <SampleType> > >& sigmaPoints, const SampleType& mean, const CovarianceType& covariance){

    const int dim = mean.size();
    const int numPoints = 2 * dim + 1;
    assert (covariance.rows() == covariance.cols() && covariance.cols() == mean.size() && "Dimension Mismatch");
    const number_t alpha = cst(1e-3);
    const number_t beta  = 2;
    const number_t lambda = alpha * alpha * dim;
    const number_t wi = cst(1) / (2 * (dim + lambda) );
    
    sigmaPoints.resize(numPoints);
    sigmaPoints[0] = SigmaPoint<SampleType>(mean, 
              lambda/(dim + lambda), 
              lambda/(dim + lambda) + (1.-alpha*alpha+beta) ); 
    Eigen::LLT<CovarianceType> cholDecomp;
    cholDecomp.compute(covariance*(dim+lambda));
    if (cholDecomp.info()==Eigen::NumericalIssue)
      return false;
    const CovarianceType& L=cholDecomp.matrixL();
    int k=1;
    for (int i=0; i<dim; i++) {
      SampleType s(L.col(i));
      sigmaPoints[k++]=SigmaPoint<SampleType>(mean + s, wi, wi);
      sigmaPoints[k++]=SigmaPoint<SampleType>(mean - s, wi, wi);
    }
    return true;
  }

  template <class SampleType, class CovarianceType>
  void reconstructGaussian(SampleType& mean, CovarianceType& covariance,
			   const std::vector<SigmaPoint<SampleType>, Eigen::aligned_allocator<SigmaPoint <SampleType> > >& sigmaPoints){

    mean.fill(0);
    covariance.fill(0);
    for (size_t i=0; i<sigmaPoints.size(); i++){
      mean += sigmaPoints[i]._wi * sigmaPoints[i]._sample;
    }
    for (size_t i=0; i<sigmaPoints.size(); i++){
      SampleType delta = sigmaPoints[i]._sample - mean;
      covariance += sigmaPoints[i]._wp * ( delta* delta.transpose() ) ;
    }

  }
}

#endif
