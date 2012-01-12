#ifndef G2O_UNSCENTED_
#define G2O_UNSCENTED_

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include<Eigen/StdVector>

namespace g2o {
  using namespace Eigen;
  
  template <class SampleType>
  struct SigmaPoint {
    SigmaPoint(const SampleType& sample, double wi, double wp):
      _sample(sample), _wi(wi), _wp(wp){}
    SigmaPoint(): _wi(0), _wp(0) {}
    SampleType _sample;
    double _wi;
    double _wp;
  };
  
  
  template <class SampleType, class CovarianceType>
  bool sampleUnscented(std::vector<SigmaPoint <SampleType> >& sigmaPoints, const SampleType& mean, const CovarianceType& covariance){

    const int dim = mean.size();
    const int numPoints = 2 * dim + 1;
    assert (covariance.rows() == covariance.cols() && covariance.cols() == mean.size() && "Dimension Mismatch");
    const double alpha = 1e-3;
    const double beta  = 2.;
    const double lambda = alpha * alpha * dim;
    const double wi = 1./(2. * (dim + lambda) );
    
    sigmaPoints.resize(numPoints);
    sigmaPoints[0] = SigmaPoint<SampleType>(mean, 
              lambda/(dim + lambda), 
              lambda/(dim + lambda) + (1.-alpha*alpha+beta) ); 
    LLT<CovarianceType> cholDecomp;
    cholDecomp.compute(covariance*(dim+lambda));
    if (cholDecomp.info()==Eigen::NumericalIssue)
      return false;
    CovarianceType L=cholDecomp.matrixL();
    int k=1;
    for (int i=0; i<dim; i++) {
      SampleType s(L.col(i));
      sigmaPoints[k++]=SigmaPoint<SampleType>(mean + s, wi, wi);
      sigmaPoints[k++]=SigmaPoint<SampleType>(mean - s, wi, wi);
    }
    return true;
  }

  template <class SampleType, class CovarianceType>
  void reconstructGaussian(SampleType& mean, CovarianceType& covariance, const std::vector<SigmaPoint<SampleType> >& sigmaPoints){

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
