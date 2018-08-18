#include "g2o/stuff/sampler.h"
#include "g2o/stuff/command_args.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_unary_edge.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"

using namespace ::std;

class DynamicVertex : public g2o::BaseVertex<Eigen::Dynamic, Eigen::VectorXd>
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  using g2o::BaseVertex<Eigen::Dynamic, Eigen::VectorXd>::resizeDimensionImpl;
  
  DynamicVertex()
  {
  }
  
  virtual bool read(std::istream& /*is*/)
  {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
  }

  virtual bool write(std::ostream& /*os*/) const
  {
    cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    return false;
    }
  
  virtual void setToOriginImpl()
  {
    _estimate.setZero();
  }
  
  virtual void oplusImpl(const double* update)
  {
    Eigen::VectorXd::ConstMapType v(update, _dimension);
      _estimate += v;
  }

  virtual void resizeDimensionImpl(int newDimension)
  {
    cerr << __PRETTY_FUNCTION__ << " invoked with " << newDimension << endl;
    _estimate.resize(newDimension);
    cout << "_estimate.rows()=" << _estimate.rows() << endl;
    cout << "_estimate.cols()=" << _estimate.cols() << endl;
  }
};

int main(int argc, const char* argv[])
{
  DynamicVertex v;

  cout << "v.dimension()=" << v.dimension() << endl;

  v.resizeDimension(15);
  cout << "v.dimension()=" << v.dimension() << endl;
  
  
  v.setToOrigin();
  
  cout << "v.estimate()=" << v.estimate() << endl;
  
  
}
