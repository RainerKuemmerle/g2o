#include "edge_se3.h"
#include "isometry3d_gradients.h"
#include <iostream>

#include "g2o/stuff/opengl_wrapper.h"

namespace g2o {
  using namespace std;

  EdgeSE3::EdgeSE3() : BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3>() {
    information().setIdentity();
  }

  bool EdgeSE3::read(std::istream& is) {
    Vector7d meas;
    for (int i=0; i<7; i++) 
      is >> meas[i];
    // normalize the quaternion to recover numerical precision lost by storing as human readable text
    Vector4d::MapType(meas.data()+3).normalize();
    setMeasurement(internal::fromVectorQT(meas));

    if (is.bad()) {
      return false;
    }
    for ( int i=0; i<information().rows() && is.good(); i++)
      for (int j=i; j<information().cols() && is.good(); j++){
        is >> information()(i,j);
        if (i!=j)
          information()(j,i)=information()(i,j);
      }
    if (is.bad()) {
      //  we overwrite the information matrix with the Identity
      information().setIdentity();
    } 
    return true;
  }

  bool EdgeSE3::write(std::ostream& os) const {
    Vector7d meas=internal::toVectorQT(_measurement);
    for (int i=0; i<7; i++) os  << meas[i] << " ";
    for (int i=0; i<information().rows(); i++)
      for (int j=i; j<information().cols(); j++) {
        os <<  information()(i,j) << " ";
      }
    return os.good();
  }

  void EdgeSE3::computeError() {
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);
    Eigen::Isometry3d delta=_inverseMeasurement * from->estimate().inverse() * to->estimate();
    _error=internal::toVectorMQT(delta);
  }

  bool EdgeSE3::setMeasurementFromState(){
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);
    Eigen::Isometry3d delta = from->estimate().inverse() * to->estimate();
    setMeasurement(delta);
    return true;
  }
  
  void EdgeSE3::linearizeOplus(){
    
    // BaseBinaryEdge<6, Eigen::Isometry3d, VertexSE3, VertexSE3>::linearizeOplus();
    // return;

    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);
    Eigen::Isometry3d E;
    const Eigen::Isometry3d& Xi=from->estimate();
    const Eigen::Isometry3d& Xj=to->estimate();
    const Eigen::Isometry3d& Z=_measurement;
    internal::computeEdgeSE3Gradient(E, _jacobianOplusXi , _jacobianOplusXj, Z, Xi, Xj);
  }

  void EdgeSE3::initialEstimate(const OptimizableGraph::VertexSet& from_, OptimizableGraph::Vertex* /*to_*/) {
    VertexSE3 *from = static_cast<VertexSE3*>(_vertices[0]);
    VertexSE3 *to   = static_cast<VertexSE3*>(_vertices[1]);

    if (from_.count(from) > 0) {
      to->setEstimate(from->estimate() * _measurement);
    } else
      from->setEstimate(to->estimate() * _measurement.inverse());
    //cerr << "IE" << endl;
  }

  EdgeSE3WriteGnuplotAction::EdgeSE3WriteGnuplotAction(): WriteGnuplotAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3WriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertices()[1]);
    Vector6d fromV, toV;
    fromV=internal::toVectorMQT(fromEdge->estimate());
    toV=internal::toVectorMQT(toEdge->estimate());
    for (int i=0; i<6; i++){
      *(params->os) << fromV[i] << " ";
    }
    for (int i=0; i<6; i++){
      *(params->os) << toV[i] << " ";
    }
    *(params->os) << std::endl;
    return this;
  }

#ifdef G2O_HAVE_OPENGL
  EdgeSE3DrawAction::EdgeSE3DrawAction(): DrawAction(typeid(EdgeSE3).name()){}

  HyperGraphElementAction* EdgeSE3DrawAction::operator()(HyperGraph::HyperGraphElement* element, 
               HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;
    refreshPropertyPtrs(params_);
    if (! _previousParams)
      return this;
    
    if (_show && !_show->value())
      return this;
    
    EdgeSE3* e =  static_cast<EdgeSE3*>(element);
    VertexSE3* fromEdge = static_cast<VertexSE3*>(e->vertices()[0]);
    VertexSE3* toEdge   = static_cast<VertexSE3*>(e->vertices()[1]);
    glColor3f(0.5f,0.5f,0.8f);
    glPushAttrib(GL_ENABLE_BIT);
    glDisable(GL_LIGHTING);
    glBegin(GL_LINES);
    glVertex3f((float)fromEdge->estimate().translation().x(),(float)fromEdge->estimate().translation().y(),(float)fromEdge->estimate().translation().z());
    glVertex3f((float)toEdge->estimate().translation().x(),(float)toEdge->estimate().translation().y(),(float)toEdge->estimate().translation().z());
    glEnd();
    glPopAttrib();
    return this;
  }
#endif

}
