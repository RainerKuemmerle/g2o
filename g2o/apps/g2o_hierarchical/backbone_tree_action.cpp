#include <limits>
#include "g2o/core/factory.h"
#include "backbone_tree_action.h"
#include "edge_types_cost_function.h"
namespace g2o {

  using namespace std;

  BackBoneTreeAction::BackBoneTreeAction(SparseOptimizer* optimizer, std::string vertexTag, int level, int step):
    _optimizer(optimizer),
    _vertexTag(vertexTag),
    _level(level),
    _step(step) {
    _factory=Factory::instance();
    init();
  }

  void BackBoneTreeAction::init(){
    _vsMap.clear();
    _vsMmap.clear();
    _freeEdges.clear();
    for (HyperGraph::EdgeSet::iterator it=_optimizer->edges().begin(); it!=_optimizer->edges().end(); it++){
      OptimizableGraph::Edge* e = (OptimizableGraph::Edge*)(*it);
      if (e->level()==_level) {
  _freeEdges.insert(e);
      }
    }
  }

  double BackBoneTreeAction::perform(HyperGraph::Vertex* v,
             HyperGraph::Vertex* vParent,
             HyperGraph::Edge* e,
             double distance){
    int depth=(int) distance;
    if (_factory->tag(v)!= _vertexTag)
      return 0;
    Star* parentStar=getStar(vParent);
    if (! parentStar){
      parentStar=new Star(_level+1,_optimizer);
      addToMap(parentStar, vParent);
      parentStar->_gauge.insert(vParent);
    }
    addToMap(parentStar,v);
    fillStar(parentStar, e);

    // every _step levels you go down in the tree, create a new star
    if (depth && ! (depth%_step )){
      Star* star=new Star(_level+1, _optimizer);
      addToMap(star,v);
      star->_gauge.insert(v);
    }
    return 1;
  }


  void  BackBoneTreeAction::addToMap(Star* s, HyperGraph::Vertex* v_){
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)v_;
    VertexStarMap::iterator it=_vsMap.find(v);
    if (it!=_vsMap.end())
      it->second = s;
    else
      _vsMap.insert(make_pair(v,s));
    _vsMmap.insert(make_pair(v,s));
    s->_lowLevelVertices.insert(v);
  }

  Star* BackBoneTreeAction::getStar(HyperGraph::Vertex* v_){
    OptimizableGraph::Vertex* v= (OptimizableGraph::Vertex*)v_;
    VertexStarMap::iterator it=_vsMap.find(v);
    if (it==_vsMap.end())
      return 0;
    return it->second;
  }

  bool BackBoneTreeAction::fillStar(Star* s, HyperGraph::Edge* e_){
    OptimizableGraph::Edge* e = (OptimizableGraph::Edge*) e_;
    HyperGraph::EdgeSet::iterator it=_freeEdges.find(e);
    if (it!=_freeEdges.end()) {
      _freeEdges.erase(it);
      s->_lowLevelEdges.insert(e);
      for (size_t i=0; i<e->vertices().size(); i++){
  s->_lowLevelVertices.insert(e->vertices()[i]);
      }
      return true;
    }
    return false;
  }
}
