#ifndef G2O_EDGE_CREATOR_
#define G2O_EDGE_CREATOR_

#include "g2o/core/sparse_optimizer.h"
#include <Eigen/Core>
#include <vector>

#include "g2o_hierarchical_api.h"

namespace g2o {

  /**
   * Class that implements a simple edge_creation, based on the types and the
   * ordes of the vertices passed as argument.  Namely, based on an ordered
   * vector of vertices this class implements a method that construct a new
   * edge compatible with the vertices in the vector. The order of the vector
   * matters.  This class is heavily based on the Factory, and utilizes strings
   * to identify the edge types.
   */
struct G2O_HIERARCHICAL_API EdgeCreator{
  struct EdgeCreatorEntry {
    EdgeCreatorEntry(const std::string& edgeTypeName, const std::vector<int>& parameterIds)
    :_edgeTypeName(edgeTypeName), _parameterIds(parameterIds) {}

    EdgeCreatorEntry(const std::string& edgeTypeName)
    :_edgeTypeName(edgeTypeName){}

    std::string _edgeTypeName;
    std::vector<int> _parameterIds;
  };

  typedef std::map<std::string, EdgeCreatorEntry> EntryMap;

  //! Adds an association to the association map
  //! @param vertexTypes: a string containing the tags of the vertices separated by a ";".
  //! For instance an edge between a VertexSE2 and and EdgeSE2 is identified by the string "VERTEX_SE2;EDGE_SE2;".
  //! The order matters.
  //! @param edgeType: the tag of edge to create
  //! @returns false on failure (incompatible types). Currently returns always true because i did not have time to implement checks
  bool addAssociation(const std::string& vertexTypes, const std::string& edgeType);

  //! Adds an association to the association map
  //! @param vertexTypes: a string containing the tags of the vertices separated by a ";".
  //! For instance an edge between a VertexSE2 and and EdgeSE2 is identified by the string "VERTEX_SE2;EDGE_SE2;".
  //! The order matters.
  //! @param edgeType: the tag of edge to create
  //! @param parameterIds: the ids of the parameters uses as argument when creating the edge (the same as Edge::_parameterIds)
  //! @returns false on failure (incompatible types). Currently returns always true because i did not have time to implement checks
  bool addAssociation(const std::string& vertexTypes, const std::string& edgeType, const std::vector<int>& parameterIds);

  //! Removes an association to the association map.
  //! @params vertexTypes: the string of the vertex ids connected by the edge
  bool removeAssociation(std::string vertexTypes);


  //! constructs an edge based on the verticesVector given as argument
  //! The vertices of the newly created edge are set to the parameter
  //! @params vertices: the vertices to be connected by the new edge
  //! @returns the new edge on succes, 0 on failure (no edge association in the map compatible with the vertices in the parameter)
  OptimizableGraph::Edge* createEdge(std::vector<OptimizableGraph::Vertex*>& vertices );

protected:
  EntryMap _vertexToEdgeMap;
};


} // end namespace
#endif
