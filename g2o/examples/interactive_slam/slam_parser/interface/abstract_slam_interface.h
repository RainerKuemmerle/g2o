#ifndef ABSTRACT_SLAM_INTERFACE_H
#define ABSTRACT_SLAM_INTERFACE_H

#include <vector>
#include <string>

namespace SlamParser {
  
  /**
   * \brief interface for communicating with the SLAM algorithm
   *
   * This interface allows the parser to communicate with the SLAM algorithm.
   */
  class AbstractSlamInterface {
    public:
      /**
       * adding a node to the SLAM engine.
       * @param tag: the tag specifying the type of the vertex
       * @param id: the unique id of the node.
       * @param dimension: the dimension of the node.
       * @param values: the pose of the node, may be empty (i.e., the engine should initialize the node itself)
       * @return true, if adding was successful
       */
      virtual bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values) = 0;

      /**
       * adding an edge to the SLAM engine.
       * @param tag: the tag specifying the type of the vertex
       * @param id: the unique id of the edge.
       * @param dimension: the dimension of the edge.
       * @param v1: the unique id of the edge of the first vertex
       * @param v2: the unique id of the edge of the second vertex
       * @param measurement: the measurement of the constraint
       * @param information: the information matrix (inverse of the covariance) representing the uncertainty of the measurement (row-major upper triangular and diagonal)
       * @return true, if adding was successful
       */
      virtual bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information) = 0;

      /**
       * set some nodes to a fixed position
       * @param nodes: the list of vertex IDs to fix
       * @return true, if successful
       */
      virtual bool fixNode(const std::vector<int>& nodes) = 0;

      /**
       * Ask the SLAM engine to print the current estimate of the variables
       * @param nodes: the list of vertex IDs to print, if empty print all variables
       * @return true, if successful
       */
      virtual bool queryState(const std::vector<int>& nodes) = 0;

      /**
       * ask the engine to solve
       * @return true, if successful
       */
      virtual bool solveState() = 0;
  };

}

#endif
