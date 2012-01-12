#ifndef EXAMPLE_SLAM_INTERFACE_H
#define EXAMPLE_SLAM_INTERFACE_H

#include "slam_parser/interface/abstract_slam_interface.h"

#include <map>
#include <vector>

/**
 * \brief example for an interface to a SLAM algorithm
 *
 * Example for an interface to a SLAM algorithm. You may modify this class to fit your needs.
 * The example class does not actually perform any optimization, it just keeps the input values
 * and outputs the same values if asked. See the documentation of SlamParser::AbstractSlamInterface
 * for details.
 */
class ExampleSlamInterface : public SlamParser::AbstractSlamInterface
{
  public:
    ExampleSlamInterface();

    bool addNode(const std::string& tag, int id, int dimension, const std::vector<double>& values);

    bool addEdge(const std::string& tag, int id, int dimension, int v1, int v2, const std::vector<double>& measurement, const std::vector<double>& information);

    bool fixNode(const std::vector<int>& nodes);

    bool queryState(const std::vector<int>& nodes);

    bool solveState();

  protected:
    /* add variables to control the SLAM algorithm or for other general requirements */
    std::map<int, std::pair<std::string, std::vector<double> > > _vertices; ///< the original value of the input (actually not needed if a real SLAM engine is running)
};

#endif
