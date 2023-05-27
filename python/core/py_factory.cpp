#include "py_factory.h"

#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>

namespace g2o {

void declareFactory(py::module& m) {
  py::class_<Factory, std::unique_ptr<Factory, py::nodelete>>(m, "Factory")
      .def(py::init([]() {
        return std::unique_ptr<Factory, py::nodelete>(Factory::instance());
      }))
      .def(
          "knows_tag",
          [](Factory& factory, const std::string& tag) {
            int elementType = -1;
            bool result = factory.knowsTag(tag, &elementType);
            return std::make_pair(
                result,
                static_cast<HyperGraph::HyperGraphElementType>(elementType));
          },
          "tag"_a)
      .def("known_types",
           [](Factory& factory) {
             std::vector<std::string> types;
             factory.fillKnownTypes(types);
             return types;
           })
      // TODO(Rainer): Wrap remaining functions
      ;
}

}  // end namespace g2o
