#include "py_factory.h"

#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>

namespace g2o {

void declareFactory(py::module& m) {
  // Wrap Factory::TypeInfo struct
  py::class_<Factory::TypeInfo>(m, "FactoryTypeInfo")
      .def(py::init<>())
      .def_readwrite("element_type_bit", &Factory::TypeInfo::elementTypeBit)
      .def_readwrite("dimension", &Factory::TypeInfo::dimension)
      .def_readwrite("dimension_at_compile_time",
                     &Factory::TypeInfo::dimension_at_compile_time)
      .def_readwrite("minimal_dimension", &Factory::TypeInfo::minimal_dimension)
      .def_readwrite("number_vertices", &Factory::TypeInfo::number_vertices)
      .def_readwrite("number_vertices_at_compile_time",
                     &Factory::TypeInfo::number_vertices_at_compile_time)
      .def_readwrite("number_parameters", &Factory::TypeInfo::number_parameters)
      .def_readwrite("error_dimension", &Factory::TypeInfo::error_dimension)
      .def_readwrite("error_dimension_at_compile_time",
                     &Factory::TypeInfo::error_dimension_at_compile_time);

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
      .def(
          "construct",
          [](Factory& factory, const std::string& tag) {
            return factory.construct(tag);
          },
          "tag"_a)
      .def(
          "type_info",
          [](Factory& factory, const std::string& tag) {
            return factory.typeInfo(tag);
          },
          "tag"_a);
}

}  // end namespace g2o
