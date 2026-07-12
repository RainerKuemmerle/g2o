#include "py_factory.h"

#include <g2o/core/factory.h>
#include <g2o/core/hyper_graph.h>

namespace g2o {

void declareFactory(py::module_& m) {
  // Wrap Factory::TypeInfo struct
  py::class_<Factory::TypeInfo>(m, "FactoryTypeInfo")
      .def(py::init<>())
      .def_rw("element_type_bit", &Factory::TypeInfo::elementTypeBit)
      .def_rw("dimension", &Factory::TypeInfo::dimension)
      .def_rw("dimension_at_compile_time",
              &Factory::TypeInfo::dimension_at_compile_time)
      .def_rw("minimal_dimension", &Factory::TypeInfo::minimal_dimension)
      .def_rw("number_vertices", &Factory::TypeInfo::number_vertices)
      .def_rw("number_vertices_at_compile_time",
              &Factory::TypeInfo::number_vertices_at_compile_time)
      .def_rw("number_parameters", &Factory::TypeInfo::number_parameters)
      .def_rw("error_dimension", &Factory::TypeInfo::error_dimension)
      .def_rw("error_dimension_at_compile_time",
              &Factory::TypeInfo::error_dimension_at_compile_time);

  py::class_<Factory>(m, "Factory", py::never_destruct{})
      .def_static(
          "instance", []() -> Factory* { return Factory::instance(); },
          py::rv_policy::reference)
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
