#include "py_hyper_graph.h"

#include <g2o/core/hyper_graph.h>

namespace g2o {

void declareHyperGraph(py::module& m) {
  py::class_<HyperGraph> cls(m, "HyperGraph");

  py::enum_<HyperGraph::HyperGraphElementType>(cls, "HyperGraphElementType")
      .value("HGET_VERTEX", HyperGraph::HyperGraphElementType::kHgetVertex)
      .value("HGET_EDGE", HyperGraph::HyperGraphElementType::kHgetEdge)
      .value("HGET_PARAMETER",
             HyperGraph::HyperGraphElementType::kHgetParameter)
      .value("HGET_CACHE", HyperGraph::HyperGraphElementType::kHgetCache)
      .value("HGET_DATA", HyperGraph::HyperGraphElementType::kHgetData)
      .value("HGET_NUM_ELEMS", HyperGraph::HyperGraphElementType::kHgetNumElems)
      .export_values();

  py::enum_<HyperGraph::HyperGraphDefaultIds>(cls, "HyperGraphDefaultIds")
      .value("UnassignedId", HyperGraph::HyperGraphDefaultIds::kUnassignedId)
      .value("InvalidId", HyperGraph::HyperGraphDefaultIds::kInvalidId)
      .export_values();

  py::class_<HyperGraph::HyperGraphElement,  // NOLINT
             std::shared_ptr<HyperGraph::HyperGraphElement>>(
      cls, "HyperGraphElement");

  py::class_<HyperGraph::Data, HyperGraph::HyperGraphElement,
             std::shared_ptr<HyperGraph::Data>>(cls, "Data")
      //.def(py::init<>())   // invalid new-expression of abstract class
      .def("element_type",
           &HyperGraph::Data::elementType)  // virtual, -> HyperGraphElementType
      .def("next", &HyperGraph::Data::next)
      .def("set_next", &HyperGraph::Data::setNext, "next"_a,
           py::keep_alive<1, 2>())  // -> void
      .def("data_container", &HyperGraph::Data::dataContainer)
      .def("set_data_container", &HyperGraph::Data::setDataContainer,
           "data_container"_a, py::keep_alive<1, 2>())  // -> void
      ;

  py::class_<HyperGraph::DataContainer,
             std::shared_ptr<HyperGraph::DataContainer>>(cls, "DataContainer")
      .def(py::init<>())
      .def("user_data", &HyperGraph::DataContainer::userData)
      .def("set_user_data", &HyperGraph::DataContainer::setUserData, "obs"_a,
           py::keep_alive<1, 2>())  // Data* -> void
      .def("add_user_data", &HyperGraph::DataContainer::addUserData, "obs"_a,
           py::keep_alive<1, 2>())  // Data* -> void
      ;

  // typedef std::set<Edge*>                           EdgeSet;
  // typedef std::set<Vertex*>                         VertexSet;
  // typedef std::unordered_map<int, Vertex*>     VertexIDMap;
  // typedef std::vector<Vertex*>                      VertexContainer;
  // automatically convert python type, by COPY data

  // py::bind_set<std::set<HyperGraph::Edge*>>(cls, "EdgeSet");

  py::class_<HyperGraph::Vertex, HyperGraph::HyperGraphElement,
             std::shared_ptr<HyperGraph::Vertex>>(cls, "Vertex")
      .def(py::init<int>(), "id"_a = HyperGraph::kUnassignedId)
      .def("id", &HyperGraph::Vertex::id)  // -> int
      .def("set_id", &HyperGraph::Vertex::setId,
           "id"_a)  // int -> void
      .def("edges",
           static_cast<HyperGraph::EdgeSetWeak& (HyperGraph::Vertex::*)()>(
               &HyperGraph::Vertex::edges),
           py::return_value_policy::reference)
      .def("element_type",
           &HyperGraph::Vertex::elementType)  // virtual, ->
                                              // HyperGraphElementType
      ;

  py::class_<HyperGraph::Edge, HyperGraph::HyperGraphElement,
             std::shared_ptr<HyperGraph::Edge>>(cls, "Edge")
      .def(py::init<int>(), "id"_a = HyperGraph::kInvalidId)
      .def("resize", &HyperGraph::Edge::resize,
           "size"_a)  // virtual, size_t -> void
      .def("vertices",
           static_cast<HyperGraph::VertexContainer& (HyperGraph::Edge::*)()>(
               &HyperGraph::Edge::vertices),
           py::return_value_policy::reference)
      .def("vertex",
           static_cast<std::shared_ptr<HyperGraph::Vertex> (
               HyperGraph::Edge::*)(size_t)>(&HyperGraph::Edge::vertex),
           "i"_a, py::return_value_policy::reference)
      .def("set_vertex", &HyperGraph::Edge::setVertex, "i"_a, "v"_a,
           py::keep_alive<1, 2>())  // (size_t, Vertex*) -> void

      .def("id", &HyperGraph::Edge::id)  // -> int
      .def("set_id", &HyperGraph::Edge::setId,
           "id"_a)  // int -> void
      .def("element_type",
           &HyperGraph::Edge::elementType)  // virtual, -> HyperGraphElementType
      .def("num_undefined_vertices",
           &HyperGraph::Edge::numUndefinedVertices)  // -> int
      ;

  cls.def(py::init<>());
  cls.def("vertex",
          static_cast<std::shared_ptr<HyperGraph::Vertex> (HyperGraph::*)(int)>(
              &HyperGraph::vertex),
          "id"_a, py::return_value_policy::reference);

  cls.def("remove_vertex", &HyperGraph::removeVertex, "v"_a,
          "detach"_a);  // virtual, (Vertex*, bool) -> bool
  cls.def("remove_edge", &HyperGraph::removeEdge,
          "e"_a);                        // virtual, Edge* -> bool
  cls.def("clear", &HyperGraph::clear);  // virtual, ->void

  cls.def("vertices",
          static_cast<HyperGraph::VertexIDMap& (HyperGraph::*)()>(
              &HyperGraph::vertices),
          py::return_value_policy::reference);
  cls.def(
      "edges",
      static_cast<HyperGraph::EdgeSet& (HyperGraph::*)()>(&HyperGraph::edges),
      py::return_value_policy::reference);

  cls.def("add_vertex", &HyperGraph::addVertex, "v"_a,
          py::keep_alive<1, 2>());  // virtual, Vertex* -> bool
  cls.def("add_edge", &HyperGraph::addEdge, "e"_a,
          py::keep_alive<1, 2>());  // virtual, Edge* -> bool
  cls.def("set_edge_vertex", &HyperGraph::setEdgeVertex, "e"_a, "pos"_a, "v"_a,
          py::keep_alive<1, 2>(),
          py::keep_alive<1, 4>());  // virtual, (Edge*, int, Vertex*) -> bool
  cls.def("merge_vertices", &HyperGraph::mergeVertices, "v_big"_a, "v_small"_a,
          "erase"_a);  // virtual, (Vertex*, Vertex*, bool) -> bool
  cls.def("detach_vertex", &HyperGraph::detachVertex,
          "v"_a);  // virtual, Vertex* -> bool
  cls.def("change_id", &HyperGraph::changeId, "v"_a,
          "new_id"_a);  // virtual, (Vertex*, int) -> bool
}

}  // end namespace g2o
