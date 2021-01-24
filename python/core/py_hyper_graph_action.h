#include <g2o/core/hyper_graph_action.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace g2o {

void delcareHyperGraphAction(py::module& m) {
  py::class_<HyperGraphAction>(m, "HyperGraphAction");

  // class G2O_CORE_API HyperGraphElementAction{
  // class G2O_CORE_API HyperGraphElementActionCollection: public HyperGraphElementAction{
  // class G2O_CORE_API HyperGraphActionLibrary{
  // class G2O_CORE_API WriteGnuplotAction: public HyperGraphElementAction{
  // class G2O_CORE_API DrawAction : public HyperGraphElementAction{
  // class RegisterActionProxy
  // void G2O_CORE_API applyAction
}

}  // namespace g2o