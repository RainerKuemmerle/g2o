#include "py_robust_kernel.h"

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace g2o {

void declareRobustKernel(py::module& m) {
  py::classh<RobustKernel>(m, "BaseRobustKernel")
      .def("robustify", &RobustKernel::robustify, "squared_error"_a,
           "rho"_a)  // (double, Vector3&) -> void
      .def("set_delta", &RobustKernel::setDelta,
           "delta"_a)                      // double -> void
      .def("delta", &RobustKernel::delta)  // -> double
      ;

  py::classh<RobustKernelScaleDelta, RobustKernel>(m, "RobustKernelScaleDelta")
      .def(py::init<>())
      .def(py::init<double>(), "delta"_a = 1.)
      .def(py::init<const RobustKernelPtr&, double>(), "kernel"_a,
           "delta"_a = 1., py::keep_alive<1, 2>())
      .def("kernel", &RobustKernelScaleDelta::kernel)  // -> RobustKernelPtr&
      .def("set_kernel", &RobustKernelScaleDelta::setKernel, "ptr"_a,
           py::keep_alive<1, 2>())  // const RobustKernelPtr& ->
      .def("robustify", &RobustKernelScaleDelta::robustify, "error"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelHuber, RobustKernel>(m, "RobustKernelHuber")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelHuber kernel = RobustKernelHuber();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)
      .def("robustify", &RobustKernelHuber::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelPseudoHuber, RobustKernel>(m,
                                                    "RobustKernelPseudoHuber")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelPseudoHuber kernel = RobustKernelPseudoHuber();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelPseudoHuber::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelCauchy, RobustKernel>(m, "RobustKernelCauchy")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelCauchy kernel = RobustKernelCauchy();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelCauchy::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelGemanMcClure, RobustKernel>(m,
                                                     "RobustKernelGemanMcClure")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelGemanMcClure kernel = RobustKernelGemanMcClure();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelGemanMcClure::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelWelsch, RobustKernel>(m, "RobustKernelWelsch")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelWelsch kernel = RobustKernelWelsch();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelWelsch::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelFair, RobustKernel>(m, "RobustKernelFair")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelFair kernel = RobustKernelFair();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelFair::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelTukey, RobustKernel>(m, "RobustKernelTukey")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelTukey kernel = RobustKernelTukey();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelTukey::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelSaturated, RobustKernel>(m, "RobustKernelSaturated")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelSaturated kernel = RobustKernelSaturated();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelSaturated::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::classh<RobustKernelDCS, RobustKernel>(m, "RobustKernelDCS")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::init([](double delta) {
             RobustKernelDCS kernel = RobustKernelDCS();
             kernel.setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelDCS::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;
}

}  // end namespace g2o
