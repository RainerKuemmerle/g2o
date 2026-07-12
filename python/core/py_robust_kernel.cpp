#include "py_robust_kernel.h"

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace g2o {

void declareRobustKernel(py::module_& m) {
  py::class_<RobustKernel>(m, "BaseRobustKernel")
      .def("robustify", &RobustKernel::robustify, "squared_error"_a,
           "rho"_a)  // (double, Vector3&) -> void
      .def("set_delta", &RobustKernel::setDelta,
           "delta"_a)                      // double -> void
      .def("delta", &RobustKernel::delta)  // -> double
      ;

  py::class_<RobustKernelScaleDelta, RobustKernel>(m, "RobustKernelScaleDelta")
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

  py::class_<RobustKernelHuber, RobustKernel>(m, "RobustKernelHuber")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelHuber();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)
      .def("robustify", &RobustKernelHuber::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelPseudoHuber, RobustKernel>(m,
                                                    "RobustKernelPseudoHuber")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelPseudoHuber();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelPseudoHuber::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelCauchy, RobustKernel>(m, "RobustKernelCauchy")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelCauchy();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelCauchy::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelGemanMcClure, RobustKernel>(m,
                                                     "RobustKernelGemanMcClure")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelGemanMcClure();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelGemanMcClure::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelWelsch, RobustKernel>(m, "RobustKernelWelsch")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelWelsch();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelWelsch::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelFair, RobustKernel>(m, "RobustKernelFair")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelFair();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelFair::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelTukey, RobustKernel>(m, "RobustKernelTukey")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelTukey();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelTukey::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelSaturated, RobustKernel>(m, "RobustKernelSaturated")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelSaturated();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelSaturated::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;

  py::class_<RobustKernelDCS, RobustKernel>(m, "RobustKernelDCS")
      .def(py::init<>())
      //.def(py::init<double>(), "delta"_a)
      .def(py::new_([](double delta) {
             auto* kernel = new RobustKernelDCS();
             kernel->setDelta(delta);
             return kernel;
           }),
           "delta"_a)

      .def("robustify", &RobustKernelDCS::robustify, "e2"_a,
           "rho"_a)  // (double, Vector3&) -> void
      ;
}

}  // end namespace g2o
