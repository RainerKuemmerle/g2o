#pragma once

#include <g2o/core/eigen_types.h>

#include "g2opy.h"

namespace g2o {
namespace {

template <typename _Scalar, int _Dim, int _Mode, int _Options = Eigen::AutoAlign>
void templatedEigenIsometry(py::module& m, const std::string& name) {
  using namespace pybind11::literals;
  using CLS = Eigen::Transform<_Scalar, _Dim, _Mode, _Options>;

  typedef typename Eigen::Matrix<_Scalar, CLS::Rows, CLS::HDim, CLS::Options> MatrixType;
  typedef typename Eigen::Matrix<_Scalar, CLS::Dim, CLS::HDim, CLS::Options> CompactMatrixType;
  typedef typename Eigen::Matrix<_Scalar, CLS::Dim, CLS::Dim, CLS::Options> RotationMatrixType;
  typedef typename Eigen::Matrix<_Scalar, CLS::Dim, Eigen::Dynamic> TranslationMatrixType;

  py::class_<CLS>(m, name.c_str())
      //.def(py::init<>())
      .def(py::init([]() { return CLS::Identity(); }))

      .def(py::init<CLS&>(), "other"_a)

      .def(py::init([](MatrixType& m) { return CLS(m); }))

      .def(py::init([](CompactMatrixType& m) {
        MatrixType matrix = MatrixType::Identity();
        matrix.block(0, 0, CLS::Dim, CLS::HDim) = m;
        return CLS(matrix);
      }))

      .def(py::init([](RotationMatrixType& r, TranslationMatrixType& t) {
        MatrixType matrix = MatrixType::Identity();
        matrix.block(0, 0, CLS::Dim, CLS::Dim) = r;
        matrix.block(0, CLS::Dim, CLS::Dim, 1) = t;
        return CLS(matrix);
      }))

      .def(py::init([](Eigen::Quaterniond& q, TranslationMatrixType& t) {
        MatrixType matrix = MatrixType::Identity();
        Eigen::Matrix3d r = q.toRotationMatrix();
        matrix.block(0, 0, CLS::Dim, CLS::Dim) = r.block(0, 0, CLS::Dim, CLS::Dim);
        matrix.block(0, CLS::Dim, CLS::Dim, 1) = t;
        return CLS(matrix);
      }))

      .def("set_rotation",
           [](CLS& trans, RotationMatrixType& r) {
             trans.matrix().block(0, 0, CLS::Dim, CLS::Dim) = r;
           })
      .def("set_rotation",
           [](CLS& trans, Eigen::Quaterniond& q) {
             Eigen::Matrix3d r = q.toRotationMatrix();
             trans.matrix().block(0, 0, CLS::Dim, CLS::Dim) = r;
           })

      .def("set_translation", [](CLS& trans, TranslationMatrixType& t) { trans.translation() = t; })

      .def(py::self * py::self)

      .def("__mul__",
           [](CLS& trans, Eigen::Matrix<_Scalar, CLS::Dim, 1>& t) {
             Eigen::Matrix<_Scalar, CLS::Dim, 1> result = trans * t;
             return result;
           })
      .def("__mul__", [](CLS& trans, TranslationMatrixType& t) { return trans * t; })

      .def("rows", &CLS::rows)
      .def("cols", &CLS::cols)

      .def("__call__", [](CLS& trans, int row, int col) { return trans(row, col); })
      //.def("__getitem__", )

      .def("matrix", (MatrixType & (CLS::*)()) & CLS::matrix)
      .def("set_identity", &CLS::setIdentity)
      .def_static("identity", &CLS::Identity)

      // .def("rotation", [](CLS& trans) {
      //         MatrixType matrix = trans.matrix();
      //         RotationMatrixType r = matrix.block(0, 0, CLS::Dim, CLS::Dim);
      //         return r;
      //     })
      // .def("orientation", [](CLS& trans) {
      //         MatrixType matrix = trans.matrix();
      //         RotationMatrixType r = matrix.block(0, 0, CLS::Dim, CLS::Dim);
      //         return r;
      //     })
      .def("rotation_matrix",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             RotationMatrixType r = matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return r;
           })
      .def_property_readonly("R",
                             [](CLS& trans) {
                               MatrixType matrix = trans.matrix();
                               RotationMatrixType r = matrix.block(0, 0, CLS::Dim, CLS::Dim);
                               return r;
                             })

      .def("Quaternion",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<_Scalar, 3, 3> r = Eigen::Matrix<_Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) = matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      .def("rotation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<_Scalar, 3, 3> r = Eigen::Matrix<_Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) = matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      .def("orientation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<_Scalar, 3, 3> r = Eigen::Matrix<_Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) = matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      // .def_property_readonly("q", [](CLS& trans) {
      //         MatrixType matrix = trans.matrix();
      //         Eigen::Matrix<_Scalar, 3, 3> r = Eigen::Matrix<_Scalar, 3, 3>::Identity();
      //         r.block(0, 0, CLS::Dim, CLS::Dim) = matrix.block(0, 0, CLS::Dim, CLS::Dim);
      //         return Eigen::Quaterniond(r);
      //     })

      .def("translation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<_Scalar, CLS::Dim, 1> t = matrix.block(0, CLS::Dim, CLS::Dim, 1);
             return t;
           })
      .def("position",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<_Scalar, CLS::Dim, 1> t = matrix.block(0, CLS::Dim, CLS::Dim, 1);
             return t;
           })
      .def_property_readonly("t",
                             [](CLS& trans) {
                               MatrixType matrix = trans.matrix();
                               Eigen::Matrix<_Scalar, CLS::Dim, 1> t =
                                   matrix.block(0, CLS::Dim, CLS::Dim, 1);
                               return t;
                             })

      .def("inverse", &CLS::inverse, "traits"_a = (Eigen::TransformTraits)(CLS::Mode))
      .def("make_affine", &CLS::makeAffine)

      ;
}

}  // namespace

void declareEigenTypes(py::module& m);

}  // end namespace g2o
