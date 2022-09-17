#include "py_eigen_types.h"

#include "g2o/stuff/misc.h"

namespace g2o {

namespace {

template <typename Scalar, int Dim, int Mode, int Options = Eigen::AutoAlign>
void templatedEigenIsometry(py::module& m, const std::string& name) {
  using CLS = Eigen::Transform<Scalar, Dim, Mode, Options>;

  using MatrixType =
      typename Eigen::Matrix<Scalar, CLS::Rows, CLS::HDim, CLS::Options>;
  using CompactMatrixType =
      typename Eigen::Matrix<Scalar, CLS::Dim, CLS::HDim, CLS::Options>;
  using RotationMatrixType =
      typename Eigen::Matrix<Scalar, CLS::Dim, CLS::Dim, CLS::Options>;
  using TranslationMatrixType =
      typename Eigen::Matrix<Scalar, CLS::Dim, Eigen::Dynamic>;

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
        matrix.block(0, 0, CLS::Dim, CLS::Dim) =
            r.block(0, 0, CLS::Dim, CLS::Dim);
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

      .def(
          "set_translation",
          [](CLS& trans, TranslationMatrixType& t) { trans.translation() = t; })

      .def(py::self * py::self)

      .def("__mul__",
           [](CLS& trans, Eigen::Matrix<Scalar, CLS::Dim, 1>& t) {
             Eigen::Matrix<Scalar, CLS::Dim, 1> result = trans * t;
             return result;
           })
      .def("__mul__",
           [](CLS& trans, TranslationMatrixType& t) { return trans * t; })

      .def("rows", &CLS::rows)
      .def("cols", &CLS::cols)

      .def("__call__",
           [](CLS& trans, int row, int col) { return trans(row, col); })
      //.def("__getitem__", )

      .def("matrix", static_cast<MatrixType& (CLS::*)()>(&CLS::matrix))
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
                               RotationMatrixType r =
                                   matrix.block(0, 0, CLS::Dim, CLS::Dim);
                               return r;
                             })

      .def("Quaternion",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<Scalar, 3, 3> r =
                 Eigen::Matrix<Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) =
                 matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      .def("rotation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<Scalar, 3, 3> r =
                 Eigen::Matrix<Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) =
                 matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      .def("orientation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<Scalar, 3, 3> r =
                 Eigen::Matrix<Scalar, 3, 3>::Identity();
             r.block(0, 0, CLS::Dim, CLS::Dim) =
                 matrix.block(0, 0, CLS::Dim, CLS::Dim);
             return Eigen::Quaterniond(r);
           })
      // .def_property_readonly("q", [](CLS& trans) {
      //         MatrixType matrix = trans.matrix();
      //         Eigen::Matrix<_Scalar, 3, 3> r = Eigen::Matrix<_Scalar, 3,
      //         3>::Identity(); r.block(0, 0, CLS::Dim, CLS::Dim) =
      //         matrix.block(0, 0, CLS::Dim, CLS::Dim); return
      //         Eigen::Quaterniond(r);
      //     })

      .def("translation",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<Scalar, CLS::Dim, 1> t =
                 matrix.block(0, CLS::Dim, CLS::Dim, 1);
             return t;
           })
      .def("position",
           [](CLS& trans) {
             MatrixType matrix = trans.matrix();
             Eigen::Matrix<Scalar, CLS::Dim, 1> t =
                 matrix.block(0, CLS::Dim, CLS::Dim, 1);
             return t;
           })
      .def_property_readonly("t",
                             [](CLS& trans) {
                               MatrixType matrix = trans.matrix();
                               Eigen::Matrix<Scalar, CLS::Dim, 1> t =
                                   matrix.block(0, CLS::Dim, CLS::Dim, 1);
                               return t;
                             })

      .def("inverse", &CLS::inverse,
           "traits"_a = static_cast<Eigen::TransformTraits>(CLS::Mode))
      .def("make_affine", &CLS::makeAffine);
}

}  // namespace

void declareEigenTypes(py::module& m) {
  py::class_<Eigen::Quaterniond>(m, "Quaternion")
      //.def(py::init<>())
      .def(py::init([]() { return Eigen::Quaterniond::Identity(); }))

      .def(py::init<const Eigen::Quaterniond&>())
      .def(py::init<const Eigen::AngleAxisd&>())
      .def(py::init<const Eigen::Matrix<double, 3, 3>&>())
      .def(py::init<const double&, const double&, const double&,
                    const double&>(),
           "w"_a, "x"_a, "y"_a, "z"_a)

      .def(py::init([](const Eigen::Matrix<double, 4, 1>& m) {
        return g2o::make_unique<Eigen::Quaterniond>(m(0), m(1), m(2), m(3));
      }))

      .def_static(
          "from_two_vectors",
          [](Eigen::Matrix<double, 3, 1>& a, Eigen::Matrix<double, 3, 1>& b) {
            return Eigen::Quaterniond::FromTwoVectors(a, b);
          })

      .def("x", static_cast<const double& (Eigen::Quaterniond::*)() const>(
                    &Eigen::Quaterniond::x))
      .def("y", static_cast<const double& (Eigen::Quaterniond::*)() const>(
                    &Eigen::Quaterniond::y))
      .def("z", static_cast<const double& (Eigen::Quaterniond::*)() const>(
                    &Eigen::Quaterniond::z))
      .def("w", static_cast<const double& (Eigen::Quaterniond::*)() const>(
                    &Eigen::Quaterniond::w))

      .def("vec", static_cast<const Eigen::VectorBlock<
                      const Eigen::Quaterniond::Coefficients, 3> (
                      Eigen::Quaterniond::*)() const>(&Eigen::Quaterniond::vec))

      .def_static("identity", &Eigen::Quaterniond::Identity)
      .def("set_identity", [](Eigen::Quaterniond& q) { q.setIdentity(); })

      .def("rotation_matrix", &Eigen::Quaterniond::toRotationMatrix)
      .def("matrix", &Eigen::Quaterniond::toRotationMatrix)
      .def_property_readonly("R", &Eigen::Quaterniond::toRotationMatrix)

      .def("squared_norm", &Eigen::Quaterniond::squaredNorm)
      .def("norm", &Eigen::Quaterniond::norm)
      .def("normalize", &Eigen::Quaterniond::normalize)
      .def("normalized", &Eigen::Quaterniond::normalized)
      .def("dot", [](Eigen::Quaterniond& q1,
                     Eigen::Quaterniond& q2) { return q1.dot(q2); })
      .def("angular_distance",
           [](Eigen::Quaterniond& q1, Eigen::Quaterniond& q2) {
             return q1.angularDistance(q2);
           })
      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def("inverse", &Eigen::Quaterniond::inverse)
      .def("conjugate", &Eigen::Quaterniond::conjugate)
      .def("coeffs",
           static_cast<Eigen::Quaterniond::Coefficients& (
               Eigen::Quaterniond::*)()>(
               &Eigen::Quaterniond::coeffs))  // x, y, z, w

      .def("__mul__",
           [](Eigen::Quaterniond& q, Eigen::Matrix<double, 3, 1>& t) {
             // Eigen::Matrix<double,3,1> result = q * t;
             Eigen::Matrix<double, 3, 1> result = q._transformVector(t);
             return result;
           })
      //.def("__mul__", [](Eigen::Quaterniond& q,
      // Eigen::Matrix<double,3,Eigen::Dynamic>& t) {
      //        return q * t;
      //    })

      ;

  py::class_<Eigen::Rotation2Dd>(m, "Rotation2d")
      //.def(py::init<>())
      .def(py::init([]() { return Eigen::Rotation2Dd::Identity(); }))
      .def(py::init<Eigen::Rotation2Dd&>())
      .def(py::init<const double&>())
      .def(py::init<const Eigen::Matrix<double, 2, 2>&>())

      .def("angle", static_cast<double& (Eigen::Rotation2Dd::*)()>(
                        &Eigen::Rotation2Dd::angle))
      .def("smallest_positive_angle",
           &Eigen::Rotation2Dd::smallestPositiveAngle)
      .def("smallest_angle", &Eigen::Rotation2Dd::smallestAngle)
      .def("inverse", &Eigen::Rotation2Dd::inverse)
      .def(py::self * py::self)
      .def(py::self *= py::self)
      .def(py::self * Eigen::Matrix<double, 2, 1>())
      //.def("from_rotation_matrix", (Eigen::Rotation2Dd&
      //(Eigen::Rotation2Dd::*) (const Eigen::Matrix<double, 2, 2>&))
      // &Eigen::Rotation2Dd::fromRotationMatrix)
      // TODO(goki): Figure out if below should be static
      .def("from_rotation_matrix",
           [](Eigen::Rotation2Dd& r, const Eigen::Matrix<double, 2, 2>& R) {
             r.fromRotationMatrix(R);
           })
      .def("to_rotation_matrix", &Eigen::Rotation2Dd::toRotationMatrix)
      .def("rotation_matrix", &Eigen::Rotation2Dd::toRotationMatrix)
      .def("matrix", &Eigen::Rotation2Dd::toRotationMatrix)
      .def_property_readonly("R", &Eigen::Rotation2Dd::toRotationMatrix)
      .def("slerp", &Eigen::Rotation2Dd::slerp)
      .def_static("ientity", &Eigen::Rotation2Dd::Identity);

  py::class_<Eigen::AngleAxisd>(m, "AngleAxis")
      .def(py::init([]() { return Eigen::AngleAxisd::Identity(); }))
      .def(py::init<const double&, const Eigen::Matrix<double, 3, 1>&>())
      .def(py::init<const Eigen::AngleAxisd&>())
      .def(py::init<const Eigen::Quaterniond&>())
      .def(py::init<const Eigen::Matrix<double, 3, 3>&>())
      .def("angle", static_cast<double& (Eigen::AngleAxisd::*)()>(
                        &Eigen::AngleAxisd::angle))
      .def("axis",
           static_cast<Eigen::Matrix<double, 3, 1>& (Eigen::AngleAxisd::*)()>(
               &Eigen::AngleAxisd::axis))
      .def(py::self * py::self)
      .def(py::self * Eigen::Quaterniond())
      .def(Eigen::Quaterniond() * py::self)
      .def("inverse", &Eigen::AngleAxisd::inverse)
      //.def("from_rotation_matrix", &Eigen::AngleAxisd::fromRotationMatrix)
      // TODO(goki): Figure out if below should be static
      .def("from_rotation_matrix",
           [](Eigen::AngleAxisd& r, const Eigen::Matrix<double, 3, 3>& R) {
             r.fromRotationMatrix(R);
           })
      .def("to_rotation_matrix", &Eigen::AngleAxisd::toRotationMatrix)
      .def("rotation_matrix", &Eigen::AngleAxisd::toRotationMatrix)
      .def("matrix", &Eigen::AngleAxisd::toRotationMatrix)
      .def_property_readonly("R", &Eigen::AngleAxisd::toRotationMatrix)
      .def_static("ientity", &Eigen::AngleAxisd::Identity);

  py::enum_<Eigen::TransformTraits>(m, "TransformTraits")
      .value("Isometry", Eigen::Isometry)
      .value("Affine", Eigen::Affine)
      .value("AffineCompact", Eigen::AffineCompact)
      .value("Projective", Eigen::Projective)
      .export_values();

  templatedEigenIsometry<double, 2, Eigen::Isometry>(m, "Isometry2d");
  templatedEigenIsometry<double, 3, Eigen::Isometry>(m, "Isometry3d");

  /*templatedEigenTransform<double, 2, Eigen::Projective>(m, "Projective2d");
  templatedEigenTransform<double, 3, Eigen::Projective>(m, "Projective3d");

  templatedEigenTransform<double, 2, Eigen::Affine>(m, "Affine2d");
  templatedEigenTransform<double, 3, Eigen::Affine>(m, "Affine3d");*/
}

}  // end namespace g2o
