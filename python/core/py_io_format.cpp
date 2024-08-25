#include "py_io_format.h"

#include "g2o/core/io/io_format.h"

namespace g2o {

namespace {
struct IoWrapper {};
}  // namespace

void declareIOFormat(py::module& m) {
  py::enum_<io::Format>(m, "IoFormat")
      .value("G2O", io::Format::kG2O, "G2O's custom ASCII IO format")
      .value("JSON", io::Format::kJson, "JSON IO format")
      .value("BINARY", io::Format::kBinary, "G2O's custom binary format");

  py::class_<io::FileFilter>(m, "IoFileFilter")
      .def(py::init<std::string, io::Format>(), "filter"_a, "format"_a)
      .def_readwrite("filter", &io::FileFilter::filter)
      .def_readwrite("format", &io::FileFilter::format);

  py::class_<IoWrapper>(m, "IoWrapper")
      .def_static("to_string", &io::to_string)
      .def_static("file_filter", &io::getFileFilter)
      .def_static("format_for_file_extension", &io::formatForFileExtension);
}

}  // end namespace g2o
