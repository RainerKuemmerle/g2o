#include "py_io_format.h"

#include <pybind11/native_enum.h>

#include "g2o/core/io/io_format.h"

namespace g2o {

namespace {
struct IoWrapper {};
}  // namespace

void declareIOFormat(py::module& m) {
  py::native_enum<io::Format>(m, "IoFormat", "enum.Enum")
      .value("G2O", io::Format::kG2O, "G2O's custom ASCII IO format")
      .value("JSON", io::Format::kJson, "JSON IO format")
      .value("BINARY", io::Format::kBinary, "G2O's custom binary format")
      .export_values()
      .finalize();

  py::classh<io::FileFilter>(m, "IoFileFilter")
      .def(py::init<std::string, io::Format>(), "filter"_a, "format"_a)
      .def_readwrite("filter", &io::FileFilter::filter)
      .def_readwrite("format", &io::FileFilter::format);

  py::classh<IoWrapper>(m, "IoWrapper")
      .def_static("to_string", &io::to_string)
      .def_static("file_filter", &io::getFileFilter)
      .def_static("format_for_file_extension", &io::formatForFileExtension);
}

}  // end namespace g2o
