#include "py_io_format.h"

#include "g2o/core/io/io_format.h"
#include "g2opy.h"

namespace g2o {

namespace {
struct IoWrapper {};
}  // namespace

void declareIOFormat(py::module_& m) {
  py::enum_<io::Format>(m, "IoFormat", "enum.Enum")
      .value("G2O", io::Format::kG2O, "G2O's custom ASCII IO format")
      .value("JSON", io::Format::kJson, "JSON IO format")
      .value("BINARY", io::Format::kBinary, "G2O's custom binary format")
      .export_values();

  py::class_<io::FileFilter>(m, "IoFileFilter")
      .def(py::init<std::string, io::Format>(), "filter"_a, "format"_a)
      .def_rw("filter", &io::FileFilter::filter)
      .def_rw("format", &io::FileFilter::format);

  py::class_<IoWrapper>(m, "IoWrapper")
      .def_static(
          "to_string",
          [](io::Format format) { return std::string(io::to_string(format)); })
      .def_static("file_filter", &io::getFileFilter)
      .def_static("format_for_file_extension",
                  [](const std::string& extension) {
                    return io::formatForFileExtension(extension);
                  });
}

}  // end namespace g2o
