#include "py_io_format.h"

#include "g2o/core/io/io_format.h"

namespace g2o {

void declareIOFormat(py::module& m) {
  py::enum_<io::Format>(m, "IoFormat")
      .value("G2O", io::Format::kG2O, "G2O's custom ASCII IO format")
      .value("JSON", io::Format::kJson, "JSON IO format")
      .value("BINARY", io::Format::kBinary, "G2O's custom binary format")
      .value("XML", io::Format::kXML, "XML IO format");
}

}  // end namespace g2o
