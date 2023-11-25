// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
//
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#include "viewer_properties_widget.h"

#include "g2o/stuff/property.h"
#include "g2o_qglviewer.h"

#ifdef __GNUC__
#include <cxxabi.h>
#endif

namespace {
/**
 * demangle the name of a type, depends on the used compiler
 */
std::string demangleName(const std::string& fullPropName) {
#ifdef __GNUC__
  // find :: and extract the mangled class name from the whole string
  std::string mangledName;
  std::string propName;
  std::string::size_type found = fullPropName.rfind("::");
  if (found != std::string::npos) {
    mangledName = fullPropName.substr(0, found);
    propName = fullPropName.substr(found);
  } else {
    mangledName = propName;
  }

  int status;
  char* s = abi::__cxa_demangle(mangledName.c_str(), nullptr, nullptr, &status);
  if (status != 0) {
    free(s);
    return fullPropName;
  }
  std::string demangled(s);
  free(s);
  return demangled + propName;

#else
  // TODO for other compilers
  return fullPropName;
#endif
}
}  // namespace

ViewerPropertiesWidget::ViewerPropertiesWidget(QWidget* parent)
    : AbstractPropertiesWidget(parent) {}

void ViewerPropertiesWidget::applyProperties() {
  AbstractPropertiesWidget::applyProperties();

  // draw with the new properties
  viewer_->setUpdateDisplay(true);
  viewer_->update();
}

void ViewerPropertiesWidget::setViewer(g2o::G2oQGLViewer* viewer) {
  viewer_ = viewer;
  updateDisplayedProperties();
}

std::string ViewerPropertiesWidget::humanReadablePropName(
    const std::string& propertyName) const {
  return demangleName(propertyName);
}

g2o::PropertyMap* ViewerPropertiesWidget::propertyMap() {
  if (!viewer_) return nullptr;
  return viewer_->parameters().get();
}
