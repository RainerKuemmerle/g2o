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

#include "g2o_qglviewer.h"
#include "g2o/stuff/property.h"

#include <QLineEdit>

#include <iostream>
#include <cassert>

#ifdef __GNUC__
  #include <cxxabi.h>
#endif

using namespace std;

using namespace g2o;

/**
 * demangle the name of a type, depends on the used compiler
 */
static std::string demangleName(const std::string& fullPropName)
{
#ifdef __GNUC__
  // find :: and extract the mangled class name from the whole string
  string mangledName;
  string propName;
  string::size_type found = fullPropName.rfind("::");
  if (found != string::npos) {
    mangledName = fullPropName.substr(0, found);
    propName    = fullPropName.substr(found);
  } else {
    mangledName = propName;
  }

  int status;
  char* s = abi::__cxa_demangle(mangledName.c_str(), 0, 0, &status);
  if (status != 0) {
    free(s);
    return fullPropName;
  } else {
    std::string demangled(s);
    free(s);
    return demangled + propName;
  }
#else
  // TODO for other compilers
  return fullPropName;
#endif
}

ViewerPropertiesWidget::ViewerPropertiesWidget(QWidget * parent, Qt::WindowFlags f) :
  PropertiesWidget(parent, f)
{
}

ViewerPropertiesWidget::~ViewerPropertiesWidget()
{
}

void ViewerPropertiesWidget::applyProperties()
{
  PropertiesWidget::applyProperties();

  // draw with the new properties
  _viewer->setUpdateDisplay(true);
  _viewer->updateGL();
}

void ViewerPropertiesWidget::setViewer(g2o::G2oQGLViewer* viewer)
{
  _viewer = viewer;
  setProperties(viewer->parameters());
}

std::string ViewerPropertiesWidget::humanReadablePropName(const std::string& propertyName) const
{
  return demangleName(propertyName);
}
