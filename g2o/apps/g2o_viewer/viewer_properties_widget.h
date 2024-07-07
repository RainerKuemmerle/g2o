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

#ifndef G2O_VIEWER_PROPERTIES_WIDGET_H
#define G2O_VIEWER_PROPERTIES_WIDGET_H

#include <memory>

#include "abstract_properties_widget.h"
#include "g2o_viewer_api.h"

namespace g2o {
class G2oQGLViewer;
class PropertyMap;
}  // namespace g2o

class G2O_VIEWER_API ViewerPropertiesWidget : public AbstractPropertiesWidget {
 public:
  explicit ViewerPropertiesWidget(QWidget* parent = nullptr);
  ~ViewerPropertiesWidget() override = default;

  void setViewer(g2o::G2oQGLViewer* viewer);
  const g2o::PropertyMap* propertyMap() override;

 protected:
  g2o::G2oQGLViewer* viewer_ = nullptr;
  std::shared_ptr<g2o::PropertyMap> properties_;

  void applyProperties() override;
  [[nodiscard]] std::string humanReadablePropName(
      const std::string& propertyName) const override;
};

#endif
