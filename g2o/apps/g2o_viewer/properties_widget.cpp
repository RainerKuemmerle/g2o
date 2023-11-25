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

#include "properties_widget.h"

#include "g2o/core/optimization_algorithm.h"
#include "g2o/stuff/property.h"

PropertiesWidget::PropertiesWidget(QWidget* parent)
    : AbstractPropertiesWidget(parent) {
  std::cerr << "Created Prop widget\n";
}

void PropertiesWidget::setSolver(
    std::shared_ptr<g2o::OptimizationAlgorithm> solver) {
  solver_ = std::move(solver);
  updateDisplayedProperties();
}

g2o::PropertyMap* PropertiesWidget::propertyMap() {
  if (!solver_) return nullptr;
  // HACK cast away the constness of the property map
  return const_cast<g2o::PropertyMap*>(&solver_->properties());
}
