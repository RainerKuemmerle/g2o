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

#ifndef G2O_PROPERTIES_WINDOW_H
#define G2O_PROPERTIES_WINDOW_H

#include "abstract_properties_widget.h"

namespace g2o {
class OptimizationAlgorithm;
}  // namespace g2o

/**
 * @brief Widget for displaying properties of an OptimizationAlgorithm
 */
class G2O_VIEWER_API PropertiesWidget : public AbstractPropertiesWidget {
 public:
  explicit PropertiesWidget(QWidget* parent = nullptr);
  ~PropertiesWidget() override = default;

  const g2o::PropertyMap* propertyMap() override;

  void setSolver(std::shared_ptr<g2o::OptimizationAlgorithm> solver);

 protected:
  std::shared_ptr<g2o::OptimizationAlgorithm> solver_;
};

#endif
